//! Intake plenum: the box between the throttle and the intake runners, modelled along its length.
//!
//! It is finite so that gas back-flowing up an intake valve during overlap is held and handed back on
//! the next intake stroke. That residual dilution is the dominant reason a real engine's exhaust
//! temperature collapses at light load, so the composition is tracked as well as the mass.
//!
//! And it is long: a pressure wave takes a millisecond or so to cross it, so the cylinders at its far end
//! do not see what those by the throttle do, and the box rings along its length. So it is a row of zones
//! from the throttle at its front to its back, each a finite volume of its own, the gas between each and
//! the next carried by its momentum: the throttle feeds the first, and each runner draws from the zone it
//! leaves by. Its section narrows towards the back by its taper, as the drawn plenum does.

use crate::intake::Runner;
use crate::math::{self, PI, clamp};
use crate::spec::{
    EngineSpec, crank_pins, cylinder_spacing, cylinder_z, displacement, gas, gas_energy, gas_enthalpy, gas_gamma,
    gas_temperature, intake_runner_of,
};
use crate::valve::orifice_mass_flow;

/// Floor on a zone's mass, kg.
const MIN_MASS: f64 = 1e-7;

/// Throttle area at the closed stop, as a fraction of the full bore: leakage and the idle bypass.
const IDLE_BYPASS: f64 = 0.002;

/// Discharge coefficient of the butterfly, closed and wide open.
const CD_CLOSED: f64 = 0.25;
const CD_OPEN: f64 = 0.75;

/// Air velocity through a wide-open throttle at peak rpm, m/s, used to size the bore.
const THROTTLE_DESIGN_VELOCITY: f64 = 25.0;
const THROTTLE_DESIGN_RPM: f64 = 7000.0;

/// Plenum volume as a multiple of total swept volume, when not given explicitly.
const PLENUM_VOLUME_RATIO: f64 = 1.5;

/// The drawn plenum's least height, its rounded edges, and the throttle body's wall, m: what an auto-sized
/// plenum's height and length are worked out from, as the app draws it (`inletLayout.ts`).
const PLENUM_HEIGHT: f64 = 0.1;
const PLENUM_ROUNDING: f64 = 0.012;
const THROTTLE_WALL: f64 = 0.006;

/// How sharply the plenum rings along its length: the quality factor of its lowest mode. A real plenum's
/// waves lose much of their strength each pass, to the runner mouths along it, the throttle's jet and the
/// turbulence they stir, far more than a smooth tube's walls would take: a plenum rings for a few cycles,
/// not tens.
const PLENUM_Q: f64 = 8.0;
/// How quickly the flow along it is followed for its steady part, s: its waves are damped, not the steady
/// flow the throttle feeds the runners, which would otherwise meet a drag no plenum has.
const STEADY_FLOW_TIME: f64 = 0.05;

/// How fast the plenum's gas mixes along it, m^2/s: the turbulence the throttle's jet and the runners'
/// pulses stir. Without it the hot, spent gas a cylinder pushes back up its runner at overlap would sit in
/// its zone until that cylinder drew it straight back in, where in a real plenum it spreads through the
/// box within a cycle or two.
const PLENUM_MIXING: f64 = 0.5;

/// How long each zone along the plenum is meant to be, m, and the most there are.
const ZONE_LENGTH: f64 = 0.03;
const MAX_ZONES: usize = 32;
/// Fastest sound the plenum's gas carries, m/s, hot backflow and all, and the most of a zone a wave may
/// cross in a sample: what keeps the zones from being so short the explicit step goes unstable.
const MAX_SOUND_SPEED: f64 = 450.0;
const MAX_COURANT: f64 = 0.5;

fn total_displacement(spec: &EngineSpec) -> f64 {
    displacement(spec) * math::max(spec.cylinders as f64, 1.0)
}

/// Throttle bore, m: the spec's, or sized to pass peak airflow at the design velocity.
pub fn throttle_dia_of(spec: &EngineSpec) -> f64 {
    if spec.throttle_dia > 0.0 {
        return spec.throttle_dia;
    }
    let area = (total_displacement(spec) * (THROTTLE_DESIGN_RPM / 120.0)) / THROTTLE_DESIGN_VELOCITY;
    math::sqrt((4.0 * area) / PI)
}

/// A plenum's size, m: along the engine, across and high at its front, and how much of its section it has
/// lost at its back, a fraction, narrowing evenly from none at the front.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlenumShape {
    pub length: f64,
    pub width: f64,
    pub height: f64,
    pub taper: f64,
}

impl PlenumShape {
    /// Its section `s` m back from its front, m^2.
    pub fn area_at(&self, s: f64) -> f64 {
        self.width * self.height * (1.0 - self.taper * clamp(s / self.length, 0.0, 1.0))
    }

    pub fn volume(&self) -> f64 {
        self.width * self.height * self.length * (1.0 - self.taper / 2.0)
    }
}

/// The plenum's size: the spec's, each that is 0 or less worked out, as `intakeSizing.ts` works it out
/// for the drawn one. Its length runs past every runner by its flared mouth, along most of the engine; its
/// height takes the throttle body's flange; its width holds `plenum_volume`, or one and a half times the
/// engine's displacement, at that length and height. A width or height given is no less than the flange.
pub fn plenum_shape_of(spec: &EngineSpec) -> PlenumShape {
    let taper = clamp(spec.plenum_taper, 0.0, 0.8);
    let length = if spec.plenum_length > 0.0 {
        spec.plenum_length
    } else {
        let spacing = cylinder_spacing(spec);
        let pins = crank_pins(spec).len() as f64;
        let block = (pins - 1.0) * spacing + math::max(spacing, spec.bore * 1.3);
        let reach = (0..spec.cylinders as usize).map(|c| cylinder_z(spec, c).abs()).fold(0.0, f64::max);
        let last = reach + 1.5 * intake_runner_of(spec).diameter / 2.0 + PLENUM_ROUNDING + 0.006;
        math::max(block * 0.85, 2.0 * last)
    };
    // No smaller across or high than takes the throttle body's flange on its front.
    let face = throttle_dia_of(spec) + 2.0 * THROTTLE_WALL + 2.0 * PLENUM_ROUNDING + 0.01;
    let height = if spec.plenum_height > 0.0 { math::max(spec.plenum_height, face) } else { math::max(PLENUM_HEIGHT, face) };
    let width = if spec.plenum_width > 0.0 {
        math::max(spec.plenum_width, face)
    } else {
        let target = if spec.plenum_volume > 0.0 { spec.plenum_volume } else { PLENUM_VOLUME_RATIO * total_displacement(spec) };
        target / (height * length * (1.0 - taper / 2.0))
    };
    PlenumShape { length, width, height, taper }
}

/// Plenum volume, m^3, of the plenum as `plenum_shape_of` sizes it.
pub fn plenum_volume_of(spec: &EngineSpec) -> f64 {
    plenum_shape_of(spec).volume()
}

/// One zone along the plenum: its gas, and its share of the volume.
#[derive(Clone, Copy, Debug)]
struct Zone {
    mass: f64,
    /// Sensible internal energy, J.
    energy: f64,
    burned_mass: f64,
    fuel_mass: f64,
    volume: f64,
}

impl Zone {
    fn temp(&self) -> f64 {
        clamp(gas_temperature(self.energy / math::max(self.mass, MIN_MASS)), 150.0, 3000.0)
    }

    fn pressure(&self) -> f64 {
        (math::max(self.mass, MIN_MASS) * gas::R * self.temp()) / self.volume
    }

    fn burned(&self) -> f64 {
        clamp(self.burned_mass / math::max(self.mass, MIN_MASS), 0.0, 1.0)
    }

    fn fuel(&self) -> f64 {
        clamp(self.fuel_mass / math::max(self.mass, MIN_MASS), 0.0, 1.0)
    }

    fn ambient(volume: f64) -> Zone {
        let mass = (gas::P_AMB * volume) / (gas::R * gas::T_AMB);
        Zone { mass, energy: mass * gas_energy(gas::T_AMB), burned_mass: 0.0, fuel_mass: 0.0, volume }
    }
}

/// What a runner draws from: its zone's pressure, Pa, density, kg/m^3, and spent-gas and fuel fractions.
#[derive(Clone, Copy, Debug, Default)]
pub struct PlenumFeed {
    pub p: f64,
    pub rho: f64,
    pub burned: f64,
    pub fuel: f64,
}

pub struct IntakePlenum {
    shape: PlenumShape,
    sample_rate: f64,
    /// Front to back.
    zones: Vec<Zone>,
    /// Mass flow from each zone into the next back, kg/s, its steady part, and the section it flows
    /// through, m^2.
    flows: Vec<f64>,
    steady: Vec<f64>,
    face_areas: Vec<f64>,
    /// How long each zone is, m, and how fast the waves in the flow between them die away, 1/s.
    dx: f64,
    damping: f64,
    /// The zone each cylinder's runner leaves by.
    attach: Vec<usize>,
    /// The throttle plate's opening, 0..1, and the idle air valve's, as more of the plate's.
    opening: f64,
    bypass: f64,
    /// Effective throttle area, m^2.
    area: f64,
    /// Each zone's change over a sample, worked out before any is applied: mass, energy, spent gas, fuel.
    change: Vec<[f64; 4]>,
}

impl IntakePlenum {
    pub fn new(spec: &EngineSpec, sample_rate: f64) -> IntakePlenum {
        let mut plenum = IntakePlenum {
            shape: plenum_shape_of(spec),
            sample_rate,
            zones: Vec::new(),
            flows: Vec::new(),
            steady: Vec::new(),
            face_areas: Vec::new(),
            dx: 0.0,
            damping: 0.0,
            attach: Vec::new(),
            opening: spec.throttle,
            bypass: 0.0,
            area: IntakePlenum::throttle_area(spec),
            change: Vec::new(),
        };
        plenum.lay_out(spec);
        for z in plenum.zones.iter_mut() {
            *z = Zone::ambient(z.volume);
        }
        plenum
    }

    /// Divide the plenum into zones along its length, each as near `ZONE_LENGTH` long as a wave crossing
    /// it in a sample allows, and find the zone each runner leaves by. The zones are left holding nothing.
    fn lay_out(&mut self, spec: &EngineSpec) {
        let shape = self.shape;
        let length = math::max(shape.length, 1e-3);
        let shortest = (MAX_SOUND_SPEED / self.sample_rate) / MAX_COURANT;
        let n = clamp((length / ZONE_LENGTH).round(), 1.0, math::max((length / shortest).floor(), 1.0)) as usize;
        let n = n.min(MAX_ZONES);
        let dx = length / n as f64;
        let floor = 1e-5 / n as f64;
        self.zones = (0..n)
            .map(|i| Zone { mass: 0.0, energy: 0.0, burned_mass: 0.0, fuel_mass: 0.0, volume: math::max(shape.area_at((i as f64 + 0.5) * dx) * dx, floor) })
            .collect();
        self.face_areas = (1..n).map(|i| math::max(shape.area_at(i as f64 * dx), 1e-6)).collect();
        self.flows = vec![0.0; n.saturating_sub(1)];
        self.steady = vec![0.0; n.saturating_sub(1)];
        self.change = vec![[0.0; 4]; n];
        self.dx = dx;
        let c = math::sqrt(gas::GAMMA_AIR * gas::R * gas::T_AMB);
        self.damping = (2.0 * PI * (c / (2.0 * length))) / PLENUM_Q;
        // The runners leave it where their cylinders are along the engine, its middle the engine's.
        self.attach = (0..spec.cylinders as usize)
            .map(|c| (((cylinder_z(spec, c) + length / 2.0) / dx).floor() as isize).clamp(0, n as isize - 1) as usize)
            .collect();
    }

    /// Set the throttle to `opening`, 0..1, in place of the spec's, until the next `set_geometry`.
    pub fn set_opening(&mut self, spec: &EngineSpec, opening: f64) {
        self.opening = opening;
        self.area = IntakePlenum::throttle_area_at(spec, self.opening + self.bypass);
    }

    /// Open the idle air valve, the bypass round the throttle plate, by `bypass`: as much air as that
    /// much more of the plate's opening would pass.
    pub fn set_bypass(&mut self, spec: &EngineSpec, bypass: f64) {
        if bypass == self.bypass {
            return;
        }
        self.bypass = bypass;
        self.area = IntakePlenum::throttle_area_at(spec, self.opening + self.bypass);
    }

    /// Rebuild geometry in place. Its gas is kept: at the same state, where only the throttle changed, or
    /// at its mean pressure, temperature and composition through a plenum of a new shape.
    pub fn set_geometry(&mut self, spec: &EngineSpec) {
        self.opening = spec.throttle;
        self.area = IntakePlenum::throttle_area_at(spec, self.opening + self.bypass);
        let shape = plenum_shape_of(spec);
        if shape == self.shape && self.attach.len() == spec.cylinders as usize {
            return;
        }
        let total = |f: fn(&Zone) -> f64| self.zones.iter().map(f).sum::<f64>();
        let (mass, energy, burned, fuel, volume) =
            (total(|z| z.mass), total(|z| z.energy), total(|z| z.burned_mass), total(|z| z.fuel_mass), total(|z| z.volume));
        self.shape = shape;
        self.lay_out(spec);
        // Each zone at the mean density and state the old plenum held.
        for z in self.zones.iter_mut() {
            let share = z.volume / math::max(volume, 1e-12);
            z.mass = mass * share;
            z.energy = energy * share;
            z.burned_mass = burned * share;
            z.fuel_mass = fuel * share;
        }
    }

    /// Its zones, front to back: how many.
    pub fn zone_count(&self) -> usize {
        self.zones.len()
    }

    /// Gauge pressure in each zone, front to back, Pa.
    pub fn zone_pressures(&self) -> impl Iterator<Item = f64> + '_ {
        self.zones.iter().map(|z| z.pressure() - gas::P_AMB)
    }

    /// What cylinder `c`'s runner draws from.
    pub fn feed(&self, c: usize) -> PlenumFeed {
        let z = &self.zones[self.attach.get(c).copied().unwrap_or(0)];
        let p = z.pressure();
        PlenumFeed { p, rho: p / (gas::R * z.temp()), burned: z.burned(), fuel: z.fuel() }
    }

    /// Its mean temperature, K.
    pub fn temp(&self) -> f64 {
        let mass: f64 = self.zones.iter().map(|z| z.mass).sum();
        let energy: f64 = self.zones.iter().map(|z| z.energy).sum();
        clamp(gas_temperature(energy / math::max(mass, MIN_MASS)), 150.0, 3000.0)
    }

    /// Its mean absolute pressure, Pa, over its volume.
    pub fn pressure(&self) -> f64 {
        let volume: f64 = self.zones.iter().map(|z| z.volume).sum();
        self.zones.iter().map(|z| z.pressure() * z.volume).sum::<f64>() / volume
    }

    /// Absolute pressure by the throttle, at its front, Pa.
    pub fn throttle_pressure(&self) -> f64 {
        self.zones[0].pressure()
    }

    pub fn burned_fraction(&self) -> f64 {
        let mass: f64 = self.zones.iter().map(|z| z.mass).sum();
        clamp(self.zones.iter().map(|z| z.burned_mass).sum::<f64>() / math::max(mass, MIN_MASS), 0.0, 1.0)
    }

    pub fn fuel_fraction(&self) -> f64 {
        let mass: f64 = self.zones.iter().map(|z| z.mass).sum();
        clamp(self.zones.iter().map(|z| z.fuel_mass).sum::<f64>() / math::max(mass, MIN_MASS), 0.0, 1.0)
    }

    /// Effective throttle flow area, m^2: geometric area times the plate's discharge coefficient.
    pub fn throttle_area(spec: &EngineSpec) -> f64 {
        IntakePlenum::throttle_area_at(spec, spec.throttle)
    }

    pub fn throttle_area_at(spec: &EngineSpec, opening: f64) -> f64 {
        let d = throttle_dia_of(spec);
        let bore = (PI * d * d) / 4.0;
        let open = 1.0 - math::cos(clamp(opening, 0.0, 1.0) * (PI / 2.0));
        let geometric = bore * (IDLE_BYPASS + (1.0 - IDLE_BYPASS) * open);
        geometric * (CD_CLOSED + (CD_OPEN - CD_CLOSED) * open)
    }

    /// Effective throttle flow area, m^2, as the plate is set.
    pub fn area(&self) -> f64 {
        self.area
    }

    /// Advance by `dt`, drawing through the throttle into the front zone from air at `p_up` (Pa) and
    /// `t_up` (K): the atmosphere, or a turbocharger's charge air. Each of `runners`, a cylinder's, takes
    /// from or gives back to the zone it leaves by what flowed out of its plenum end over the sample, at
    /// its own temperature and composition where it flows back in. Returns the flow in through the
    /// throttle, kg/s.
    pub fn step(&mut self, dt: f64, p_up: f64, t_up: f64, runners: &[Runner]) -> f64 {
        let n = self.zones.len();
        for c in self.change.iter_mut() {
            *c = [0.0; 4];
        }

        // The flow between each zone and the next back, driven by the pressure across it, its waves about
        // its steady part losing their strength, taken implicitly.
        let follow = dt / STEADY_FLOW_TIME;
        for f in 0..n.saturating_sub(1) {
            let push = (self.face_areas[f] / self.dx) * (self.zones[f].pressure() - self.zones[f + 1].pressure());
            let steady = self.steady[f];
            self.flows[f] = steady + (self.flows[f] - steady + dt * push) / (1.0 + dt * self.damping);
            self.steady[f] += follow * (self.flows[f] - steady);
        }

        // The throttle, into the front zone.
        let front = self.zones[0];
        let (p, t) = (front.pressure(), front.temp());
        let area = self.area;
        let throttle_flow = if p < p_up {
            orifice_mass_flow(area, 1.0, p_up, t_up, p, gas::GAMMA_AIR)
        } else {
            -orifice_mass_flow(area, 1.0, p, t, p_up, gas_gamma(t))
        };
        {
            let c = &mut self.change[0];
            c[0] += throttle_flow;
            c[1] += throttle_flow * gas_enthalpy(if throttle_flow >= 0.0 { t_up } else { t });
            if throttle_flow < 0.0 {
                c[2] += throttle_flow * front.burned();
                c[3] += throttle_flow * front.fuel();
            }
        }

        // From each zone to the next, at the state of the one it leaves.
        for f in 0..n.saturating_sub(1) {
            let q = self.flows[f];
            let from = if q >= 0.0 { self.zones[f] } else { self.zones[f + 1] };
            let moved = [q, q * gas_enthalpy(from.temp()), q * from.burned(), q * from.fuel()];
            for k in 0..4 {
                self.change[f][k] -= moved[k];
                self.change[f + 1][k] += moved[k];
            }
        }

        // And mixing between each zone and the next: as much gas of each swapped for the other's, so their
        // heat and makeup even out without moving any mass.
        for f in 0..n.saturating_sub(1) {
            let (a, b) = (self.zones[f], self.zones[f + 1]);
            let density = 0.5 * (a.mass / a.volume + b.mass / b.volume);
            let swapped = density * PLENUM_MIXING * self.face_areas[f] / self.dx;
            let per = |z: &Zone| [0.0, z.energy / math::max(z.mass, MIN_MASS), z.burned(), z.fuel()];
            let (pa, pb) = (per(&a), per(&b));
            for k in 1..4 {
                let moved = swapped * (pa[k] - pb[k]);
                self.change[f][k] -= moved;
                self.change[f + 1][k] += moved;
            }
        }

        // The runners: drawing from their zones, or pushing back into them what their cylinders sent up.
        for (c, r) in runners.iter().enumerate() {
            let i = self.attach.get(c).copied().unwrap_or(0).min(n - 1);
            let flow = r.plenum_flow;
            let zone = self.zones[i];
            let moved = if flow >= 0.0 {
                [flow, flow * gas_enthalpy(r.plenum_temp), flow * r.burned, flow * r.fuel]
            } else {
                [flow, flow * gas_enthalpy(zone.temp()), flow * zone.burned(), flow * zone.fuel()]
            };
            for k in 0..4 {
                self.change[i][k] += moved[k];
            }
        }

        let mut broken = false;
        for (z, c) in self.zones.iter_mut().zip(&self.change) {
            let t = z.temp();
            z.mass += c[0] * dt;
            z.energy += c[1] * dt;
            z.burned_mass += c[2] * dt;
            z.fuel_mass += c[3] * dt;
            if z.mass < MIN_MASS {
                z.mass = MIN_MASS;
                z.energy = MIN_MASS * gas_energy(math::max(t, 150.0));
            }
            z.burned_mass = clamp(z.burned_mass, 0.0, z.mass);
            z.fuel_mass = clamp(z.fuel_mass, 0.0, z.mass - z.burned_mass);
            let e_min = z.mass * gas_energy(150.0);
            if z.energy < e_min {
                z.energy = e_min;
            }
            broken |= !z.energy.is_finite() || !z.mass.is_finite();
        }
        if broken || self.flows.iter().chain(&self.steady).any(|q| !q.is_finite()) {
            self.reset();
        }
        throttle_flow
    }

    pub fn reset(&mut self) {
        for z in self.zones.iter_mut() {
            *z = Zone::ambient(z.volume);
        }
        for q in self.flows.iter_mut().chain(self.steady.iter_mut()) {
            *q = 0.0;
        }
    }
}
