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
//!
//! A V or a boxer can have dual plenums instead, as the LT6 does: the casting divided down its middle,
//! each bank's runners drawing from their own half, each half with its own throttle body. Apart, each
//! half sees only its own bank's pulses and rings with them; balance valves through the wall between
//! them join them above a set speed, into one box.

use crate::intake::Runner;
use crate::math::{self, PI, clamp};
use crate::spec::{
    EngineSpec, crank_pins, cylinder_spacing, cylinder_z, displacement, gas, gas_energy, gas_enthalpy, gas_gamma,
    gas_temperature, intake_runner_of, physical_bank, physical_bank_count,
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

/// The wall between dual plenums, m, which a balance valve's air flows through.
const BALANCE_WALL: f64 = 0.008;
/// How long a balance valve takes to swing open or shut, s.
const BALANCE_TRAVEL: f64 = 0.1;
/// How far below its opening speed the balance valve shuts again, rev/min.
const BALANCE_HYSTERESIS: f64 = 150.0;

fn total_displacement(spec: &EngineSpec) -> f64 {
    displacement(spec) * math::max(spec.cylinders as f64, 1.0)
}

/// How many plenums the runners draw from: two with `dual_plenum` on an engine of two banks, each bank's
/// runners from their own, side by side in one casting; otherwise one.
pub fn plenum_count_of(spec: &EngineSpec) -> usize {
    if spec.dual_plenum && physical_bank_count(spec) > 1 { 2 } else { 1 }
}

/// Each throttle body's bore, m: the spec's, or sized for the throttles together to pass peak airflow at
/// the design velocity. There is one on the front of each plenum.
pub fn throttle_dia_of(spec: &EngineSpec) -> f64 {
    if spec.throttle_dia > 0.0 {
        return spec.throttle_dia;
    }
    sized_throttle_dia(spec)
}

/// The bore `throttle_dia_of` sizes each throttle body to, whatever the spec gives, m.
fn sized_throttle_dia(spec: &EngineSpec) -> f64 {
    let area = (total_displacement(spec) * (THROTTLE_DESIGN_RPM / 120.0)) / THROTTLE_DESIGN_VELOCITY;
    math::sqrt((4.0 * area) / (PI * plenum_count_of(spec) as f64))
}

/// A plenum's size, m: along the engine, across and high at its front, and how much of its section it has
/// lost at its back, a fraction, narrowing evenly from none at the front. Dual plenums are one casting
/// this size, divided down its middle.
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
/// engine's displacement, at that length and height. A width or height given is no less than the flange,
/// nor a width less than a flange for each plenum side by side.
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
    // No smaller across or high than takes the throttle bodies' flanges on its front.
    let face = throttle_dia_of(spec) + 2.0 * THROTTLE_WALL + 2.0 * PLENUM_ROUNDING + 0.01;
    let across = face * plenum_count_of(spec) as f64;
    let height = if spec.plenum_height > 0.0 { math::max(spec.plenum_height, face) } else { math::max(PLENUM_HEIGHT, face) };
    let width = if spec.plenum_width > 0.0 {
        math::max(spec.plenum_width, across)
    } else {
        let target = if spec.plenum_volume > 0.0 { spec.plenum_volume } else { PLENUM_VOLUME_RATIO * total_displacement(spec) };
        target / (height * length * (1.0 - taper / 2.0))
    };
    PlenumShape { length, width, height, taper }
}

/// Plenum volume, m^3, of the plenum as `plenum_shape_of` sizes it: dual plenums' together.
pub fn plenum_volume_of(spec: &EngineSpec) -> f64 {
    plenum_shape_of(spec).volume()
}

/// Effective flow area of a throttle plate `d` m across at `opening`, 0..1, m^2: geometric area times the
/// plate's discharge coefficient.
fn plate_area(d: f64, opening: f64) -> f64 {
    let bore = (PI * d * d) / 4.0;
    let open = 1.0 - math::cos(clamp(opening, 0.0, 1.0) * (PI / 2.0));
    let geometric = bore * (IDLE_BYPASS + (1.0 - IDLE_BYPASS) * open);
    geometric * (CD_CLOSED + (CD_OPEN - CD_CLOSED) * open)
}

/// Where along dual plenums the balance valves are, as fractions of their length from the front: two,
/// each a throttle body's bore, through the wall between them.
pub const BALANCE_VALVES: [f64; 2] = [1.0 / 3.0, 2.0 / 3.0];

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

/// One plenum, front to back: its zones, the mass flow from each zone into the next back, kg/s, its
/// steady part, and the section it flows through, m^2.
#[derive(Clone, Debug)]
struct Row {
    zones: Vec<Zone>,
    flows: Vec<f64>,
    steady: Vec<f64>,
    face_areas: Vec<f64>,
    /// Each zone's change over a sample, worked out before any is applied: mass, energy, spent gas, fuel.
    change: Vec<[f64; 4]>,
}

/// A balance valve between dual plenums: the zone of each it opens into, its bore over the length of the
/// air it carries, m, wide open, and its flow from bank 0's plenum into bank 1's, kg/s.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Balance {
    zone: usize,
    reach: f64,
    flow: f64,
}

/// How the plenum is divided: its plenums' zones, holding nothing, how long each zone is, m, how fast the
/// waves in the flow between them die away, 1/s, the plenum and zone each cylinder's runner leaves by, and
/// the balance valves.
struct Layout {
    rows: Vec<Row>,
    dx: f64,
    damping: f64,
    attach: Vec<(usize, usize)>,
    balances: Vec<Balance>,
}

impl Layout {
    /// Divide each plenum into zones along its length, each as near `ZONE_LENGTH` long as a wave crossing
    /// it in a sample allows, and find the zone each runner leaves by.
    fn new(spec: &EngineSpec, shape: PlenumShape, sample_rate: f64) -> Layout {
        let count = plenum_count_of(spec);
        let share = 1.0 / count as f64;
        let length = math::max(shape.length, 1e-3);
        let shortest = (MAX_SOUND_SPEED / sample_rate) / MAX_COURANT;
        let n = clamp((length / ZONE_LENGTH).round(), 1.0, math::max((length / shortest).floor(), 1.0)) as usize;
        let n = n.min(MAX_ZONES);
        let dx = length / n as f64;
        let floor = 1e-5 / n as f64;
        let row = Row {
            zones: (0..n)
                .map(|i| Zone {
                    mass: 0.0,
                    energy: 0.0,
                    burned_mass: 0.0,
                    fuel_mass: 0.0,
                    volume: math::max(shape.area_at((i as f64 + 0.5) * dx) * share * dx, floor),
                })
                .collect(),
            face_areas: (1..n).map(|i| math::max(shape.area_at(i as f64 * dx) * share, 1e-6)).collect(),
            flows: vec![0.0; n - 1],
            steady: vec![0.0; n - 1],
            change: vec![[0.0; 4]; n],
        };
        let c = math::sqrt(gas::GAMMA_AIR * gas::R * gas::T_AMB);
        let damping = (2.0 * PI * (c / (2.0 * length))) / PLENUM_Q;
        // The runners leave it where their cylinders are along the engine, its middle the engine's: dual
        // plenums' each from its own bank's.
        let attach = (0..spec.cylinders as usize)
            .map(|c| {
                let row = if count > 1 { (physical_bank(spec, c) as usize).min(count - 1) } else { 0 };
                let zone = (((cylinder_z(spec, c) + length / 2.0) / dx).floor() as isize).clamp(0, n as isize - 1);
                (row, zone as usize)
            })
            .collect();
        // Each balance valve's air, a throttle bore across, carried through the wall and the end corrections
        // either side of it; no freer than the plenum's own section from one zone to the next, which keeps
        // it as stable as they are.
        let bore = throttle_dia_of(spec);
        let carried = BALANCE_WALL + 1.7 * (bore / 2.0);
        let reach = math::min((PI * bore * bore) / 4.0 / carried, (shape.area_at(0.0) * share) / dx);
        let balances = if count > 1 {
            BALANCE_VALVES
                .iter()
                .map(|f| Balance { zone: ((f * n as f64).floor() as usize).min(n - 1), reach, flow: 0.0 })
                .collect()
        } else {
            Vec::new()
        };
        Layout { rows: vec![row; count], dx, damping, attach, balances }
    }
}

pub struct IntakePlenum {
    shape: PlenumShape,
    sample_rate: f64,
    /// One plenum, or dual plenums, bank 0's first.
    rows: Vec<Row>,
    /// How long each zone is, m, and how fast the waves in the flow between them die away, 1/s.
    dx: f64,
    damping: f64,
    /// The plenum and zone each cylinder's runner leaves by.
    attach: Vec<(usize, usize)>,
    balances: Vec<Balance>,
    /// The speeds the balance valves open at, shut again at and open again at, rev/min, each 0 or less
    /// for never (`balance_wanted`); whether they are opening, and how far open they are, 0..1.
    balance_rpms: [f64; 3],
    balance_open: bool,
    balance_opening: f64,
    /// The throttle plate's opening, 0..1, and the idle air valve's, as more of the plate's.
    opening: f64,
    bypass: f64,
    /// Effective area of each throttle body, m^2, and the flow through each last sample, kg/s.
    area: f64,
    throttle_flows: [f64; 2],
}

impl IntakePlenum {
    pub fn new(spec: &EngineSpec, sample_rate: f64) -> IntakePlenum {
        let shape = plenum_shape_of(spec);
        let layout = Layout::new(spec, shape, sample_rate);
        let open = balance_wanted(balance_rpms_of(spec), spec.rpm, false);
        let mut plenum = IntakePlenum {
            shape,
            sample_rate,
            rows: layout.rows,
            dx: layout.dx,
            damping: layout.damping,
            attach: layout.attach,
            balances: layout.balances,
            balance_rpms: balance_rpms_of(spec),
            balance_open: open,
            balance_opening: if open { 1.0 } else { 0.0 },
            opening: spec.throttle,
            bypass: 0.0,
            area: IntakePlenum::throttle_area(spec),
            throttle_flows: [0.0; 2],
        };
        plenum.reset();
        plenum
    }

    fn zones(&self) -> impl Iterator<Item = &Zone> + '_ {
        self.rows.iter().flat_map(|r| r.zones.iter())
    }

    /// Set the throttle to `opening`, 0..1, in place of the spec's, until the next `set_geometry`.
    pub fn set_opening(&mut self, spec: &EngineSpec, opening: f64) {
        self.opening = opening;
        self.area = IntakePlenum::bypassed_area(spec, self.opening, self.bypass);
    }

    /// Open the idle air valve, the bypass round the throttle plate, by `bypass`: as much air as that
    /// much more of the plate's opening would pass.
    pub fn set_bypass(&mut self, spec: &EngineSpec, bypass: f64) {
        if bypass == self.bypass {
            return;
        }
        self.bypass = bypass;
        self.area = IntakePlenum::bypassed_area(spec, self.opening, self.bypass);
    }

    /// Rebuild geometry in place. Its gas is kept: at the same state, where only the throttle changed, or
    /// at its mean pressure, temperature and composition through a plenum of a new shape.
    pub fn set_geometry(&mut self, spec: &EngineSpec) {
        self.opening = spec.throttle;
        self.area = IntakePlenum::bypassed_area(spec, self.opening, self.bypass);
        self.balance_rpms = balance_rpms_of(spec);
        let shape = plenum_shape_of(spec);
        let layout = Layout::new(spec, shape, self.sample_rate);
        let reaches = |b: &[Balance]| b.iter().map(|b| (b.zone, b.reach)).collect::<Vec<_>>();
        if shape == self.shape
            && layout.rows.len() == self.rows.len()
            && layout.attach == self.attach
            && reaches(&layout.balances) == reaches(&self.balances)
        {
            return;
        }
        let total = |f: fn(&Zone) -> f64| self.zones().map(f).sum::<f64>();
        let (mass, energy, burned, fuel, volume) =
            (total(|z| z.mass), total(|z| z.energy), total(|z| z.burned_mass), total(|z| z.fuel_mass), total(|z| z.volume));
        self.shape = shape;
        self.rows = layout.rows;
        self.dx = layout.dx;
        self.damping = layout.damping;
        self.attach = layout.attach;
        self.balances = layout.balances;
        // Each zone at the mean density and state the old plenum held.
        for z in self.rows.iter_mut().flat_map(|r| r.zones.iter_mut()) {
            let share = z.volume / math::max(volume, 1e-12);
            z.mass = mass * share;
            z.energy = energy * share;
            z.burned_mass = burned * share;
            z.fuel_mass = fuel * share;
        }
    }

    /// Open or shut the balance valves between dual plenums for the engine's speed, `rpm`, as `dt` passes:
    /// open from their opening speed up to their shutting speed, and again from their reopening speed
    /// (`balance_wanted`), swinging over `BALANCE_TRAVEL`.
    pub fn update_balance(&mut self, dt: f64, rpm: f64) {
        if self.balances.is_empty() {
            return;
        }
        self.balance_open = balance_wanted(self.balance_rpms, rpm, self.balance_open);
        let swing = dt / BALANCE_TRAVEL;
        self.balance_opening = if self.balance_open {
            math::min(self.balance_opening + swing, 1.0)
        } else {
            math::max(self.balance_opening - swing, 0.0)
        };
    }

    /// How many plenums it is: 2 for dual plenums.
    pub fn count(&self) -> usize {
        self.rows.len()
    }

    /// Whether dual plenums' balance valves are open, or opening.
    pub fn balanced(&self) -> bool {
        !self.balances.is_empty() && self.balance_open
    }

    /// Its zones, front to back, every plenum's: how many.
    pub fn zone_count(&self) -> usize {
        self.rows.iter().map(|r| r.zones.len()).sum()
    }

    /// Gauge pressure in each zone, Pa: front to back, bank 0's plenum and then bank 1's.
    pub fn zone_pressures(&self) -> impl Iterator<Item = f64> + '_ {
        self.zones().map(|z| z.pressure() - gas::P_AMB)
    }

    /// What cylinder `c`'s runner draws from.
    pub fn feed(&self, c: usize) -> PlenumFeed {
        let (row, zone) = self.attach.get(c).copied().unwrap_or((0, 0));
        let z = &self.rows[row].zones[zone];
        let p = z.pressure();
        PlenumFeed { p, rho: p / (gas::R * z.temp()), burned: z.burned(), fuel: z.fuel() }
    }

    /// Its mean temperature, K.
    pub fn temp(&self) -> f64 {
        let mass: f64 = self.zones().map(|z| z.mass).sum();
        let energy: f64 = self.zones().map(|z| z.energy).sum();
        clamp(gas_temperature(energy / math::max(mass, MIN_MASS)), 150.0, 3000.0)
    }

    /// Its mean absolute pressure, Pa, over its volume.
    pub fn pressure(&self) -> f64 {
        let volume: f64 = self.zones().map(|z| z.volume).sum();
        self.zones().map(|z| z.pressure() * z.volume).sum::<f64>() / volume
    }

    /// Absolute pressure by the throttles, at the plenums' fronts, Pa, on average.
    pub fn throttle_pressure(&self) -> f64 {
        self.rows.iter().map(|r| r.zones[0].pressure()).sum::<f64>() / self.rows.len() as f64
    }

    pub fn burned_fraction(&self) -> f64 {
        let mass: f64 = self.zones().map(|z| z.mass).sum();
        clamp(self.zones().map(|z| z.burned_mass).sum::<f64>() / math::max(mass, MIN_MASS), 0.0, 1.0)
    }

    pub fn fuel_fraction(&self) -> f64 {
        let mass: f64 = self.zones().map(|z| z.mass).sum();
        clamp(self.zones().map(|z| z.fuel_mass).sum::<f64>() / math::max(mass, MIN_MASS), 0.0, 1.0)
    }

    /// Effective flow area of one throttle body, m^2: geometric area times the plate's discharge
    /// coefficient.
    pub fn throttle_area(spec: &EngineSpec) -> f64 {
        IntakePlenum::throttle_area_at(spec, spec.throttle)
    }

    /// Effective flow area of one throttle body, m^2, its plate at `opening` and the idle air valve round
    /// it at `bypass`. The valve is sized for the engine, not the throttle body: it passes what that much
    /// more opening would pass through a throttle body of the size `throttle_dia_of` gives the engine left
    /// to itself, so a small throttle body given has no less air to idle on.
    pub fn bypassed_area(spec: &EngineSpec, opening: f64, bypass: f64) -> f64 {
        if !(spec.throttle_dia > 0.0) {
            return IntakePlenum::throttle_area_at(spec, opening + bypass);
        }
        let sized = sized_throttle_dia(spec);
        let valve = plate_area(sized, opening + bypass) - plate_area(sized, opening);
        IntakePlenum::throttle_area_at(spec, opening) + valve
    }

    pub fn throttle_area_at(spec: &EngineSpec, opening: f64) -> f64 {
        plate_area(throttle_dia_of(spec), opening)
    }

    /// The flow in through each throttle body last sample, kg/s, as the plenums are ordered.
    pub fn throttle_flows(&self) -> &[f64] {
        &self.throttle_flows[..self.rows.len()]
    }

    /// Effective flow area of each throttle body, m^2, as its plate is set.
    pub fn throttle_area_each(&self) -> f64 {
        self.area
    }

    /// Effective throttle flow area, m^2, as the plates are set: every throttle body's together.
    pub fn area(&self) -> f64 {
        self.area * self.rows.len() as f64
    }

    /// Advance by `dt`, drawing through each plenum's throttle into its front zone from air at `p_up` (Pa),
    /// each throttle's own or, where there are fewer, the last, and `t_up` (K): the atmosphere through an
    /// inlet tract, or a turbocharger's charge air. Each of `runners`, a cylinder's, takes from or gives
    /// back to the zone it leaves by what flowed out of its plenum end over the sample, at its own
    /// temperature and composition where it flows back in. Returns the flow in through the throttles,
    /// kg/s, each one's after in `throttle_flows`.
    pub fn step(&mut self, dt: f64, p_up: &[f64], t_up: f64, runners: &[Runner]) -> f64 {
        let follow = dt / STEADY_FLOW_TIME;
        let (dx, damping, area) = (self.dx, self.damping, self.area);
        let mut throttle_flow = 0.0;
        for (k, row) in self.rows.iter_mut().enumerate() {
            let p_up = p_up[k.min(p_up.len() - 1)];
            let n = row.zones.len();
            for c in row.change.iter_mut() {
                *c = [0.0; 4];
            }

            // The flow between each zone and the next back, driven by the pressure across it, its waves
            // about its steady part losing their strength, taken implicitly.
            for f in 0..n - 1 {
                let push = (row.face_areas[f] / dx) * (row.zones[f].pressure() - row.zones[f + 1].pressure());
                let steady = row.steady[f];
                row.flows[f] = steady + (row.flows[f] - steady + dt * push) / (1.0 + dt * damping);
                row.steady[f] += follow * (row.flows[f] - steady);
            }

            // The throttle, into the front zone.
            let front = row.zones[0];
            let (p, t) = (front.pressure(), front.temp());
            let through = if p < p_up {
                orifice_mass_flow(area, 1.0, p_up, t_up, p, gas::GAMMA_AIR)
            } else {
                -orifice_mass_flow(area, 1.0, p, t, p_up, gas_gamma(t))
            };
            throttle_flow += through;
            self.throttle_flows[k] = through;
            {
                let c = &mut row.change[0];
                c[0] += through;
                c[1] += through * gas_enthalpy(if through >= 0.0 { t_up } else { t });
                if through < 0.0 {
                    c[2] += through * front.burned();
                    c[3] += through * front.fuel();
                }
            }

            // From each zone to the next, at the state of the one it leaves.
            for f in 0..n - 1 {
                let q = row.flows[f];
                let from = if q >= 0.0 { row.zones[f] } else { row.zones[f + 1] };
                let moved = [q, q * gas_enthalpy(from.temp()), q * from.burned(), q * from.fuel()];
                for k in 0..4 {
                    row.change[f][k] -= moved[k];
                    row.change[f + 1][k] += moved[k];
                }
            }

            // And mixing between each zone and the next: as much gas of each swapped for the other's, so
            // their heat and makeup even out without moving any mass.
            for f in 0..n - 1 {
                let (a, b) = (row.zones[f], row.zones[f + 1]);
                let swapped = mixing(&a, &b) * row.face_areas[f] / dx;
                swap(&a, &b, swapped, &mut row.change, f, f + 1);
            }
        }

        // The balance valves, as far open as they are: the air through each carried by its momentum, as
        // between the zones, and the gas either side mixing through it.
        if let [first, second] = &mut self.rows[..] {
            let open = self.balance_opening;
            for b in self.balances.iter_mut() {
                let (a, c) = (first.zones[b.zone], second.zones[b.zone]);
                let push = open * b.reach * (a.pressure() - c.pressure());
                b.flow = if open > 0.0 { (b.flow + dt * push) / (1.0 + dt * damping) } else { 0.0 };
                let q = b.flow;
                let from = if q >= 0.0 { a } else { c };
                let moved = [q, q * gas_enthalpy(from.temp()), q * from.burned(), q * from.fuel()];
                for k in 0..4 {
                    first.change[b.zone][k] -= moved[k];
                    second.change[b.zone][k] += moved[k];
                }
                let swapped = mixing(&a, &c) * open * b.reach;
                let (per_a, per_c) = (per_mass(&a), per_mass(&c));
                for k in 1..4 {
                    let moved = swapped * (per_a[k] - per_c[k]);
                    first.change[b.zone][k] -= moved;
                    second.change[b.zone][k] += moved;
                }
            }
        }

        // The runners: drawing from their zones, or pushing back into them what their cylinders sent up.
        for (c, r) in runners.iter().enumerate() {
            let (row, i) = self.attach.get(c).copied().unwrap_or((0, 0));
            let row = &mut self.rows[row];
            let i = i.min(row.zones.len() - 1);
            let flow = r.plenum_flow;
            let zone = row.zones[i];
            let moved = if flow >= 0.0 {
                [flow, flow * gas_enthalpy(r.plenum_temp), flow * r.mouth_burned, flow * r.mouth_fuel]
            } else {
                [flow, flow * gas_enthalpy(zone.temp()), flow * zone.burned(), flow * zone.fuel()]
            };
            for k in 0..4 {
                row.change[i][k] += moved[k];
            }
        }

        let mut broken = false;
        for row in self.rows.iter_mut() {
            for (z, c) in row.zones.iter_mut().zip(&row.change) {
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
            broken |= row.flows.iter().chain(&row.steady).any(|q| !q.is_finite());
        }
        if broken || self.balances.iter().any(|b| !b.flow.is_finite()) {
            self.reset();
        }
        throttle_flow
    }

    pub fn reset(&mut self) {
        for row in self.rows.iter_mut() {
            for z in row.zones.iter_mut() {
                *z = Zone::ambient(z.volume);
            }
            for q in row.flows.iter_mut().chain(row.steady.iter_mut()) {
                *q = 0.0;
            }
        }
        for b in self.balances.iter_mut() {
            b.flow = 0.0;
        }
    }
}

/// The speeds `spec`'s balance valves open at, shut again at and open again at, rev/min.
fn balance_rpms_of(spec: &EngineSpec) -> [f64; 3] {
    [spec.plenum_balance_rpm, spec.plenum_balance_shut_rpm, spec.plenum_balance_reopen_rpm]
}

/// Whether balance valves opening at `open` rev/min, shutting again at `shut` and opening again at
/// `reopen`, each 0 or less for never, want to be open at `rpm`, `was` whether they are: open from the
/// first up to the second, and from the third up. They stay as they are within `BALANCE_HYSTERESIS`
/// below each speed, so they do not flap back and forth at it.
fn balance_wanted([open, shut, reopen]: [f64; 3], rpm: f64, was: bool) -> bool {
    let band = if was { BALANCE_HYSTERESIS } else { 0.0 };
    let first = open > 0.0 && rpm >= open - band && (shut <= 0.0 || rpm < shut - (BALANCE_HYSTERESIS - band));
    let again = reopen > 0.0 && rpm >= reopen - band;
    first || again
}

/// The gas two neighbouring zones' mixing swaps, kg/s for each metre of the opening between them over the
/// distance it is across, m: at their mean density, `PLENUM_MIXING`.
fn mixing(a: &Zone, b: &Zone) -> f64 {
    0.5 * (a.mass / a.volume + b.mass / b.volume) * PLENUM_MIXING
}

/// A zone's energy, spent gas and fuel per kilogram of its gas, after a place for its mass.
fn per_mass(z: &Zone) -> [f64; 4] {
    [0.0, z.energy / math::max(z.mass, MIN_MASS), z.burned(), z.fuel()]
}

/// `swapped` kg/s of zone `a`'s gas, at `change[i]`, traded for as much of zone `b`'s, at `change[j]`.
fn swap(a: &Zone, b: &Zone, swapped: f64, change: &mut [[f64; 4]], i: usize, j: usize) {
    let (pa, pb) = (per_mass(a), per_mass(b));
    for k in 1..4 {
        let moved = swapped * (pa[k] - pb[k]);
        change[i][k] -= moved;
        change[j][k] += moved;
    }
}
