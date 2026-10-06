//! In-cylinder thermodynamics: a single control volume whose boundary moves with the piston, filled
//! and emptied through the valves.
//!
//! Mass and sensible internal energy are the state; temperature and pressure follow. The pressure at
//! exhaust valve opening is a consequence of the compression ratio, the spark, the burn and how much
//! charge was trapped, not a number anyone typed in.

use std::sync::OnceLock;

use crate::dsp::{Noise, cycle_delta, window_phase, wrap_cycle};
use crate::math::{self, PI, clamp};
use crate::spec::{
    CV_REF, CV_SLOPE, CrankGeometry, CrankState, EngineSpec, Fuel, T_REF, crank_at, cylinder_volume, displacement,
    fuel_fraction_at, gas,
};

const HALF_SLOPE: f64 = 0.5 * CV_SLOPE;
const CV_REF_SQ: f64 = CV_REF * CV_REF;
const TWO_SLOPE: f64 = 2.0 * CV_SLOPE;

/// An engine spec together with what the cylinders precompute from it, and an identity.
///
/// The cylinder recomputes what it caches from a spec whenever it is handed one with a different
/// identity, and a spec gets a new one after every edit and on every switch of cam profile, never
/// merely because the throttle moved. Switching cams also resets whether the cylinder counts itself
/// as exchanging gas, so the identity is part of the result, not only a cache key.
#[derive(Clone, Debug)]
pub struct SpecInstance {
    pub spec: EngineSpec,
    pub id: u64,
    pub crank: CrankGeometry,
    /// The identity of `crank`, shared by a copy of the spec that differs only in its cams.
    pub crank_id: u64,
}

fn next_id() -> u64 {
    static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(1);
    NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
}

impl SpecInstance {
    pub fn new(spec: EngineSpec) -> SpecInstance {
        let id = next_id();
        let crank = CrankGeometry::of(&spec);
        SpecInstance { spec, id, crank, crank_id: id }
    }

    /// `spec`, which must differ from this one only in its valve events and lift, as a new instance
    /// sharing this one's crank.
    pub fn with_cams(&self, spec: EngineSpec) -> SpecInstance {
        SpecInstance { spec, id: next_id(), crank: self.crank, crank_id: self.crank_id }
    }
}

/// The inputs to one `Cylinder::advance`.
#[derive(Clone, Copy, Debug, Default)]
pub struct AdvanceIo {
    pub dt: f64,
    /// Crank speed, rad/s.
    pub omega: f64,
    /// kg/s, positive out of the cylinder into the exhaust port.
    pub ex_mdot: f64,
    /// kg/s, positive into the cylinder from the intake.
    pub in_mdot: f64,
    /// Temperature of the charge arriving through the intake, K.
    pub intake_t: f64,
    /// Temperature of the gas in the exhaust port, carried in by reverse flow, K.
    pub port_t: f64,
    /// Burned and fuel fractions of what arrives through the intake.
    pub intake_burned: f64,
    pub intake_fuel: f64,
}

/// An overrun crackle map's spark: fired `atdc` degrees after top dead centre, in place of the advance
/// map's, and skipped on a random `skip` share of the cycles, whose charge goes out unburned.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CrackleSpark {
    pub atdc: f64,
    pub skip: f64,
}

/// Pressure, temperature, burned fraction and fuel fraction of a cylinder's contents.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CylState {
    pub pressure: f64,
    pub temp: f64,
    pub burned: f64,
    pub fuel: f64,
}

// On cache lines of its own, as each may be stepped on a thread of its own.
#[repr(align(128))]
pub struct Cylinder {
    /// Crank angle, deg in [0, 720). 0 = TDC firing.
    pub angle: f64,
    /// Trapped gas mass, kg.
    pub mass: f64,
    /// Total sensible internal energy of the trapped gas, J: the state variable, not temperature.
    energy: f64,
    /// 0..1 mass fraction burned for the current cycle.
    pub burned: f64,
    q_cycle: f64,
    /// Unburned charge currently trapped, kg.
    fresh_mass: f64,
    /// Of `fresh_mass`, how much is unburned fuel, kg.
    fuel_mass: f64,
    burn_fuel: f64,
    charge_phi: f64,
    charge_residual: f64,
    /// Fresh charge trapped at the last intake valve closing, kg.
    pub trapped_fresh: f64,
    /// Wiebe duration this cycle burns over, deg: a diesel's diffusion burn.
    pub burn_angle: f64,
    /// Crank angle this cycle's spark fires at, deg: where a diesel's charge lights, after its
    /// ignition delay.
    pub spark: f64,
    armed: bool,
    /// Set by the rev limiter; read when the charge is committed.
    pub spark_cut: bool,
    /// A diesel's fuel demand from its pedal and governor, 0..1 of the full delivery; read when the
    /// charge is committed.
    pub fuel_demand: f64,
    /// This cycle's charge is a diesel's: injected as it burns, in two stages.
    diesel: bool,
    /// Of a diesel's fuel, the share that mixes during the ignition delay and burns at once, 0..1.
    pub premixed_share: f64,
    /// Wiebe duration of a diesel's premixed burn, deg.
    pub premixed_angle: f64,
    /// Stoichiometric air-fuel ratio of this cycle's fuel.
    afr: f64,
    /// Fuel injected this step, kg: it joins the gas as it burns.
    injected: f64,
    /// Set by the overrun crackle map; read when the charge is committed.
    pub crackle: Option<CrackleSpark>,
    /// Unburned fuel, and the air with it, sent out through the exhaust valve since the last
    /// `take_exhausted`, kg.
    exhausted_fuel: f64,
    exhausted_air: f64,
    /// This cylinder's cam timing offsets from nominal, crank degrees.
    pub intake_cam_offset: f64,
    pub exhaust_cam_offset: f64,

    /// Instantaneous gas torque at the crank, N*m.
    pub torque: f64,
    /// Reciprocating inertia torque at the crank, N*m.
    pub inertia_torque: f64,
    /// Times the gas temperature had to be clamped. Should stay zero.
    pub clamp_hits: u64,
    /// Rate of cylinder pressure rise, Pa/s.
    pub dpdt: f64,

    next_angle: f64,
    /// The crank at `crank_angle` on the crank `crank_id`: a substep ends where the next begins, so
    /// each angle is evaluated once.
    crank_angle: f64,
    crank_id: u64,
    crank_state: CrankState,
    burn_scale: f64,
    ignition_offset: f64,
    noise: Noise,

    woschni_spec: u64,
    woschni_bore: f64,
    woschni_omega: f64,
    woschni_piston_speed: f64,
    woschni_speed_exchange: f64,
    woschni_speed_closed: f64,
    ref_pressure: f64,
    ref_mass_r: f64,
    combustion_velocity: f64,
    event_spec: u64,
    event_intake_offset: f64,
    event_exhaust_offset: f64,
    evo_at: f64,
    ivc_at: f64,
    exchanging: bool,
    motored_pressure: f64,
    step_pressure: f64,
    step_volume: f64,
    step_temp: f64,
    step_omega: f64,
}

/// Woschni's velocity coefficients.
const WOSCHNI_C1_EXCHANGE: f64 = 6.18;
const WOSCHNI_C1_CLOSED: f64 = 2.28;
const WOSCHNI_C2: f64 = 3.24e-3;

pub(crate) const COMBUSTION_EFFICIENCY: f64 = 0.96;

/// Floor on trapped mass, kg.
const MIN_MASS: f64 = 2e-7;

/// Ceiling on the per-cycle combustion scatter.
const MAX_SCATTER: f64 = 0.16;

/// Spent-gas fraction of the trapped charge beyond which the flame starts to fail, and at which it
/// always does.
const DILUTION_ONSET: f64 = 0.4;
const DILUTION_FULL: f64 = 0.9;

const SL_PHI_PEAK: f64 = 1.21;
const SL_PEAK: f64 = 0.305;
const SL_CURVE: f64 = -0.549;

/// Laminar speed of the mixture, as a fraction of a stoichiometric one's, below which the spark kernel
/// starts to fail.
const KERNEL_SPEED_ONSET: f64 = 0.5;

const SL_DILUTION_FLOOR: f64 = 0.1;

const TURBULENCE_PER_PISTON_SPEED: f64 = 0.5;

const REF_PISTON_SPEED: f64 = 10.0;
const REF_TURBULENCE: f64 = TURBULENCE_PER_PISTON_SPEED * REF_PISTON_SPEED;

const BURNUP_SHARE: f64 = 0.35;
const MAX_BURN_ANGLE: f64 = 150.0;
const COMPRESSION_EXPONENT: f64 = 1.32;
const WIEBE_HALF: f64 = 0.516;
const MAX_ADVANCE: f64 = 50.0;
const MIN_ADVANCE: f64 = 0.0;

/// Cetane number of the diesel fuel, which sets its autoignition activation energy.
const CETANE: f64 = 45.0;
/// Universal gas constant, J/(mol K).
const R_UNIVERSAL: f64 = 8.3143;
/// The pressure the ignition delay correlation is singular at, bar: it is floored a little above.
const DELAY_PRESSURE_FLOOR: f64 = 13.0;
/// Longest ignition delay, deg: a charge that would wait longer lights there, late.
const MAX_IGNITION_DELAY: f64 = 60.0;
/// Watson's premixed share: `1 - A phi^B / tau^C`, `tau` in ms.
const PREMIXED_A: f64 = 0.926;
const PREMIXED_B: f64 = 0.37;
const PREMIXED_C: f64 = 0.26;
const MAX_PREMIXED_SHARE: f64 = 0.9;
/// How long the premixed burn takes, s: chemistry, so fixed in time rather than in degrees.
const PREMIXED_TIME: f64 = 0.8e-3;
const MIN_PREMIXED_ANGLE: f64 = 2.0;
/// Wiebe form factor of the diffusion burn: fast to start and slow to finish, as the last of the
/// fuel finds its air.
const DIFFUSION_M: f64 = 0.9;
/// Share of the spark's timing scatter a diesel's injection keeps: the pump meters the same
/// moment each time, and only the ignition delay varies.
const DIESEL_TIMING_SCATTER: f64 = 0.15;

/// The constants derived through transcendental functions, computed once.
struct Derived {
    stoich_speed: f64,
    ref_laminar: f64,
    ref_viscosity: f64,
    wiebe_norm: f64,
    energy_at_floor: f64,
}

fn derived() -> &'static Derived {
    static D: OnceLock<Derived> = OnceLock::new();
    D.get_or_init(|| Derived {
        stoich_speed: laminar_speed_base(1.0),
        ref_laminar: laminar_flame_speed(1.0, 650.0, 13e5, 0.04),
        ref_viscosity: kinematic_viscosity(650.0, 13e5),
        wiebe_norm: 1.0 / (1.0 - math::exp(-5.0)),
        energy_at_floor: energy_at(150.0),
    })
}

impl Cylinder {
    pub fn new(spec: &EngineSpec, angle: f64, seed: f64) -> Cylinder {
        let angle = wrap_cycle(angle);
        let mass = (gas::P_AMB * cylinder_volume(spec, angle)) / (gas::R * 700.0);
        Cylinder {
            angle,
            mass,
            energy: mass * energy_at(700.0),
            burned: 0.0,
            q_cycle: 0.0,
            fresh_mass: 0.0,
            fuel_mass: 0.0,
            burn_fuel: 0.0,
            charge_phi: 1.0,
            charge_residual: 0.0,
            trapped_fresh: 0.0,
            burn_angle: 0.0,
            spark: 0.0,
            armed: false,
            spark_cut: false,
            fuel_demand: 1.0,
            diesel: false,
            premixed_share: 0.0,
            premixed_angle: 0.0,
            afr: gas::AFR_STOICH,
            injected: 0.0,
            crackle: None,
            exhausted_fuel: 0.0,
            exhausted_air: 0.0,
            intake_cam_offset: 0.0,
            exhaust_cam_offset: 0.0,
            torque: 0.0,
            inertia_torque: 0.0,
            clamp_hits: 0,
            dpdt: 0.0,
            next_angle: 0.5,
            crank_angle: f64::NAN,
            crank_id: 0,
            crank_state: CrankState::default(),
            burn_scale: 1.0,
            ignition_offset: 0.0,
            noise: Noise::new(seed),
            woschni_spec: 0,
            woschni_bore: 0.0,
            woschni_omega: f64::NAN,
            woschni_piston_speed: 0.0,
            woschni_speed_exchange: 0.0,
            woschni_speed_closed: 0.0,
            ref_pressure: 0.0,
            ref_mass_r: 1.0,
            combustion_velocity: 0.0,
            event_spec: 0,
            event_intake_offset: 0.0,
            event_exhaust_offset: 0.0,
            evo_at: 0.0,
            ivc_at: 0.0,
            exchanging: false,
            motored_pressure: 0.0,
            step_pressure: 0.0,
            step_volume: 0.0,
            step_temp: 0.0,
            step_omega: 0.0,
        }
    }

    /// The unburned fuel and air sent out through the exhaust valve since the last call, kg.
    pub fn take_exhausted(&mut self) -> (f64, f64) {
        let out = (self.exhausted_fuel, self.exhausted_air);
        self.exhausted_fuel = 0.0;
        self.exhausted_air = 0.0;
        out
    }

    /// Fuel this cycle's charge burns, kg: a diesel's, what it is injected.
    pub fn cycle_fuel(&self) -> f64 {
        self.burn_fuel
    }

    /// Fraction of the trapped charge that is spent gas, 0..1.
    pub fn burned_fraction(&self) -> f64 {
        clamp(1.0 - self.fresh_mass / math::max(self.mass, MIN_MASS), 0.0, 1.0)
    }

    /// Bulk gas temperature, K.
    #[inline]
    pub fn temp(&self) -> f64 {
        let u = self.energy / self.mass;
        clamp(T_REF + (2.0 * u) / (CV_REF + math::sqrt(CV_REF_SQ + TWO_SLOPE * u)), 150.0, 6000.0)
    }

    pub fn set_temp(&mut self, value: f64) {
        self.energy = self.mass * energy_at(value);
    }

    /// Absolute pressure, Pa.
    pub fn pressure(&self, spec: &EngineSpec) -> f64 {
        (self.mass * gas::R * self.temp()) / cylinder_volume(spec, self.angle)
    }

    /// The crank at `angle`, from the cache when it holds that angle on this crank.
    #[inline]
    fn crank(&mut self, si: &SpecInstance, angle: f64) -> CrankState {
        if angle.to_bits() != self.crank_angle.to_bits() || si.crank_id != self.crank_id {
            self.crank_angle = angle;
            self.crank_id = si.crank_id;
            self.crank_state = crank_at(&si.crank, angle);
        }
        self.crank_state
    }

    /// The crank where this cylinder's piston is now.
    #[inline]
    pub fn crank_now(&mut self, si: &SpecInstance) -> CrankState {
        self.crank(si, self.angle)
    }

    /// Pressure, temperature, burned fraction and fuel fraction now.
    #[inline]
    pub fn read_state(&mut self, si: &SpecInstance) -> CylState {
        let b = 1.0 - self.fresh_mass / math::max(self.mass, MIN_MASS);
        let burned = if b < 0.0 {
            0.0
        } else if b > 1.0 {
            1.0
        } else {
            b
        };
        let f = self.fuel_mass / math::max(self.mass, MIN_MASS);
        let fuel = if f > 1.0 { 1.0 } else { f };
        let u = self.energy / self.mass;
        let raw = T_REF + (2.0 * u) / (CV_REF + math::sqrt(CV_REF_SQ + TWO_SLOPE * u));
        let temp = if raw < 150.0 {
            150.0
        } else if raw > 6000.0 {
            6000.0
        } else {
            raw
        };
        let volume = self.crank(si, self.angle).volume;
        CylState { pressure: (self.mass * gas::R * temp) / volume, temp, burned, fuel }
    }

    /// Advance the gas state by `dt` at crank speed `omega`, with the fuel of a stoichiometric
    /// mixture arriving through the intake unless `intake_fuel` says otherwise.
    #[allow(clippy::too_many_arguments)]
    pub fn advance_with(
        &mut self,
        spec: &SpecInstance,
        dt: f64,
        omega: f64,
        ex_mdot: f64,
        in_mdot: f64,
        intake_temp: f64,
        port_temp: f64,
        intake_burned: f64,
        intake_fuel: Option<f64>,
    ) {
        let io = AdvanceIo {
            dt,
            omega,
            ex_mdot,
            in_mdot,
            intake_t: intake_temp,
            port_t: port_temp,
            intake_burned,
            intake_fuel: intake_fuel.unwrap_or(fuel_fraction_at(1.0) * (1.0 - intake_burned)),
        };
        self.advance(spec, &io);
    }

    /// Advance the gas state by `io.dt` seconds.
    pub fn advance(&mut self, si: &SpecInstance, io: &AdvanceIo) {
        let spec = &si.spec;
        let dt = io.dt;
        let omega = io.omega;
        let ex_mdot = io.ex_mdot;
        let in_mdot = io.in_mdot;
        let intake_temp = io.intake_t;
        let port_temp = io.port_t;
        let intake_burned = io.intake_burned;
        let intake_fuel = io.intake_fuel;
        let d_theta = (omega * dt * 180.0) / PI;
        let next_angle = wrap_cycle(self.angle + d_theta);
        self.next_angle = next_angle;
        self.step_omega = omega;

        let t_now = self.temp();
        let k = self.crank(si, self.angle);
        let v = k.volume;
        let p = (self.mass * gas::R * t_now) / v;
        let dv_dtheta = k.d_volume;
        let dv_dt = dv_dtheta * omega;
        let p_before = p;
        let xd = k.d_position;
        let xdd = k.d2_position;
        self.step_pressure = p;
        self.step_volume = v;
        self.step_temp = t_now;

        // --- Combustion ---
        self.update_combustion_latches(si);
        let dq_comb = self.heat_release();

        // --- Wall heat transfer (Woschni) ---
        let dq_wall = self.woschni(si, p, v, omega, t_now) * dt;

        let dq = dq_comb + dq_wall;

        // --- Energy balance: each valve by direction, each stream at its own temperature ---
        let d_own = t_now - T_REF;
        let h_own = d_own * (CV_REF + HALF_SLOPE * d_own) + gas::R * t_now;
        let d_port = port_temp - T_REF;
        let d_intake = intake_temp - T_REF;
        let h_ex = if ex_mdot >= 0.0 {
            ex_mdot * h_own
        } else {
            ex_mdot * (d_port * (CV_REF + HALF_SLOPE * d_port) + gas::R * port_temp)
        };
        let h_in = if in_mdot >= 0.0 {
            in_mdot * (d_intake * (CV_REF + HALF_SLOPE * d_intake) + gas::R * intake_temp)
        } else {
            in_mdot * h_own
        };
        let du = dq + (-p * dv_dt - h_ex + h_in) * dt;

        let dm = (in_mdot - ex_mdot) * dt + self.injected;
        let mass_before = self.mass;
        self.mass = math::max(self.mass + dm, MIN_MASS);
        self.energy += du;

        // --- Composition ---
        let fresh_frac = self.fresh_mass / math::max(mass_before, MIN_MASS);
        let fuel_frac = self.fuel_mass / math::max(mass_before, MIN_MASS);
        let mut d_fresh = 0.0;
        d_fresh += (if in_mdot >= 0.0 { in_mdot * (1.0 - intake_burned) } else { in_mdot * fresh_frac }) * dt;
        d_fresh -= (if ex_mdot >= 0.0 { ex_mdot * fresh_frac } else { 0.0 }) * dt;
        self.fresh_mass = clamp(self.fresh_mass + d_fresh, 0.0, self.mass);
        let mut d_fuel = 0.0;
        d_fuel += (if in_mdot >= 0.0 { in_mdot * intake_fuel } else { in_mdot * fuel_frac }) * dt;
        d_fuel -= (if ex_mdot >= 0.0 { ex_mdot * fuel_frac } else { 0.0 }) * dt;
        self.fuel_mass = clamp(self.fuel_mass + d_fuel, 0.0, self.fresh_mass);
        if ex_mdot > 0.0 {
            let out = ex_mdot * dt;
            self.exhausted_fuel += out * fuel_frac;
            self.exhausted_air += out * math::max(fresh_frac - fuel_frac, 0.0);
        }

        // Floor the internal energy so the derived temperature stays admissible. Counted.
        let min_energy = self.mass * derived().energy_at_floor;
        if self.energy < min_energy {
            self.energy = min_energy;
            self.clamp_hits += 1;
        }

        self.torque = (p - gas::P_AMB) * dv_dtheta;
        self.inertia_torque = -spec.recip_mass * omega * omega * xdd * xd;

        self.angle = next_angle;

        let v_after = self.crank(si, self.angle).volume;
        let t_after = self.temp();
        let p_after = (self.mass * gas::R * t_after) / v_after;
        self.dpdt = (p_after - p_before) / dt;

        // Woschni's motored pressure, compressed isentropically with the volume.
        let t_motored = (self.motored_pressure * v) / self.ref_mass_r;
        let g_motored = 1.0 + gas::R / (CV_REF + CV_SLOPE * (t_motored - T_REF));
        self.motored_pressure *= 1.0 - (g_motored * (v_after - v)) / (0.5 * (v + v_after));
    }

    /// Latch per-cycle quantities at the moments they are physically determined.
    fn update_combustion_latches(&mut self, si: &SpecInstance) {
        let spec = &si.spec;
        let next_angle = self.next_angle;
        let mut ivc_passed = false;
        if si.id != self.event_spec
            || self.intake_cam_offset != self.event_intake_offset
            || self.exhaust_cam_offset != self.event_exhaust_offset
        {
            self.event_spec = si.id;
            self.event_intake_offset = self.intake_cam_offset;
            self.event_exhaust_offset = self.exhaust_cam_offset;
            let evo = spec.evo + self.exhaust_cam_offset;
            let ivc = spec.ivc + self.intake_cam_offset;
            let was_ahead = cycle_delta(self.angle, self.ivc_at) < 0.0;
            self.evo_at = wrap_cycle(evo);
            self.ivc_at = wrap_cycle(ivc);
            // A phaser advancing the intake cam can move its closing back past the crank between two
            // steps, so the crank never sweeps over it: the valve has closed all the same.
            ivc_passed = self.exchanging && was_ahead && cycle_delta(self.angle, self.ivc_at) >= 0.0;
            self.exchanging = window_phase(self.angle, evo, ivc) >= 0.0;
        }
        if crossed(self.angle, next_angle, self.evo_at) {
            self.exchanging = true;
        }
        if ivc_passed || crossed(self.angle, next_angle, self.ivc_at) {
            self.exchanging = false;
            self.ref_pressure = self.step_pressure;
            self.motored_pressure = self.step_pressure;
            self.ref_mass_r = (self.step_pressure * self.step_volume) / self.step_temp;
            self.combustion_velocity =
                (WOSCHNI_C2 * displacement(spec) * self.step_temp) / (self.step_pressure * self.step_volume);

            let fresh = self.fresh_mass;
            self.trapped_fresh = fresh;
            let fuel = self.fuel_mass;
            let air = math::max(fresh - fuel, 0.0);
            let mass = math::max(self.mass, MIN_MASS);
            self.burn_fuel = math::min(fuel, air / gas::AFR_STOICH);
            self.charge_phi = (fuel * gas::AFR_STOICH) / math::max(air, 1e-12);
            self.charge_residual = 1.0 - fresh / mass;

            // --- Cycle-to-cycle combustion scatter ---
            let full_charge = (gas::P_AMB * displacement(spec)) / (gas::R * gas::T_AMB);
            let fresh_fill = clamp(fresh / math::max(full_charge, 1e-12), 0.06, 1.2);
            let scatter = math::min(spec.combustion_variability * (0.016 + 0.019 / fresh_fill), MAX_SCATTER);

            let shared = self.noise.gaussian();
            self.burn_scale = clamp(1.0 + shared * scatter, 0.55, 2.0);
            let q_scale = clamp(1.0 + shared * scatter * 0.45 + self.noise.gaussian() * scatter * 0.3, 0.3, 1.25);
            self.ignition_offset = clamp(self.noise.gaussian() * scatter * 22.0, -14.0, 14.0);

            if spec.fuel == Fuel::Diesel {
                self.commit_injection(spec, air, q_scale);
                return;
            }
            self.diesel = false;
            self.q_cycle = self.burn_fuel * gas::FUEL_LHV * COMBUSTION_EFFICIENCY * q_scale;

            // --- Flammability, dilution, lean and rich limits ---
            let mixture_speed = laminar_speed_base(self.charge_phi);
            if !(mixture_speed > 0.0) {
                self.q_cycle = 0.0;
            } else {
                let excess_air = math::max(air - fuel * gas::AFR_STOICH, 0.0) / mass;
                let dilution = self.charge_residual + excess_air;
                let x_dilution = (dilution - DILUTION_ONSET) / (DILUTION_FULL - DILUTION_ONSET);
                let x_mixture = 1.0 - mixture_speed / (KERNEL_SPEED_ONSET * derived().stoich_speed);
                let x = math::min(math::max(x_dilution, x_mixture), 1.0);
                if x > 0.0 {
                    let u = (self.noise.next() + 1.0) / 2.0;
                    if u < x * x {
                        self.q_cycle = 0.0;
                    }
                }
            }

            // --- Burn duration and spark timing ---
            let nominal = match self.crackle {
                Some(c) => 720.0 + c.atdc + self.ignition_offset,
                None => spec.ignition + self.ignition_offset,
            };
            let squeeze = self.step_volume / cylinder_volume(spec, nominal);
            let predicted = burn_angle(
                spec,
                self.step_omega,
                self.step_pressure * math::pow(squeeze, COMPRESSION_EXPONENT),
                self.step_temp * math::pow(squeeze, COMPRESSION_EXPONENT - 1.0),
                self.charge_phi,
                self.charge_residual,
            );
            self.burn_angle = clamp(predicted * self.burn_scale, 4.0, MAX_BURN_ANGLE);
            self.spark = if self.crackle.is_some() {
                wrap_cycle(nominal)
            } else if spec.advance_curve {
                advanced_spark(spec, nominal, predicted)
            } else {
                nominal
            };

            self.burned = 0.0;
            self.armed = !self.spark_cut;
            if let Some(c) = self.crackle {
                if (self.noise.next() + 1.0) / 2.0 < c.skip {
                    self.armed = false;
                }
            }
        }
    }

    /// Commit a diesel's charge at intake valve closing: the fuel the pedal and the governor ask for,
    /// up to what the trapped air takes at the smoke limit and the pump's full delivery, injected at
    /// `spec.ignition` and lit once its ignition delay has run.
    fn commit_injection(&mut self, spec: &EngineSpec, air: f64, q_scale: f64) {
        let afr = spec.fuel.afr_stoich();
        let smoke = air / (afr * math::max(spec.smoke_lambda, 1.0));
        let full = if spec.max_fuel > 0.0 { math::min(smoke, spec.max_fuel) } else { smoke };
        let demand = clamp(self.fuel_demand, 0.0, 1.0);
        self.diesel = true;
        self.afr = afr;
        self.burn_fuel = demand * full;
        self.charge_phi = (self.burn_fuel * afr) / math::max(air, 1e-12);
        self.q_cycle = self.burn_fuel * spec.fuel.lhv() * COMBUSTION_EFFICIENCY * q_scale;

        // The charge as compression leaves it at top dead centre.
        let squeeze = self.step_volume / cylinder_volume(spec, 0.0);
        let p_tdc = self.step_pressure * math::pow(squeeze, COMPRESSION_EXPONENT);
        let t_tdc = self.step_temp * math::pow(squeeze, COMPRESSION_EXPONENT - 1.0);
        let piston_speed = (spec.stroke * self.step_omega.abs()) / PI;
        let deg_per_s = math::max((self.step_omega.abs() * 180.0) / PI, 1e-3);
        let delay = ignition_delay(piston_speed, p_tdc, t_tdc);

        self.premixed_share = premixed_share(self.charge_phi, delay / deg_per_s);
        self.premixed_angle = math::max(PREMIXED_TIME * deg_per_s, MIN_PREMIXED_ANGLE);
        let diffusion =
            spec.burn_duration * math::sqrt(piston_speed / REF_PISTON_SPEED) * (0.5 + 0.5 * demand) * self.burn_scale;
        self.burn_angle = clamp(diffusion, 4.0, MAX_BURN_ANGLE);
        let injection = spec.ignition + self.ignition_offset * DIESEL_TIMING_SCATTER;
        self.spark = wrap_cycle(injection + delay);
        self.burned = 0.0;
        self.armed = !self.spark_cut && self.q_cycle > 0.0;
    }

    /// A diesel's heat release for this step, J: the premixed and diffusion burns together, each
    /// fuel parcel injected as it burns.
    fn diesel_release(&mut self) -> f64 {
        let start = self.spark;
        let premixed = self.premixed_share;
        let burnt = |deg: f64, s: &Self| {
            premixed * wiebe(deg, s.premixed_angle) + (1.0 - premixed) * wiebe_m(deg, s.burn_angle, DIFFUSION_M)
        };
        let from = burnt(cycle_delta(self.angle, start), self);
        let to = burnt(cycle_delta(self.next_angle, start), self);
        let d = to - from;
        if d <= 0.0 {
            return 0.0;
        }
        self.burned = to;
        let mut fuel_burned = d * self.burn_fuel;
        let mut q = d * self.q_cycle;
        let left = math::max((self.fresh_mass - self.fuel_mass) / self.afr, 0.0);
        if fuel_burned > left {
            q *= left / fuel_burned;
            fuel_burned = left;
        }
        self.fresh_mass = math::max(self.fresh_mass - fuel_burned * self.afr, 0.0);
        self.fuel_mass = math::min(self.fuel_mass, self.fresh_mass);
        self.injected = fuel_burned;
        if to >= 0.999 {
            self.armed = false;
        }
        q
    }

    /// Wiebe-function heat release for this step, J.
    fn heat_release(&mut self) -> f64 {
        let next_angle = self.next_angle;
        self.injected = 0.0;
        if !self.armed || self.q_cycle <= 0.0 {
            return 0.0;
        }
        if self.diesel {
            return self.diesel_release();
        }
        let spark = self.spark;
        let duration = self.burn_angle;
        let from = wiebe(cycle_delta(self.angle, spark), duration);
        let to = wiebe(cycle_delta(next_angle, spark), duration);
        let d = to - from;
        if d <= 0.0 {
            return 0.0;
        }
        self.burned = to;
        let mut fuel_burned = d * self.burn_fuel;
        let mut q = d * self.q_cycle;
        if self.exchanging {
            // A burn still going once the exhaust valve has opened burns only the fuel, and the air for
            // it, still in the cylinder: what has left burns in the pipe, if at all.
            let left = math::min(self.fuel_mass, (self.fresh_mass - self.fuel_mass) / gas::AFR_STOICH);
            let left = math::max(left, 0.0);
            if fuel_burned > left {
                q *= left / fuel_burned;
                fuel_burned = left;
            }
        }
        self.fresh_mass = math::max(self.fresh_mass - fuel_burned * (1.0 + gas::AFR_STOICH), 0.0);
        self.fuel_mass = clamp(self.fuel_mass - fuel_burned, 0.0, self.fresh_mass);
        if to >= 0.999 {
            self.armed = false;
        }
        q
    }

    /// Woschni's convective heat transfer coefficient times the chamber surface, W into the gas.
    fn woschni(&mut self, si: &SpecInstance, p: f64, v: f64, omega: f64, temp: f64) -> f64 {
        let spec = &si.spec;
        if si.id != self.woschni_spec {
            self.woschni_spec = si.id;
            self.woschni_bore = math::pow(spec.bore, -0.2);
            self.woschni_omega = f64::NAN;
        }
        if omega != self.woschni_omega {
            self.woschni_omega = omega;
            self.woschni_piston_speed = (omega.abs() / (2.0 * PI)) * 2.0 * spec.stroke;
            self.woschni_speed_exchange = math::pow(WOSCHNI_C1_EXCHANGE * self.woschni_piston_speed, 0.8);
            self.woschni_speed_closed = math::pow(WOSCHNI_C1_CLOSED * self.woschni_piston_speed, 0.8);
        }
        let rise = p - self.motored_pressure;
        let pw = if self.exchanging {
            math::pow(p / 1000.0, 0.8) * self.woschni_speed_exchange
        } else if self.ref_pressure > 0.0 && rise > 0.0 {
            let w = WOSCHNI_C1_CLOSED * self.woschni_piston_speed + self.combustion_velocity * rise;
            math::pow((p / 1000.0) * w, 0.8)
        } else {
            math::pow(p / 1000.0, 0.8) * self.woschni_speed_closed
        };
        let h = 3.26 * self.woschni_bore * pw * math::pow(temp, -0.55);

        let bore = spec.bore;
        let cross_section = (PI * bore * bore) / 4.0;
        let height = v / cross_section;
        let area = 2.0 * cross_section + PI * bore * height;

        -h * area * (temp - gas::T_WALL)
    }
}

/// Laminar burning velocity of gasoline at 298 K and 1 atm, m/s, before dilution.
pub fn laminar_speed_base(phi: f64) -> f64 {
    let d = phi - SL_PHI_PEAK;
    SL_PEAK + SL_CURVE * d * d
}

/// Laminar burning velocity, m/s: Rhodes and Keck's correlation, with a floored dilution term.
pub fn laminar_flame_speed(phi: f64, t: f64, p: f64, residual: f64) -> f64 {
    let base = laminar_speed_base(phi);
    if !(base > 0.0) {
        return 0.0;
    }
    let alpha = 2.4 - 0.271 * math::pow(phi, 3.51);
    let beta = -0.357 + 0.14 * math::pow(phi, 2.77);
    let dilution = math::max(1.0 - 2.06 * math::pow(math::max(residual, 0.0), 0.77), SL_DILUTION_FLOOR);
    base * math::pow(t / 298.0, alpha) * math::pow(p / 101325.0, beta) * dilution
}

/// Kinematic viscosity of the unburned charge, m^2/s.
fn kinematic_viscosity(t: f64, p: f64) -> f64 {
    (3.3e-7 * math::pow(t, 0.7) * gas::R * t) / p
}

/// Where the spark fires under the advance map, deg.
fn advanced_spark(spec: &EngineSpec, nominal: f64, predicted: f64) -> f64 {
    let shifted = nominal - WIEBE_HALF * (predicted - spec.burn_duration);
    let advance = clamp(720.0 - shifted, MIN_ADVANCE, MAX_ADVANCE);
    720.0 - advance
}

/// Wiebe burn duration, deg, for the charge the spark finds: the entrainment picture of Blizard and
/// Keck, rescaled from the reference flame state `spec.burn_duration` is stated at.
pub fn burn_angle(spec: &EngineSpec, omega: f64, p: f64, t: f64, phi: f64, residual: f64) -> f64 {
    let d = derived();
    let piston_speed = (spec.stroke * omega.abs()) / PI;
    let turbulence = math::max(TURBULENCE_PER_PISTON_SPEED * piston_speed, 1e-3);
    let laminar = math::max(laminar_flame_speed(phi, t, p, residual), 1e-3);
    let front = (REF_TURBULENCE + d.ref_laminar) / (turbulence + laminar);
    let burnup = (d.ref_laminar / laminar)
        * math::sqrt((kinematic_viscosity(t, p) / d.ref_viscosity) * (REF_TURBULENCE / turbulence));
    spec.burn_duration * (piston_speed / REF_PISTON_SPEED) * ((1.0 - BURNUP_SHARE) * front + BURNUP_SHARE * burnup)
}

/// Hardenberg and Hase's ignition delay, deg, of diesel fuel injected into air at `p` Pa and `t` K,
/// at a mean piston speed of `piston_speed` m/s.
pub fn ignition_delay(piston_speed: f64, p: f64, t: f64) -> f64 {
    let activation = 618_840.0 / (CETANE + 25.0);
    let bar = math::max(p / 1e5, DELAY_PRESSURE_FLOOR);
    let exponent = activation * (1.0 / (R_UNIVERSAL * t) - 1.0 / 17_190.0) + math::pow(21.2 / (bar - 12.4), 0.63);
    math::min((0.36 + 0.22 * piston_speed) * math::exp(exponent), MAX_IGNITION_DELAY)
}

/// Watson's share of a diesel's fuel that burns premixed, 0..1: what mixed with the air during an
/// ignition delay of `delay` s, at overall equivalence ratio `phi`.
pub fn premixed_share(phi: f64, delay: f64) -> f64 {
    let ms = math::max(delay * 1e3, 1e-3);
    let share = 1.0 - (PREMIXED_A * math::pow(math::max(phi, 0.0), PREMIXED_B)) / math::pow(ms, PREMIXED_C);
    clamp(share, 0.0, MAX_PREMIXED_SHARE)
}

/// Wiebe mass fraction burned with form factor `m`, `a = 5`, normalised to reach exactly 1 at the
/// duration.
#[inline]
pub fn wiebe_m(deg_after_start: f64, duration: f64, m: f64) -> f64 {
    if deg_after_start <= 0.0 {
        return 0.0;
    }
    if deg_after_start >= duration {
        return 1.0;
    }
    let u = deg_after_start / duration;
    (1.0 - math::exp(-5.0 * math::pow(u, m + 1.0))) * derived().wiebe_norm
}

/// Wiebe mass fraction burned, `a = 5`, `m = 2`, normalised to reach exactly 1 at the duration.
#[inline]
pub fn wiebe(deg_after_spark: f64, duration: f64) -> f64 {
    if deg_after_spark <= 0.0 {
        return 0.0;
    }
    if deg_after_spark >= duration {
        return 1.0;
    }
    let u = deg_after_spark / duration;
    (1.0 - math::exp(-5.0 * u * u * u)) * derived().wiebe_norm
}

/// Sensible energy at `t`, J/kg.
fn energy_at(t: f64) -> f64 {
    let d = t - T_REF;
    d * (CV_REF + HALF_SLOPE * d)
}

/// True if the crank swept past `target` between `from` and `to` (720-deg wrapping).
#[inline]
fn crossed(from: f64, to: f64, target: f64) -> bool {
    if to >= from {
        return target > from && target <= to;
    }
    target > from || target <= to
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wiebe_runs_from_zero_to_one() {
        assert_eq!(wiebe(-1.0, 50.0), 0.0);
        assert_eq!(wiebe(50.0, 50.0), 1.0);
        let mid = wiebe(25.0, 50.0);
        assert!(mid > 0.3 && mid < 0.7);
    }

    #[test]
    fn a_motored_cylinder_compresses() {
        let spec = SpecInstance::new(EngineSpec::default());
        let mut cyl = Cylinder::new(&spec.spec, 540.0, 1.0);
        let p0 = cyl.pressure(&spec.spec);
        let omega = 3000.0 * 2.0 * PI / 60.0;
        for _ in 0..(48000 / 100) {
            cyl.advance_with(&spec, 1.0 / 48000.0, omega, 0.0, 0.0, 300.0, 900.0, 0.0, None);
        }
        assert!(cyl.pressure(&spec.spec) > 5.0 * p0);
    }
}
