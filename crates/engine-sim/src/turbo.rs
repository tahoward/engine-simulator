//! Turbocharger: a turbine in the exhaust driving a compressor on the same shaft, the charge air
//! between the compressor and the throttle, the wastegate that caps the boost, and the blow-off valve
//! that vents it when the throttle shuts. And the sounds all of that makes.
//!
//! ```text
//!   exhaust valves -> turbine (back pressure on the exhaust) --- shaft --- compressor
//!                        |                                                   |
//!                    wastegate                                  compressor duct (inertia)
//!                                                                            |
//!   plenum <- throttle <----------------- charge air (intercooler) <---------+---> blow-off valve
//! ```
//!
//! Lumped, one state per part, stepped once per audio sample. The turbine is a Stodola nozzle fed by
//! the exhaust flow the cylinders push out; its inlet pressure is applied as the pressure the exhaust
//! mouths open into, so the pumping loss and the residual gas it costs come out of the cylinder model.
//! The compressor is a Moore-Greitzer characteristic with the inertia of the air in its duct, which is
//! what makes it surge when the throttle shuts on boost with nowhere for the air to go.
//!
//! Twin turbos are identical and in parallel, so one lumped shaft stands for both; the sound gives
//! each its own voice, a little apart in speed as two real ones are.

use crate::dsp::{Impact, Noise, OnePole, Resonator};
use crate::math::{self, PI, clamp};
use crate::radiation::FarField;
use crate::spec::{BlowOff, EngineSpec, displacement, gas, gas_energy, gas_enthalpy, gas_temperature};
use crate::valve::orifice_mass_flow;

/// Ratio of specific heats and specific heat at constant pressure of the air, J/(kg*K).
const GAMMA_AIR: f64 = gas::GAMMA_AIR;
const CP_AIR: f64 = GAMMA_AIR * gas::R / (GAMMA_AIR - 1.0);
/// The same of the exhaust gas through the turbine.
const GAMMA_EXH: f64 = gas::GAMMA_EXH;
const CP_EXH: f64 = GAMMA_EXH * gas::R / (GAMMA_EXH - 1.0);

/// Isentropic efficiencies of the compressor and the turbine, at best.
const ETA_COMPRESSOR: f64 = 0.72;
const ETA_TURBINE: f64 = 0.68;

/// The compressor's efficiency island: best at `ETA_BEST_FLOW` of the choke flow on its speed line,
/// falling as the square of the distance from there, to `ETA_FLOOR` of its best at the choke. Near
/// the choke the wheel does much more work for the same pressure rise, which is what a turbo too
/// small for the engine runs out of at the top end.
const ETA_BEST_FLOW: f64 = 0.6;
const ETA_FALLOFF: f64 = 0.8;
const ETA_FLOOR: f64 = 0.45;

/// The compressor's characteristic, `f(phi)` for flow `phi` as a fraction of its choke flow at full
/// speed: a Moore-Greitzer cubic, `F0 + H (1 + 1.5 y - 0.5 y^3)` with `y = phi / W - 1`. It has its
/// peak pressure rise, 1, at `phi = 2 W`, the surge line, at 44% of the choke flow as on a typical
/// map; to the left of that the pressure falls with falling flow, which is what makes the compression
/// system unstable there. It falls to zero, the choke, at `phi = 1`, and shut off the wheel still
/// makes 89% of its peak.
const MG_F0: f64 = 0.889;
const MG_H: f64 = 0.0556;
const MG_W: f64 = 0.22;

/// Peak pressure rise at full speed as a multiple of the boost target: the headroom the wastegate
/// has to take away.
const PRESSURE_HEADROOM: f64 = 1.5;

/// Choke flow as a multiple of what the engine draws at its peak-power speed on full boost.
const FLOW_HEADROOM: f64 = 1.25;
/// Volumetric efficiency the engine's airflow is estimated at, for sizing, and the charge
/// temperature, K.
const SIZING_VE: f64 = 0.9;
const SIZING_CHARGE_T: f64 = 320.0;

/// Axial velocity through the inducer at choke as a fraction of the tip speed, the inducer's
/// diameter as a fraction of the wheel's, and the tip speed's share of the work the wheel does.
const INDUCER_VELOCITY_RATIO: f64 = 0.35;
const INDUCER_RATIO: f64 = 0.7;
const WORK_COEFFICIENT: f64 = 0.75;

/// Polar moment of inertia of one rotor with a 50 mm compressor wheel, kg*m^2; it goes as the
/// wheel's diameter to the fifth.
const INERTIA_AT_50MM: f64 = 2.5e-5;

/// Bearing drag at full speed, as a fraction of the compressor's power at its design point.
const FRICTION_SHARE: f64 = 0.02;

/// Turbine inlet pressure ratio, wastegate shut, at the design exhaust flow. Sizes the nozzle.
const TURBINE_DESIGN_PR: f64 = 2.0;
/// Temperature the turbine is sized at, K.
const TURBINE_DESIGN_T: f64 = 1100.0;
/// Flow capacity of the wastegate, wide open, as a multiple of the turbine's.
const WASTEGATE_CAPACITY: f64 = 1.5;
/// Boost below the target at which the wastegate starts to open, and the rise over which it goes from
/// shut to wide open, Pa.
const WASTEGATE_CRACK: f64 = 0.04e5;
const WASTEGATE_SPAN: f64 = 0.12e5;
/// Response time of the wastegate actuator and of the blow-off valve, s.
const WASTEGATE_TAU: f64 = 0.04;
const BLOW_OFF_TAU: f64 = 0.004;

/// Pressure across the throttle, charge side over plenum, at which the blow-off valve starts to open,
/// and the further rise at which it is wide open, Pa.
const BLOW_OFF_CRACK: f64 = 0.3e5;
const BLOW_OFF_SPAN: f64 = 0.15e5;
/// Blow-off valve bore as a fraction of the compressor inducer's.
const BLOW_OFF_AREA_RATIO: f64 = 0.8;

/// Charge-air volume, compressor to throttle through the intercooler, as a multiple of total swept
/// volume.
const CHARGE_VOLUME_RATIO: f64 = 2.0;
/// Helmholtz frequency of the charge-air volume on the compressor duct, Hz: sets the duct's
/// inertia, and so how fast a surge cycles.
const HELMHOLTZ_HZ: f64 = 18.0;
/// Loss coefficient of the flow through a compressor too slow to do any work, windmilling.
const WINDMILL_LOSS: f64 = 1.0;

/// Speeds below which the compressor is taken to be this fraction of full speed, for its
/// characteristic only, so it does not divide by zero.
const MIN_SPEED_FRACTION: f64 = 0.02;

/// Smoothing of the exhaust flow into the turbine, Hz: the rotor sees the mean of the pulses.
const EXHAUST_SMOOTHING_HZ: f64 = 40.0;

// --- Sound ---

/// Blades on the compressor wheel, not counting splitters: the order of the blade-pass tone.
const COMPRESSOR_BLADES: f64 = 6.0;
/// The whine's harmonics, in shaft orders, and their levels relative to the blade-pass tone.
const WHINE_ORDERS: [(f64, f64); 5] =
    [(1.0, 0.22), (2.0, 0.12), (3.0, 0.08), (COMPRESSOR_BLADES, 1.0), (2.0 * COMPRESSOR_BLADES, 0.3)];
/// Blade-pass tone at 1 m at full speed and full flow, Pa, at `turbo_noise = 1`.
const WHINE_PA: f64 = 1.6;
/// How much of the whine a freely spinning, unloaded wheel still makes.
const WHINE_UNLOADED: f64 = 0.25;
/// Relative speed of the second turbo of a pair.
const TWIN_DETUNE: f64 = 1.012;
/// Hiss of the air into the compressor at full flow, Pa at 1 m.
const INLET_HISS_PA: f64 = 0.5;

/// Blow-off jet noise at 1 m, Pa, per kg/s vented at one bar across the valve.
const BLOW_OFF_PA: f64 = 90.0;
/// The valve's pop as it lifts off its seat, Pa at 1 m.
const BLOW_OFF_POP_PA: f64 = 25.0;
/// How much quieter a recirculating valve is, venting back into the compressor inlet.
const RECIRCULATING_SHARE: f64 = 0.1;

/// Chuff of the air reversing through the compressor in a surge, Pa at 1 m per kg/s of reverse flow.
const SURGE_PA: f64 = 90.0;

/// Wastegate flap rattle at 1 m, Pa, per exhaust pulse, and the bypass hiss wide open.
const WASTEGATE_RATTLE_PA: f64 = 3.0;
const WASTEGATE_HISS_PA: f64 = 0.6;

/// How much of the exhaust note the turbine passes, and the corner of its muffling, Hz.
const TURBINE_PASS: f64 = 0.55;
const TURBINE_MUFFLE_HZ: f64 = 2200.0;

/// Everything about the turbo that follows from the spec.
#[derive(Clone, Copy, Debug)]
struct Sizing {
    count: f64,
    /// Choke flow of all the compressors together at full speed, kg/s.
    choke_flow: f64,
    /// Peak pressure rise at full speed, Pa.
    peak_rise: f64,
    /// Full shaft speed, rad/s.
    full_speed: f64,
    /// All the rotors together, kg*m^2.
    inertia: f64,
    friction: f64,
    /// Stodola flow constant of the turbines and of the wastegates wide open, kg*sqrt(K)/(s*Pa).
    turbine_k: f64,
    wastegate_k: f64,
    /// Inducer area, all together, m^2.
    inducer_area: f64,
    /// Compressor duct area over length, m.
    duct_a_over_l: f64,
    charge_volume: f64,
    blow_off_area: f64,
    boost_target: f64,
    intercooler: f64,
    blow_off: BlowOff,
    noise: f64,
}

fn sizing(spec: &EngineSpec) -> Sizing {
    let count = if spec.turbo_count >= 2.0 { 2.0 } else { 1.0 };
    let boost_target = math::max(spec.boost_target, 0.05e5);
    let swept = displacement(spec) * math::max(spec.cylinders as f64, 1.0);

    let choke_flow = if spec.turbo_size > 0.0 {
        spec.turbo_size * count
    } else {
        let rho = (gas::P_AMB + boost_target) / (gas::R * SIZING_CHARGE_T);
        FLOW_HEADROOM * rho * swept * (0.8 * spec.rev_limit / 120.0) * SIZING_VE
    };
    let peak_rise = PRESSURE_HEADROOM * boost_target;

    // The tip speed that does the work of the peak pressure rise, and the wheel that passes the choke
    // flow at it.
    let pr = (gas::P_AMB + peak_rise) / gas::P_AMB;
    let work = CP_AIR * gas::T_AMB * (math::pow(pr, (GAMMA_AIR - 1.0) / GAMMA_AIR) - 1.0) / ETA_COMPRESSOR;
    let tip = math::sqrt(work / WORK_COEFFICIENT);
    let rho_amb = gas::P_AMB / (gas::R * gas::T_AMB);
    let inducer_one = (choke_flow / count) / (rho_amb * INDUCER_VELOCITY_RATIO * tip);
    let wheel = math::sqrt((4.0 * inducer_one) / PI) / INDUCER_RATIO;
    let full_speed = (2.0 * tip) / wheel;
    let inertia = count * INERTIA_AT_50MM * math::pow(wheel / 0.05, 5.0);
    let friction = (FRICTION_SHARE * choke_flow * work) / (full_speed * full_speed);

    let exhaust_flow = (choke_flow / FLOW_HEADROOM) * (1.0 + 1.0 / gas::AFR_STOICH);
    let turbine_k = (exhaust_flow * math::sqrt(TURBINE_DESIGN_T))
        / (gas::P_AMB * math::sqrt(TURBINE_DESIGN_PR * TURBINE_DESIGN_PR - 1.0));

    let charge_volume = math::max(CHARGE_VOLUME_RATIO * swept, 1e-4);
    let c = math::sqrt(GAMMA_AIR * gas::R * gas::T_AMB);
    let k = (2.0 * PI * HELMHOLTZ_HZ) / c;
    Sizing {
        count,
        choke_flow,
        peak_rise,
        full_speed,
        inertia,
        friction,
        turbine_k,
        wastegate_k: WASTEGATE_CAPACITY * turbine_k,
        inducer_area: inducer_one * count,
        duct_a_over_l: k * k * charge_volume,
        charge_volume,
        blow_off_area: BLOW_OFF_AREA_RATIO * inducer_one * count,
        boost_target,
        intercooler: clamp(spec.intercooler, 0.0, 1.0),
        blow_off: spec.blow_off,
        noise: math::max(spec.turbo_noise, 0.0),
    }
}

/// What the turbo hands the engine each sample.
#[derive(Clone, Copy, Debug)]
pub struct TurboOut {
    /// Pressure and temperature of the charge air the throttle draws from, Pa and K.
    pub charge_p: f64,
    pub charge_t: f64,
    /// Pressure at the turbine inlet, which the exhaust mouths open into, Pa.
    pub back_pressure: f64,
    /// What the turbo radiates this sample, Pa at 1 m.
    pub sound: f64,
}

pub struct Turbo {
    sample_rate: f64,
    size: Sizing,
    /// Shaft speed, rad/s.
    omega: f64,
    /// Flow through the compressor duct, kg/s, positive toward the engine.
    compressor_flow: f64,
    /// Charge-air volume's gas: mass, kg, and sensible internal energy, J.
    mass: f64,
    energy: f64,
    wastegate: f64,
    blow_off: f64,
    blow_off_flow: f64,
    /// The exhaust flow into the turbine, kg/s, and its temperature, K, smoothed over the pulses.
    exhaust_flow: f64,
    exhaust_temp: f64,
    smoothing_c: f64,
    back_pressure: f64,
    /// Seconds since the compressor flow last ran backwards.
    since_reverse: f64,

    // Sound
    phase: [[f64; 5]; 2],
    noise: Noise,
    whine_level: OnePole,
    hiss: Resonator,
    blow_off_bands: [Resonator; 2],
    blow_off_pop: Impact,
    blow_off_body: Resonator,
    blow_off_was_open: bool,
    surge_band: Resonator,
    surge_thump: FarField,
    rattle: Impact,
    rattle_modes: [Resonator; 2],
    muffle: OnePole,
}

impl Turbo {
    pub fn new(spec: &EngineSpec, sample_rate: f64) -> Turbo {
        let size = sizing(spec);
        let mass = (gas::P_AMB * size.charge_volume) / (gas::R * gas::T_AMB);
        let mut whine_level = OnePole::default();
        whine_level.set_cutoff(30.0, sample_rate);
        let mut muffle = OnePole::default();
        muffle.set_cutoff(TURBINE_MUFFLE_HZ, sample_rate);
        let inlet_radius = math::max(math::sqrt(size.inducer_area / PI), 0.01);
        Turbo {
            sample_rate,
            omega: 0.03 * size.full_speed,
            compressor_flow: 0.0,
            mass,
            energy: mass * gas_energy(gas::T_AMB),
            wastegate: 0.0,
            blow_off: 0.0,
            blow_off_flow: 0.0,
            exhaust_flow: 0.0,
            exhaust_temp: TURBINE_DESIGN_T,
            smoothing_c: 1.0 - math::exp((-2.0 * PI * EXHAUST_SMOOTHING_HZ) / sample_rate),
            back_pressure: gas::P_AMB,
            since_reverse: f64::INFINITY,
            phase: [[0.0; 5]; 2],
            noise: Noise::new(0x7ab0_c3d1 as f64),
            whine_level,
            hiss: Resonator::new(4500.0, 0.8, sample_rate),
            blow_off_bands: [Resonator::new(2300.0, 1.6, sample_rate), Resonator::new(5600.0, 1.2, sample_rate)],
            blow_off_pop: Impact::new(0.0012, sample_rate),
            blow_off_body: Resonator::new(850.0, 5.0, sample_rate),
            blow_off_was_open: false,
            surge_band: Resonator::new(1400.0, 1.4, sample_rate),
            surge_thump: FarField::new(sample_rate, (2.0 * 343.0) / inlet_radius),
            rattle: Impact::new(0.0002, sample_rate),
            rattle_modes: [Resonator::new(1850.0, 12.0, sample_rate), Resonator::new(3300.0, 15.0, sample_rate)],
            muffle,
            size,
        }
    }

    /// Resize for `spec`, keeping the shaft speed and the charge air.
    pub fn configure(&mut self, spec: &EngineSpec) {
        let size = sizing(spec);
        let scale = size.charge_volume / self.size.charge_volume;
        self.mass *= scale;
        self.energy *= scale;
        self.omega = clamp(self.omega, 0.0, 1.2 * size.full_speed);
        self.size = size;
    }

    pub fn charge_temp(&self) -> f64 {
        clamp(gas_temperature(self.energy / math::max(self.mass, 1e-9)), 150.0, 1000.0)
    }

    pub fn charge_pressure(&self) -> f64 {
        (math::max(self.mass, 1e-9) * gas::R * self.charge_temp()) / self.size.charge_volume
    }

    /// Boost, gauge, Pa.
    pub fn boost(&self) -> f64 {
        self.charge_pressure() - gas::P_AMB
    }

    /// Speed of each turbo's shaft, rev/min.
    pub fn shaft_rpm(&self) -> f64 {
        (self.omega * 60.0) / (2.0 * PI)
    }

    /// Frequency of the compressor blade-pass tone, Hz.
    pub fn blade_pass_hz(&self) -> f64 {
        (self.omega / (2.0 * PI)) * COMPRESSOR_BLADES
    }

    pub fn compressor_flow(&self) -> f64 {
        self.compressor_flow
    }

    pub fn wastegate(&self) -> f64 {
        self.wastegate
    }

    pub fn blow_off(&self) -> f64 {
        self.blow_off
    }

    pub fn blow_off_flow(&self) -> f64 {
        self.blow_off_flow
    }

    /// Whether the compressor has run backwards in the last tenth of a second.
    pub fn surging(&self) -> bool {
        self.since_reverse < 0.1
    }

    /// Isentropic efficiency of the compressor at flow `m` (kg/s) at its present speed.
    fn compressor_efficiency(&self, m: f64) -> f64 {
        let s = math::max(self.omega / self.size.full_speed, MIN_SPEED_FRACTION);
        let phi = m.abs() / (s * self.size.choke_flow);
        let d = (phi - ETA_BEST_FLOW) / (1.0 - ETA_BEST_FLOW);
        ETA_COMPRESSOR * math::max(1.0 - ETA_FALLOFF * d * d, ETA_FLOOR)
    }

    /// Pressure at the turbine inlet, Pa.
    pub fn back_pressure(&self) -> f64 {
        self.back_pressure
    }

    /// Pressure rise across the compressor, Pa, at flow `m` (kg/s).
    fn compressor_rise(&self, m: f64) -> f64 {
        let s = self.omega / self.size.full_speed;
        let s2 = s * s;
        let rho = gas::P_AMB / (gas::R * gas::T_AMB);
        let a = self.size.inducer_area;
        let loss = (WINDMILL_LOSS * m * m.abs()) / (2.0 * rho * a * a);
        if m <= 0.0 {
            // Backwards through the wheel: its shut-off pressure, plus the loss of forcing air the
            // wrong way through the passages.
            return self.size.peak_rise * s2 * MG_F0 - loss;
        }
        let phi = math::min(m / (math::max(s, MIN_SPEED_FRACTION) * self.size.choke_flow), 5.0);
        let y = phi / MG_W - 1.0;
        let mg = self.size.peak_rise * s2 * (MG_F0 + MG_H * (1.0 + 1.5 * y - 0.5 * y * y * y));
        // Past its choke the wheel does no more work and is only a restriction.
        math::max(mg, -loss)
    }

    /// Advance by `dt`.
    ///
    /// `exhaust_flow` is the exhaust valves' mass flow, kg/s, at `exhaust_temp` (K); `throttle_flow`
    /// the flow the plenum drew through the throttle this sample, kg/s; `plenum_p` its pressure, Pa.
    /// `pulses` is how many exhaust valves started to open this sample.
    pub fn step(
        &mut self,
        dt: f64,
        exhaust_flow: f64,
        exhaust_temp: f64,
        throttle_flow: f64,
        plenum_p: f64,
        pulses: u32,
    ) -> TurboOut {
        let size = self.size;
        self.exhaust_flow += self.smoothing_c * (exhaust_flow - self.exhaust_flow);
        self.exhaust_temp += self.smoothing_c * (exhaust_temp - self.exhaust_temp);
        let m_ex = math::max(self.exhaust_flow, 0.0);
        let t_ex = clamp(self.exhaust_temp, 400.0, 1400.0);

        // --- Wastegate: opens on boost over its spring ---
        let boost = self.boost();
        let wg_target = clamp((boost - (size.boost_target - WASTEGATE_CRACK)) / WASTEGATE_SPAN, 0.0, 1.0);
        self.wastegate += (dt / WASTEGATE_TAU) * (wg_target - self.wastegate);
        self.wastegate = clamp(self.wastegate, 0.0, 1.0);

        // --- Turbine: Stodola's ellipse, the turbine and the wastegate in parallel ---
        let k_total = size.turbine_k + size.wastegate_k * self.wastegate;
        let q = (m_ex * math::sqrt(t_ex)) / k_total;
        let p_in = math::sqrt(gas::P_AMB * gas::P_AMB + q * q);
        self.back_pressure = p_in;
        let m_turbine = m_ex * (size.turbine_k / k_total);
        let expansion = 1.0 - math::pow(gas::P_AMB / p_in, (GAMMA_EXH - 1.0) / GAMMA_EXH);
        let p_turbine = ETA_TURBINE * m_turbine * CP_EXH * t_ex * expansion;

        // --- Compressor: its duct's air accelerated by the pressure rise against the charge ---
        let p2 = self.charge_pressure();
        let t2 = self.charge_temp();
        let rise = self.compressor_rise(self.compressor_flow);
        self.compressor_flow += dt * size.duct_a_over_l * (gas::P_AMB + rise - p2);
        let m_c = self.compressor_flow;
        let pr = math::max((gas::P_AMB + math::max(rise, 0.0)) / gas::P_AMB, 1.0);
        let heating = (math::pow(pr, (GAMMA_AIR - 1.0) / GAMMA_AIR) - 1.0) / self.compressor_efficiency(m_c);
        let p_compressor = m_c.abs() * CP_AIR * gas::T_AMB * heating;
        if m_c < 0.0 {
            self.since_reverse = 0.0;
        } else {
            self.since_reverse += dt;
        }

        // --- Shaft ---
        let drag = size.friction * self.omega * self.omega;
        let omega = math::max(self.omega, 0.02 * size.full_speed);
        self.omega += (dt * (p_turbine - p_compressor - drag)) / (size.inertia * omega);
        self.omega = clamp(self.omega, 0.0, 2.0 * size.full_speed);

        // --- Blow-off valve: opens on the pressure across a shut throttle ---
        let across = p2 - plenum_p;
        let bov_target = match size.blow_off {
            BlowOff::None => 0.0,
            _ => clamp((across - BLOW_OFF_CRACK) / BLOW_OFF_SPAN, 0.0, 1.0),
        };
        self.blow_off += (dt / BLOW_OFF_TAU) * (bov_target - self.blow_off);
        self.blow_off = clamp(self.blow_off, 0.0, 1.0);
        let vent = orifice_mass_flow(size.blow_off_area * self.blow_off, 0.7, p2, t2, gas::P_AMB, GAMMA_AIR);
        self.blow_off_flow = vent;

        // --- Charge air: in from the compressor through the intercooler, out through the throttle ---
        let t_in = if m_c >= 0.0 {
            let t_out = gas::T_AMB * (1.0 + heating);
            t_out - size.intercooler * (t_out - gas::T_AMB)
        } else {
            t2
        };
        let h2 = gas_enthalpy(t2);
        let out = throttle_flow + vent;
        let h_in = if m_c >= 0.0 { m_c * gas_enthalpy(t_in) } else { m_c * h2 };
        let h_out = if throttle_flow >= 0.0 { throttle_flow * h2 } else { throttle_flow * gas_enthalpy(t2) };
        self.energy += (h_in - h_out - vent * h2) * dt;
        self.mass += (m_c - out) * dt;
        let floor = 0.2 * (gas::P_AMB * size.charge_volume) / (gas::R * gas::T_AMB);
        if self.mass < floor || !self.mass.is_finite() || !self.energy.is_finite() {
            self.mass = math::max(if self.mass.is_finite() { self.mass } else { floor }, floor);
            self.energy = self.mass * gas_energy(gas::T_AMB);
        }
        let e_min = self.mass * gas_energy(200.0);
        if self.energy < e_min {
            self.energy = e_min;
        }

        let sound = self.sound(m_c, vent, pulses);
        TurboOut { charge_p: self.charge_pressure(), charge_t: self.charge_temp(), back_pressure: p_in, sound }
    }

    /// The exhaust note after the turbine, which takes the edge off every pulse it extracts work from.
    pub fn muffle(&mut self, exhaust_pa: f64) -> f64 {
        TURBINE_PASS * self.muffle.process(exhaust_pa)
    }

    fn sound(&mut self, m_c: f64, vent: f64, pulses: u32) -> f64 {
        let size = self.size;
        let level = size.noise;
        if level <= 0.0 {
            return 0.0;
        }
        let s = self.omega / size.full_speed;
        let flow_share = clamp(m_c / size.choke_flow, 0.0, 1.2);
        let loading = WHINE_UNLOADED + (1.0 - WHINE_UNLOADED) * math::sqrt(flow_share);
        let whine_amp = self.whine_level.process(WHINE_PA * level * s * s * loading);

        // --- Whine: shaft orders and blade pass, faded out below Nyquist ---
        let fade_top = math::min(0.45 * self.sample_rate, 18_000.0);
        let fade_start = 0.7 * fade_top;
        let shaft_hz = self.omega / (2.0 * PI);
        let mut whine = 0.0;
        let voices = size.count as usize;
        for v in 0..voices {
            let speed = if v == 0 { shaft_hz } else { shaft_hz * TWIN_DETUNE };
            for (h, &(order, rel)) in WHINE_ORDERS.iter().enumerate() {
                let hz = speed * order;
                let fade = if hz >= fade_top {
                    0.0
                } else if hz <= fade_start {
                    1.0
                } else {
                    1.0 - (hz - fade_start) / (fade_top - fade_start)
                };
                let ph = &mut self.phase[v][h];
                *ph += hz / self.sample_rate;
                *ph -= ph.floor();
                if fade > 0.0 {
                    whine += rel * fade * math::sin(2.0 * PI * *ph);
                }
            }
        }
        whine *= whine_amp / size.count;
        let hiss = self.hiss.process(self.noise.next()) * INLET_HISS_PA * level * flow_share * flow_share;

        // --- Blow-off: the jet of vented air, and the valve's pop as it lifts ---
        // Both go as the boost it lets go of: opening on a vacuum with no boost behind it, it is silent.
        let vent_share = if matches!(size.blow_off, BlowOff::Recirculating) { RECIRCULATING_SHARE } else { 1.0 };
        let boost = math::max(self.boost(), 0.0);
        let open = self.blow_off > 0.05;
        if open && !self.blow_off_was_open {
            self.blow_off_pop.trigger(BLOW_OFF_POP_PA * level * vent_share * math::min(boost / 0.5e5, 1.5));
        }
        self.blow_off_was_open = open;
        let jet_amp = BLOW_OFF_PA * level * vent_share * vent * math::sqrt(boost / 1e5);
        let n = self.noise.next();
        let jet = jet_amp * (self.blow_off_bands[0].process(n) + 0.7 * self.blow_off_bands[1].process(n));
        let pop = self.blow_off_body.process(self.blow_off_pop.next());

        // --- Surge: a thump as the flow collapses, and the chuff of air reversing through the wheel ---
        let thump = self.surge_thump.process(m_c / (gas::P_AMB / (gas::R * gas::T_AMB)));
        let reverse = math::max(-m_c, 0.0);
        let chuff = self.surge_band.process(self.noise.next()) * SURGE_PA * level * reverse * (0.3 + s);

        // --- Wastegate: the flap rattling on its seat with the exhaust pulses, and the bypass hiss ---
        let wg = self.wastegate;
        if pulses > 0 && wg > 0.0 && wg < 0.45 {
            let rattle = WASTEGATE_RATTLE_PA * level * (1.0 - wg / 0.45) * (0.6 + 0.4 * self.noise.next().abs());
            self.rattle.trigger(rattle);
        }
        let hit = self.rattle.next();
        let rattle = self.rattle_modes[0].process(hit) + 0.6 * self.rattle_modes[1].process(hit);
        let bypass = self.noise.next() * WASTEGATE_HISS_PA * level * wg * (self.back_pressure / gas::P_AMB - 1.0);

        whine + hiss + jet + pop + thump * level + chuff + rattle + bypass
    }
}
