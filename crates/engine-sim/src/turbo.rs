//! Turbocharger: a turbine in the exhaust driving a compressor on the same shaft, the charge air
//! between the compressor and the throttle, the wastegate that caps the boost, and the blow-off valve
//! that vents it when the throttle shuts. And the sounds all of that makes.
//!
//! ```text
//!   exhaust junction -> turbine (in the gas dynamics) --- shaft --- compressor <- inlet (radiates)
//!                          |                                             |
//!                      wastegate                            compressor duct (inertia)
//!                                                                        |
//!   plenum <- throttle <------------- charge air (intercooler) <---------+---> blow-off valve
//! ```
//!
//! The turbine is solved in the exhaust, at a junction, by `ExhaustSystem`: this sets its flow
//! constants from the wastegate each sample, and is driven by the power it reports. Everything else is
//! lumped, one state per part, stepped once per audio sample. The compressor is a Moore-Greitzer
//! characteristic with the inertia of the air in its duct, which is what makes it surge when the
//! throttle shuts on boost with nowhere for the air to go.
//!
//! What the turbo radiates comes from its flows: the compressor inlet's, carrying the blades' whine
//! and a stalled wheel's turbulence, and the blow-off valve's, with its jet's noise by Lighthill's law.
//!
//! Twin turbos are identical and in parallel, so one lumped shaft stands for both; the sound gives
//! each its own voice, a little apart in speed as two real ones are.

use crate::dsp::{Impact, Noise, Resonator};
use crate::exhaust_system::TurbineSetting;
use crate::math::{self, PI, clamp};
use crate::radiation::FarField;
use crate::spec::{BlowOff, EngineSpec, displacement, gas, gas_energy, gas_enthalpy, gas_temperature};
use crate::valve::orifice_mass_flow;

/// Ratio of specific heats and specific heat at constant pressure of the air, J/(kg*K).
const GAMMA_AIR: f64 = gas::GAMMA_AIR;
const CP_AIR: f64 = GAMMA_AIR * gas::R / (GAMMA_AIR - 1.0);

/// Isentropic efficiencies of the compressor and the turbine, at best.
const ETA_COMPRESSOR: f64 = 0.72;
const ETA_TURBINE: f64 = 0.78;

/// The turbine's map: its efficiency against its blade speed ratio `x = U / C0`, the wheel's tip
/// speed over the spouting velocity `C0 = sqrt(2 dh_s)` of the isentropic drop across it. A radial
/// inflow turbine does best near `x = 0.7`, and its efficiency falls away either side as
/// `ETA_TURBINE (2 r - r^2)`, `r = x / 0.7`: to nothing on a stalled wheel, which still takes torque
/// from the gas, and below nothing on a wheel spinning faster than the gas drives it, which then
/// churns the gas and brakes the shaft.
const TURBINE_BEST_BSR: f64 = 0.7;
/// Turbine wheel tip diameter as a fraction of the compressor wheel's.
const TURBINE_WHEEL_RATIO: f64 = 0.9;

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

/// Turbine power, wastegate shut, at the design exhaust flow, as a multiple of what the compressor
/// takes to make the boost target there: the headroom the wastegate bypasses. Sizes the nozzle, to a
/// pressure ratio near 2 at 0.7 bar and 7.5 at 2 bar.
const TURBINE_POWER_HEADROOM: f64 = 1.76;
/// Most of the isentropic enthalpy drop the nozzle is sized to take, as a share of the gas's: a
/// pressure ratio near 16.
const TURBINE_MAX_DROP: f64 = 0.5;
/// Temperature the turbine is sized at, K.
const TURBINE_DESIGN_T: f64 = 1100.0;
const GAMMA_EXH: f64 = gas::GAMMA_EXH;
const CP_EXH: f64 = GAMMA_EXH * gas::R / (GAMMA_EXH - 1.0);
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

// --- Sound ---

/// Blades on the compressor wheel, not counting splitters: the order of the blade-pass tone.
const COMPRESSOR_BLADES: f64 = 6.0;
/// The compressor's tones, in shaft orders, and their levels relative to the blade-pass tone: the
/// blade pass and its harmonic, and the shaft orders a real wheel's small imbalances put there.
const WHINE_ORDERS: [(f64, f64); 5] =
    [(1.0, 0.22), (2.0, 0.12), (3.0, 0.08), (COMPRESSOR_BLADES, 1.0), (2.0 * COMPRESSOR_BLADES, 0.3)];
/// How deeply the blades modulate the air drawn into the compressor at full speed: the blade-pass
/// tone's share of the inlet's volume flow. It goes as the square of the speed.
const WHINE_DEPTH: f64 = 0.005;
/// How much faster each turbo's whine runs than the one before's, so several beat rather than sounding
/// as one tone.
const TURBO_DETUNE: f64 = 1.012;

/// The turbine's pulsation of the exhaust flow at full speed, at the first two shaft orders, as a
/// share of its flow: its whistle, carried down the pipe.
const TURBINE_PULSATION: f64 = 0.004;
const TURBINE_ORDERS: [(f64, f64); 2] = [(1.0, 1.0), (2.0, 0.5)];

/// Lighthill's constant: the share of a jet's kinetic power `rho U^8 D^2 / c^5` it radiates as
/// sound. Measured jets give 0.3-1.2e-4.
const LIGHTHILL_K: f64 = 5e-5;
/// How much of a recirculating blow-off valve's jet noise gets out through the intake's ducting.
const RECIRCULATING_TRANSMISSION: f64 = 0.1;

/// The turbulence a stalled compressor sheds into its inlet, as a share of its flow at that speed:
/// what the flutter is made of. Stall starts at the surge line and is fully developed at no flow.
const STALL_INTENSITY: f64 = 0.06;

/// Wastegate flap rattle at 1 m, Pa, per 10 kPa of pulse across it, and how far open it can be and
/// still rattle.
const WASTEGATE_RATTLE_PA: f64 = 2.0;
const WASTEGATE_RATTLE_OPENING: f64 = 0.45;

/// Everything about the turbo that follows from the spec.
#[derive(Clone, Copy, Debug)]
struct Sizing {
    count: f64,
    /// Choke flow of all the compressors together at full speed, kg/s.
    choke_flow: f64,
    /// Peak pressure rise at full speed, Pa.
    peak_rise: f64,
    /// Full shaft speed, rad/s, and the turbine wheel's tip radius, m.
    full_speed: f64,
    turbine_radius: f64,
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

/// `turbos` is how many there are; they share the engine's airflow.
fn sizing(spec: &EngineSpec, turbos: usize) -> Sizing {
    let count = math::max(turbos as f64, 1.0);
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

    // The turbine nozzle: the pressure ratio at which, at the design flow, the turbine makes its
    // headroom over the work of compressing to the boost target, at the compressor's efficiency there.
    let exhaust_share = 1.0 + 1.0 / gas::AFR_STOICH;
    let exhaust_flow = (choke_flow / FLOW_HEADROOM) * exhaust_share;
    let d = (1.0 / FLOW_HEADROOM - ETA_BEST_FLOW) / (1.0 - ETA_BEST_FLOW);
    let eta_design = ETA_COMPRESSOR * math::max(1.0 - ETA_FALLOFF * d * d, ETA_FLOOR);
    let pr_target = (gas::P_AMB + boost_target) / gas::P_AMB;
    let compressor_work =
        CP_AIR * gas::T_AMB * (math::pow(pr_target, (GAMMA_AIR - 1.0) / GAMMA_AIR) - 1.0) / eta_design;
    // The turbine's wheel, at the speed the shaft makes the boost target at, does that work where its
    // map has it: `turbine_work` inverted for the spouting velocity.
    let turbine_radius = TURBINE_WHEEL_RATIO * wheel / 2.0;
    let tip_design = (full_speed * turbine_radius) / math::sqrt(PRESSURE_HEADROOM);
    let work_needed = (TURBINE_POWER_HEADROOM * compressor_work) / exhaust_share;
    let b = TURBINE_BEST_BSR;
    let c0 = ((work_needed / ETA_TURBINE + (tip_design * tip_design) / (2.0 * b * b)) * b) / tip_design;
    let drop = math::min((c0 * c0) / (2.0 * CP_EXH * TURBINE_DESIGN_T), TURBINE_MAX_DROP);
    let turbine_pr = math::pow(1.0 - drop, -GAMMA_EXH / (GAMMA_EXH - 1.0));
    let turbine_k =
        (exhaust_flow * math::sqrt(TURBINE_DESIGN_T)) / (gas::P_AMB * math::sqrt(turbine_pr * turbine_pr - 1.0));

    let charge_volume = math::max(CHARGE_VOLUME_RATIO * swept, 1e-4);
    let c = math::sqrt(GAMMA_AIR * gas::R * gas::T_AMB);
    let k = (2.0 * PI * HELMHOLTZ_HZ) / c;
    Sizing {
        count,
        choke_flow,
        peak_rise,
        full_speed,
        turbine_radius,
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

/// Work the turbine takes from each kg of gas through it, J/kg, with its wheel's tip at `tip_speed`
/// (m/s) and `isentropic` the isentropic enthalpy drop across it, J/kg: the drop at the efficiency
/// its map gives, written so it holds at no drop too.
pub fn turbine_work(tip_speed: f64, isentropic: f64) -> f64 {
    let c0 = math::sqrt(2.0 * math::max(isentropic, 0.0));
    let b = TURBINE_BEST_BSR;
    ETA_TURBINE * ((tip_speed * c0) / b - (tip_speed * tip_speed) / (2.0 * b * b))
}

/// Isentropic efficiency of the turbine at blade speed ratio `x`, from its map.
pub fn turbine_efficiency(x: f64) -> f64 {
    let r = x / TURBINE_BEST_BSR;
    ETA_TURBINE * (2.0 * r - r * r)
}

/// What drives the turbine this sample: the power it takes from the exhaust and the power of the
/// isentropic drop across it, W, the flow through it, kg/s, and the pressures at its inlet and outlet,
/// Pa.
#[derive(Clone, Copy, Debug)]
pub struct TurbineDrive {
    pub power: f64,
    pub isentropic_power: f64,
    pub flow: f64,
    pub inlet: f64,
    pub outlet: f64,
}

/// What the turbo hands the engine each sample.
#[derive(Clone, Copy, Debug)]
pub struct TurboOut {
    /// Pressure and temperature of the charge air the throttle draws from, Pa and K.
    pub charge_p: f64,
    pub charge_t: f64,
    /// What the turbo radiates this sample, Pa at 1 m.
    pub sound: f64,
}

/// Speed of the jet through an orifice from `p_up` (Pa) and `t_up` (K) to `p_down`, m/s: sonic at the
/// throat once it chokes.
fn jet_velocity(p_up: f64, t_up: f64, p_down: f64) -> f64 {
    if p_up <= p_down {
        return 0.0;
    }
    let k = (GAMMA_AIR - 1.0) / GAMMA_AIR;
    let critical = math::pow(2.0 / (GAMMA_AIR + 1.0), 1.0 / k);
    let pr = math::max(p_down / p_up, critical);
    math::sqrt(2.0 * CP_AIR * t_up * (1.0 - math::pow(pr, k)))
}

/// Sound pressure at 1 m, Pa RMS, of a jet of speed `u` (m/s) from a nozzle of diameter `d` (m), by
/// Lighthill's eighth-power law.
fn lighthill_pa(u: f64, d: f64) -> f64 {
    let rho = gas::P_AMB / (gas::R * gas::T_AMB);
    let c = math::sqrt(GAMMA_AIR * gas::R * gas::T_AMB);
    let u2 = u * u;
    let power = LIGHTHILL_K * rho * u2 * u2 * u2 * u2 * d * d / (c * c * c * c * c);
    math::sqrt((power * rho * c) / (4.0 * PI))
}

/// A jet's broadband noise: white noise band-limited either side of its Strouhal peak, `0.2 U / D`,
/// at a given RMS level.
struct JetNoise {
    lp: f64,
    hp: f64,
}

impl JetNoise {
    fn new() -> JetNoise {
        JetNoise { lp: 0.0, hp: 0.0 }
    }

    fn next(&mut self, noise: &mut Noise, u: f64, d: f64, pa_rms: f64, sample_rate: f64) -> f64 {
        if pa_rms <= 0.0 {
            self.lp *= 0.99;
            self.hp *= 0.99;
            return 0.0;
        }
        let peak = clamp((0.2 * u) / math::max(d, 1e-3), 100.0, 0.4 * sample_rate);
        let c_lp = 1.0 - math::exp((-2.0 * PI * 2.0 * peak) / sample_rate);
        let c_hp = 1.0 - math::exp((-2.0 * PI * 0.5 * peak) / sample_rate);
        // Uniform noise has an RMS of 1/sqrt(3); a one-pole low-pass keeps c/(2 - c) of its power.
        let norm = math::sqrt(3.0 * (2.0 - c_lp) / c_lp);
        self.lp += c_lp * (noise.next() - self.lp);
        self.hp += c_hp * (self.lp - self.hp);
        (self.lp - self.hp) * norm * pa_rms
    }
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
    back_pressure: f64,
    /// Where the turbine ran on its map last sample: its efficiency and blade speed ratio.
    turbine_efficiency: f64,
    blade_speed_ratio: f64,
    /// Seconds since the compressor flow last ran backwards.
    since_reverse: f64,

    // Sound
    noise: Noise,
    /// Each turbo's whine harmonics.
    whine_phase: Vec<[f64; 5]>,
    turbine_phase: [f64; 2],
    /// The compressor inlet and the atmospheric blow-off valve's outlet, radiating as the tailpipe does.
    inlet: FarField,
    vent: FarField,
    blow_off_jet: JetNoise,
    reverse_jet: JetNoise,
    stall_lp: f64,
    stall_hp: f64,
    /// The pressure drop across the turbine, smoothed, Pa, and whether it is above that now: the
    /// pulses the wastegate flap rattles on.
    across_mean: f64,
    across_high: bool,
    rattle: Impact,
    rattle_modes: [Resonator; 2],
    last_sound: f64,
}

impl Turbo {
    pub fn new(spec: &EngineSpec, turbos: usize, sample_rate: f64) -> Turbo {
        let size = sizing(spec, turbos);
        let mass = (gas::P_AMB * size.charge_volume) / (gas::R * gas::T_AMB);
        let c = math::sqrt(GAMMA_AIR * gas::R * gas::T_AMB);
        let inlet_radius = math::max(math::sqrt(size.inducer_area / PI), 0.01);
        let vent_radius = math::max(math::sqrt(size.blow_off_area / PI), 0.005);
        Turbo {
            sample_rate,
            omega: 0.03 * size.full_speed,
            compressor_flow: 0.0,
            mass,
            energy: mass * gas_energy(gas::T_AMB),
            wastegate: 0.0,
            blow_off: 0.0,
            back_pressure: gas::P_AMB,
            turbine_efficiency: 0.0,
            blade_speed_ratio: 0.0,
            since_reverse: f64::INFINITY,
            noise: Noise::new(0x7ab0_c3d1 as f64),
            whine_phase: vec![[0.0; 5]; size.count as usize],
            turbine_phase: [0.0; 2],
            inlet: FarField::new(sample_rate, (2.0 * c) / inlet_radius),
            vent: FarField::new(sample_rate, (2.0 * c) / vent_radius),
            blow_off_jet: JetNoise::new(),
            reverse_jet: JetNoise::new(),
            stall_lp: 0.0,
            stall_hp: 0.0,
            across_mean: 0.0,
            across_high: false,
            rattle: Impact::new(0.0002, sample_rate),
            rattle_modes: [Resonator::new(1850.0, 12.0, sample_rate), Resonator::new(3300.0, 15.0, sample_rate)],
            last_sound: 0.0,
            size,
        }
    }

    /// Resize for `spec` and `turbos` of them, keeping the shaft speed and the charge air.
    pub fn configure(&mut self, spec: &EngineSpec, turbos: usize) {
        let size = sizing(spec, turbos);
        let scale = size.charge_volume / self.size.charge_volume;
        self.mass *= scale;
        self.energy *= scale;
        self.omega = clamp(self.omega, 0.0, 1.2 * size.full_speed);
        self.whine_phase.resize(size.count as usize, [0.0; 5]);
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

    /// Whether the compressor has run backwards in the last tenth of a second.
    pub fn surging(&self) -> bool {
        self.since_reverse < 0.1
    }

    /// What the turbo itself radiated last sample, Pa at 1 m, before the listener: its own sounds
    /// alone, without the exhaust's.
    pub fn last_sound(&self) -> f64 {
        self.last_sound
    }

    /// Pressure at the turbine inlet, Pa.
    pub fn back_pressure(&self) -> f64 {
        self.back_pressure
    }

    /// The turbine's isentropic efficiency last sample, averaged over the flow through it: below
    /// nothing when it was braking the shaft.
    pub fn turbine_efficiency(&self) -> f64 {
        self.turbine_efficiency
    }

    /// The turbine's blade speed ratio last sample, its tip speed over the spouting velocity of the
    /// mean isentropic drop across it: 0.7 is where its map is best.
    pub fn blade_speed_ratio(&self) -> f64 {
        self.blade_speed_ratio
    }

    /// Isentropic efficiency of the compressor at flow `m` (kg/s) at its present speed.
    fn compressor_efficiency(&self, m: f64) -> f64 {
        let s = math::max(self.omega / self.size.full_speed, MIN_SPEED_FRACTION);
        let phi = m.abs() / (s * self.size.choke_flow);
        let d = (phi - ETA_BEST_FLOW) / (1.0 - ETA_BEST_FLOW);
        ETA_COMPRESSOR * math::max(1.0 - ETA_FALLOFF * d * d, ETA_FLOOR)
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

    /// The turbine as it sits in the exhaust this sample: its flow constants, its wheel's tip speed and
    /// the pulsation of its blades. Call once a sample.
    pub fn turbine_setting(&mut self) -> TurbineSetting {
        let size = self.size;
        let s = self.omega / size.full_speed;
        let shaft_hz = self.omega / (2.0 * PI);
        let mut pulse = 0.0;
        for (h, &(order, rel)) in TURBINE_ORDERS.iter().enumerate() {
            let hz = shaft_hz * order;
            let fade = self.fade(hz);
            let ph = &mut self.turbine_phase[h];
            *ph += hz / self.sample_rate;
            *ph -= ph.floor();
            pulse += rel * fade * math::sin(2.0 * PI * *ph);
        }
        TurbineSetting {
            k_turbine: size.turbine_k,
            k_wastegate: size.wastegate_k * self.wastegate,
            tip_speed: self.omega * size.turbine_radius,
            pulsation: 1.0 + TURBINE_PULSATION * size.noise * s * s * pulse,
            bypass_noise: size.noise,
        }
    }

    /// Advance by `dt`, with the turbine driven by `turbine`. `throttle_flow` is the flow the plenum
    /// drew through the throttle this sample, kg/s; `plenum_p` its pressure, Pa.
    pub fn step(&mut self, dt: f64, turbine: TurbineDrive, throttle_flow: f64, plenum_p: f64) -> TurboOut {
        let size = self.size;
        self.back_pressure = turbine.inlet;
        if turbine.flow > 1e-9 && turbine.isentropic_power > 1e-6 {
            self.turbine_efficiency = turbine.power / turbine.isentropic_power;
            let c0 = math::sqrt((2.0 * turbine.isentropic_power) / turbine.flow);
            self.blade_speed_ratio = (self.omega * size.turbine_radius) / c0;
        } else {
            self.turbine_efficiency = 0.0;
            self.blade_speed_ratio = 0.0;
        }

        // --- Wastegate: opens on boost over its spring ---
        let boost = self.boost();
        let wg_target = clamp((boost - (size.boost_target - WASTEGATE_CRACK)) / WASTEGATE_SPAN, 0.0, 1.0);
        self.wastegate += (dt / WASTEGATE_TAU) * (wg_target - self.wastegate);
        self.wastegate = clamp(self.wastegate, 0.0, 1.0);

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
        self.omega += (dt * (turbine.power - p_compressor - drag)) / (size.inertia * omega);
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

        let sound = self.sound(m_c, vent, p2, t2, turbine.inlet - turbine.outlet);
        self.last_sound = sound;
        TurboOut { charge_p: self.charge_pressure(), charge_t: self.charge_temp(), sound }
    }

    /// 1 for a tone well below the Nyquist frequency, fading to 0 before it, so nothing folds back.
    fn fade(&self, hz: f64) -> f64 {
        let top = math::min(0.45 * self.sample_rate, 18_000.0);
        let start = 0.7 * top;
        if hz >= top {
            0.0
        } else if hz <= start {
            1.0
        } else {
            1.0 - (hz - start) / (top - start)
        }
    }

    /// What the turbo radiates, Pa at 1 m: the compressor inlet's flow, carrying the blades' whine and
    /// a stalled wheel's turbulence; the blow-off valve's jet; the air reversing through the wheel in a
    /// surge; and the wastegate flap rattling on the pulses across the turbine.
    fn sound(&mut self, m_c: f64, vent: f64, p2: f64, t2: f64, across_turbine: f64) -> f64 {
        let size = self.size;
        let level = size.noise;
        let rho = gas::P_AMB / (gas::R * gas::T_AMB);
        let s = self.omega / size.full_speed;
        let shaft_hz = self.omega / (2.0 * PI);

        // --- The blades' modulation of the air drawn in ---
        let mut whine = 0.0;
        let mut speed = shaft_hz;
        for v in 0..self.whine_phase.len() {
            for (h, &(order, rel)) in WHINE_ORDERS.iter().enumerate() {
                let hz = speed * order;
                let fade = self.fade(hz);
                let ph = &mut self.whine_phase[v][h];
                *ph += hz / self.sample_rate;
                *ph -= ph.floor();
                if fade > 0.0 {
                    whine += rel * fade * math::sin(2.0 * PI * *ph);
                }
            }
            speed *= TURBO_DETUNE;
        }
        whine *= (WHINE_DEPTH * s * s) / size.count;

        // --- Stall: left of the surge line the wheel's flow breaks up, around a few shaft orders ---
        let phi = m_c / (math::max(s, MIN_SPEED_FRACTION) * size.choke_flow);
        let surge_line = 2.0 * MG_W;
        let stall = if s > 0.1 { clamp((surge_line - phi) / surge_line, 0.0, 1.0) } else { 0.0 };
        let stall_hz = clamp(shaft_hz, 50.0, 0.2 * self.sample_rate);
        let c_lp = 1.0 - math::exp((-2.0 * PI * 3.0 * stall_hz) / self.sample_rate);
        let c_hp = 1.0 - math::exp((-2.0 * PI * 0.3 * stall_hz) / self.sample_rate);
        self.stall_lp += c_lp * (self.noise.next() - self.stall_lp);
        self.stall_hp += c_hp * (self.stall_lp - self.stall_hp);
        let turbulence = (self.stall_lp - self.stall_hp) * STALL_INTENSITY * stall * s * size.choke_flow;

        // --- The compressor inlet: what the wheel draws from the air, less what a recirculating
        // blow-off valve hands back to it ---
        let recirculated = if matches!(size.blow_off, BlowOff::Recirculating) { vent } else { 0.0 };
        let drawn = m_c - recirculated;
        let inlet = self.inlet.process((drawn * (1.0 + whine) + turbulence) / rho);

        // --- The blow-off valve: its outflow radiating, and its jet's noise ---
        let d_vent = math::sqrt((4.0 * size.blow_off_area * self.blow_off) / PI);
        let u_vent = if vent > 0.0 { jet_velocity(p2, t2, gas::P_AMB) } else { 0.0 };
        let (vent_flow, jet_share) = match size.blow_off {
            BlowOff::Atmospheric => (vent, 1.0),
            _ => (0.0, RECIRCULATING_TRANSMISSION),
        };
        let vented = self.vent.process(vent_flow / rho);
        let jet_pa = if vent > 0.0 { jet_share * lighthill_pa(u_vent, d_vent) } else { 0.0 };
        let jet = self.blow_off_jet.next(&mut self.noise, u_vent, d_vent, jet_pa, self.sample_rate);

        // --- A surge: the air forced backwards through the inducer ---
        let d_inducer = math::sqrt((4.0 * size.inducer_area) / PI);
        let u_reverse = math::max(-m_c, 0.0) / (rho * size.inducer_area);
        let reverse_pa = lighthill_pa(u_reverse, d_inducer);
        let reverse = self.reverse_jet.next(&mut self.noise, u_reverse, d_inducer, reverse_pa, self.sample_rate);

        // --- The wastegate flap, just open, knocked on its seat by each pulse across the turbine ---
        self.across_mean +=
            (1.0 - math::exp((-2.0 * PI * 5.0) / self.sample_rate)) * (across_turbine - self.across_mean);
        let rising = across_turbine - self.across_mean;
        let high = rising > 0.15 * math::max(self.across_mean, 1e3);
        let wg = self.wastegate;
        if high && !self.across_high && wg > 0.0 && wg < WASTEGATE_RATTLE_OPENING {
            let open = 1.0 - wg / WASTEGATE_RATTLE_OPENING;
            self.rattle.trigger(WASTEGATE_RATTLE_PA * open * (rising / 1e4));
        }
        self.across_high = high;
        let hit = self.rattle.next();
        let rattle = self.rattle_modes[0].process(hit) + 0.6 * self.rattle_modes[1].process(hit);

        level * (inlet + vented + jet + reverse + rattle)
    }
}
