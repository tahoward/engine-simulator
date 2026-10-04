//! Turbocharger: a turbine in the exhaust driving a compressor on the same shaft, the charge air
//! between the compressor and the throttle, the wastegate that caps the boost, and the blow-off valve
//! that vents it when the throttle shuts. And the sounds all of that makes.
//!
//! ```text
//!   exhaust junction -> turbine (in the gas dynamics) --- shaft --- compressor <- inlet duct (inertia, radiates)
//!                          |                                             |
//!                      wastegate                                         | charge pipe (gas dynamics)
//!                                                                        v
//!   plenum <- throttle <- throttle body <- cold pipe <- intercooler <- hot pipe
//!                              |
//!                        blow-off valve
//! ```
//!
//! The turbine is solved in the exhaust, at a junction, by `ExhaustSystem`: this sets its flow
//! constants from the wastegate each sample, and is driven by the power it reports. Each compressor
//! blows down a charge pipe, through its intercooler, into the throttle body, solved with the same gas
//! dynamics as the exhaust, so the charge air's pressure waves travel and reflect between the
//! compressor and the throttle. The shaft, the wastegate, the throttle body and the blow-off valve are
//! lumped, one state each, stepped once per audio sample.
//!
//! The compressor is Moore and Greitzer's model: a cubic characteristic, the inertia of the air in the
//! duct drawing into it, and the amplitude of a rotating stall, which grows to the left of the surge
//! line and lowers the pressure the wheel makes. The pressure the wheel makes lags its characteristic
//! by a few revolutions, as the flow through its blades takes that long to settle. Left of the surge
//! line, where the pressure falls as the flow falls, the wheel feeds the charge pipe's lowest resonance
//! rather than damping it: with the throttle shut on boost and nowhere for the air to go, the flow
//! swings back and forth through the wheel, at a frequency the charge pipe sets. That is a surge.
//!
//! What the turbo radiates comes from its flows: the compressor inlet's, carrying the blades' whine
//! and a stalled wheel's turbulence, and the blow-off valve's, with its jet's noise by Lighthill's law.
//!
//! Several turbos all blow into the one throttle body, each down its own charge pipe and through its
//! own intercooler, with its own blow-off valve. Each can be set on its own, or left on the engine's
//! settings. Turbos on the same settings are identical and in parallel, so one lumped shaft and one
//! charge pipe stand for them; a turbo set differently has its own shaft, compressor, charge pipe and
//! wastegate, turned by its own turbine. The sound gives each turbo its own voice, a little apart in
//! speed as two real ones are.

use crate::dsp::{Impact, Noise, Resonator};
use crate::euler_pipe::{DuctEnd, EndState, EulerPipe, EulerPipeOptions, InletKind, OutletKind, ValveState};
use crate::exhaust_graph::TurboSettings;
use crate::exhaust_system::{TurbineResult, TurbineSetting};
use crate::intake::runner_damping;
use crate::math::{self, PI, clamp};
use crate::radiation::{FarField, lighthill_pa};
use crate::spec::{
    BlowOff, EngineSpec, PipeSegment, SegmentKind, SegmentPartial, displacement, gas, gas_energy, gas_enthalpy,
    gas_temperature, make_segment, speed_of_sound,
};
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

/// The compressor's choke flow on a speed line above full speed, as a multiple of the full speed's.
/// Up to full speed a speed line chokes at a flow in proportion to the speed. Above it the lines crowd
/// together on a real map, as the air entering the inducer nears the speed of sound relative to the
/// blades, and here the choke flow levels off at this: an overspeeding wheel makes more pressure at
/// low flow but passes little more air, so a turbo too small for the engine runs out at the top end.
const CHOKE_CEILING: f64 = 1.1;

/// The compressor's characteristic, `f(phi)` for flow `phi` as a fraction of its choke flow at its
/// speed: a Moore-Greitzer cubic, `F0 + H (1 + 1.5 y (1 - J/2) - 0.5 y^3)` with `y = phi / W - 1` and
/// `J` the squared amplitude of a rotating stall. Unstalled, it has its peak pressure rise, 1, at
/// `phi = 2 W`, the surge line, at 44% of the choke flow as on a typical map; to the left of that the
/// pressure falls with falling flow, which is what makes the compression system unstable there. It
/// falls to zero, the choke, at `phi = 1`, and shut off the wheel still makes 89% of its peak. Fully
/// stalled, it makes up to 6% less in between.
const MG_F0: f64 = 0.889;
const MG_H: f64 = 0.0556;
const MG_W: f64 = 0.22;

/// On a real map the surge line bends toward less flow on the lower speed lines, which are flatter and
/// wider than the top one. Here it is at `2 W` of the choke flow at full speed and above, and falls in
/// a straight line with the speed to this share of that at no speed: the flow is stretched, in
/// proportion either side of the surge line, to put it at `2 W`, which leaves no flow and the choke
/// where they are.
const SURGE_LINE_AT_REST: f64 = 0.4;

/// The lower speed lines of a real map are also flat toward no flow, where the top ones fall away to
/// the left of their peak, which is what drives a surge. Here a speed line has the cubic's full hump
/// from `FULL_HUMP_SPEED` of full speed, and less of it in a straight line down to none at
/// `FLAT_SPEED`: the pressure it makes left of its peak is the peak's less that share of the fall.
/// A rotating stall grows toward that share of its amplitude on the full hump, and dies away on a flat
/// speed line.
const FLAT_SPEED: f64 = 0.4;
const FULL_HUMP_SPEED: f64 = 0.8;

/// Rotor revolutions the pressure the wheel makes takes to follow its characteristic: the time the
/// flow through its blades takes to settle. No longer than `COMPRESSOR_LAG_MAX`, s: a wheel turning
/// slowly has little pressure rise to lag.
const COMPRESSOR_LAG_REVS: f64 = 2.0;
const COMPRESSOR_LAG_MAX: f64 = 1e-3;

/// Moore and Greitzer's rotating stall: its squared amplitude `J` grows as
/// `dJ/dt = J (1 - y^2 - J/4) / tau`, toward `4 (1 - y^2)` between no flow and the surge line, and dies
/// away outside it. `tau` is this many rotor revolutions, from their growth rate
/// `3 a H / ((1 + m a) W)` per radian of the rotor with their typical `a = 1/3.5`, `m = 1.75` and
/// `H / W = 0.72`: from a small disturbance a stall cell is fully grown within a few revolutions. `J`
/// never falls below `STALL_SEED`, the disturbances there always are for a stall to grow from.
const STALL_GROWTH_REVS: f64 = 0.4;
const STALL_SEED: f64 = 1e-3;

/// Backwards through the wheel the characteristic goes on falling a little past no flow, to its
/// lowest with the flow `REVERSE_DIP` of `W` reversed, then rises steeply as the spinning blades fight
/// the reversed flow, as measured reverse-flow characteristics do: by `REVERSE_RISE` times `H` for
/// each `W` of reversed flow squared, out to this `y`, three times `W` of the choke flow backwards, and
/// level beyond. So no flow at all is never a resting place for a wheel shut in behind the throttle.
/// The dip goes with the hump.
const REVERSE_Y_MIN: f64 = -4.0;
const REVERSE_DIP: f64 = 0.25;
const REVERSE_RISE: f64 = 2.0;

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
/// The boost controller: the wastegate goes from shut to wide open over this share of the boost target,
/// and a trim that settles the mean boost on the target follows its error over this time, s. The trim is
/// what holds the target, so the band can be wide enough for the loop through the shaft and the charge air
/// to settle rather than hunt from shut to wide open: the higher the boost, the more power the turbine has
/// over what it needs, and the more a band fixed in pascals would swing it.
const WASTEGATE_BAND_SHARE: f64 = 0.25;
const BOOST_TRIM_TAU: f64 = 1.0;
/// Response time of the wastegate actuator and of the blow-off valve, s.
const WASTEGATE_TAU: f64 = 0.04;
/// How long the wastegate actuator's diaphragm, fed through its hose, takes to follow the boost, s: it
/// answers the boost's mean, not the pulses the runners and the plenum's waves ride on it.
const WASTEGATE_SENSE_TAU: f64 = 0.02;
const BLOW_OFF_TAU: f64 = 0.004;

/// Pressure across the throttle, charge side over plenum, at which the blow-off valve starts to open,
/// and the further rise at which it is wide open, Pa.
const BLOW_OFF_CRACK: f64 = 0.3e5;
const BLOW_OFF_SPAN: f64 = 0.15e5;
/// Blow-off valve bore as a fraction of the compressor inducer's.
const BLOW_OFF_AREA_RATIO: f64 = 0.8;
/// Pressure difference across an open blow-off valve below which its flow goes in proportion to it,
/// Pa: see `vent_flow`.
const BLOW_OFF_LINEAR: f64 = 300.0;
/// Time the flow through a blow-off valve takes to follow the pressure across it, s: the inertia of
/// the air in its bore.
const BLOW_OFF_FLOW_TAU: f64 = 1e-3;

/// Charge-air volume, compressor to throttle through the intercooler, as a multiple of total swept
/// volume, and the throttle body's share of it, as a multiple of the swept volume too. The rest is
/// the charge pipes and the intercoolers.
const CHARGE_VOLUME_RATIO: f64 = 2.0;
const THROTTLE_BODY_VOLUME_RATIO: f64 = 0.2;
/// Each charge pipe, laid out as for an intercooler at the front of the car: from the compressor down
/// to the intercooler, the intercooler's core, and from it back up to the throttle body, m. Their
/// length sets how fast a surge cycles. The pipe's bore is this multiple of its inducer's area, and
/// the intercooler holds what is left of the charge volume, but no less than a pipe as long.
const HOT_PIPE_LENGTH: f64 = 1.2;
const INTERCOOLER_LENGTH: f64 = 0.6;
const COLD_PIPE_LENGTH: f64 = 1.5;
const CHARGE_PIPE_AREA_RATIO: f64 = 1.5;
/// Shortest cell the charge pipes are solved on, m. What they carry is the surge, the throttle's
/// pressure waves and the engine's pulses, all well below the 900 Hz or so these resolve, so they are
/// solved on fewer cells than the exhaust.
const CHARGE_PIPE_CELL: f64 = 0.08;
/// The duct the compressor draws through, from the air filter through the wheel's passages, m: its
/// air's inertia, at the inducer's area.
const COMPRESSOR_DUCT_LENGTH: f64 = 0.6;
/// Loss coefficient of the flow through a compressor too slow to do any work, windmilling.
const WINDMILL_LOSS: f64 = 1.0;

/// Speeds below which the compressor is taken to be this fraction of full speed, for its
/// characteristic, so it does not divide by zero; and the turbine too, so a stopped wheel still takes
/// the torque the gas puts on it and starts to turn.
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

/// How much of a recirculating blow-off valve's jet noise gets out through the intake's ducting.
const RECIRCULATING_TRANSMISSION: f64 = 0.1;

/// The turbulence a compressor whose flow has fully broken down sheds into its inlet, as a share of the
/// flow through it, as a wastegate's jet sheds: what the flutter is made of. It goes as the breakdown.
const STALL_INTENSITY: f64 = 0.2;

/// Wastegate flap rattle at 1 m, Pa, per 10 kPa of pulse across it, and how far open it can be and
/// still rattle.
const WASTEGATE_RATTLE_PA: f64 = 2.0;
const WASTEGATE_RATTLE_OPENING: f64 = 0.45;

/// Everything about a set of identical turbos that follows from the spec and their settings.
#[derive(Clone, Copy, Debug)]
struct RotorSizing {
    count: f64,
    /// Choke flow of all their compressors together at full speed, kg/s.
    choke_flow: f64,
    /// Peak pressure rise at full speed, Pa.
    peak_rise: f64,
    /// Full shaft speed, rad/s, and the turbine wheel's tip radius, m.
    full_speed: f64,
    turbine_radius: f64,
    /// All their rotors together, kg*m^2.
    inertia: f64,
    friction: f64,
    /// Stodola flow constant of each turbine and of its wastegate wide open, kg*sqrt(K)/(s*Pa).
    turbine_k: f64,
    wastegate_k: f64,
    /// Inducer area, all together, m^2.
    inducer_area: f64,
    /// Their compressor ducts' area over length, all together, m.
    duct_a_over_l: f64,
    /// Bore of their charge pipes, all together, and of their intercoolers, m.
    pipe_dia: f64,
    intercooler_dia: f64,
    boost_target: f64,
    /// Their intercoolers' effectiveness, and their blow-off valves and those valves' bore, all
    /// together, m^2.
    intercooler: f64,
    blow_off: BlowOff,
    blow_off_area: f64,
}

/// Everything the turbos share that follows from the spec: the throttle body they all blow into.
#[derive(Clone, Copy, Debug)]
struct ChargeSizing {
    /// The throttle body's volume, m^3.
    volume: f64,
    /// Bore of all the blow-off valves together, and inducer area of all the compressors, m^2.
    blow_off_area: f64,
    inducer_area: f64,
    noise: f64,
}

/// `members` identical turbos of `count` in all, which share the engine's airflow when left to size
/// themselves.
fn rotor_sizing(spec: &EngineSpec, settings: TurboSettings, members: usize, count: usize) -> RotorSizing {
    let n = math::max(members as f64, 1.0);
    let count = math::max(count as f64, 1.0);
    let boost_target = math::max(settings.boost_target, 0.05e5);
    let swept = displacement(spec) * math::max(spec.cylinders as f64, 1.0);

    let choke_flow = if settings.turbo_size > 0.0 {
        settings.turbo_size
    } else {
        let rho = (gas::P_AMB + boost_target) / (gas::R * SIZING_CHARGE_T);
        FLOW_HEADROOM * rho * swept * (0.8 * spec.rev_limit / 120.0) * SIZING_VE / count
    };
    let peak_rise = PRESSURE_HEADROOM * boost_target;

    // The tip speed that does the work of the peak pressure rise, and the wheel that passes the choke
    // flow at it.
    let pr = (gas::P_AMB + peak_rise) / gas::P_AMB;
    let work = CP_AIR * gas::T_AMB * (math::pow(pr, (GAMMA_AIR - 1.0) / GAMMA_AIR) - 1.0) / ETA_COMPRESSOR;
    let tip = math::sqrt(work / WORK_COEFFICIENT);
    let rho_amb = gas::P_AMB / (gas::R * gas::T_AMB);
    let inducer_area = choke_flow / (rho_amb * INDUCER_VELOCITY_RATIO * tip);
    let wheel = math::sqrt((4.0 * inducer_area) / PI) / INDUCER_RATIO;
    let full_speed = (2.0 * tip) / wheel;
    let inertia = INERTIA_AT_50MM * math::pow(wheel / 0.05, 5.0);
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

    RotorSizing {
        count: n,
        choke_flow: choke_flow * n,
        peak_rise,
        full_speed,
        turbine_radius,
        inertia: inertia * n,
        friction: friction * n,
        turbine_k,
        wastegate_k: WASTEGATE_CAPACITY * turbine_k,
        inducer_area: inducer_area * n,
        duct_a_over_l: (inducer_area * n) / COMPRESSOR_DUCT_LENGTH,
        pipe_dia: math::sqrt((4.0 * CHARGE_PIPE_AREA_RATIO * inducer_area * n) / PI),
        intercooler_dia: 0.0,
        boost_target,
        intercooler: clamp(settings.intercooler, 0.0, 1.0),
        blow_off: settings.blow_off,
        blow_off_area: BLOW_OFF_AREA_RATIO * inducer_area * n,
    }
}

/// Turbos on the same settings, as `settings` has them one for each turbo: each set's settings, and
/// which turbos are in it.
fn groups(settings: &[TurboSettings]) -> Vec<(TurboSettings, Vec<usize>)> {
    let mut out: Vec<(TurboSettings, Vec<usize>)> = Vec::new();
    for (i, &s) in settings.iter().enumerate() {
        match out.iter_mut().find(|(g, _)| *g == s) {
            Some((_, members)) => members.push(i),
            None => out.push((s, vec![i])),
        }
    }
    out
}

/// Every set of identical turbos, as `groups` has them, and the throttle body they all share. Each
/// set's intercoolers hold as much of the charge volume as their inducers are a share of them all.
fn sizing(spec: &EngineSpec, groups: &[(TurboSettings, Vec<usize>)]) -> (ChargeSizing, Vec<RotorSizing>) {
    let count = groups.iter().map(|(_, m)| m.len()).sum();
    let mut rotors: Vec<RotorSizing> =
        groups.iter().map(|(s, members)| rotor_sizing(spec, *s, members.len(), count)).collect();
    let inducer_area: f64 = rotors.iter().map(|r| r.inducer_area).sum();
    let swept = displacement(spec) * math::max(spec.cylinders as f64, 1.0);
    let volume = math::max(THROTTLE_BODY_VOLUME_RATIO * swept, 1e-5);
    let piped = math::max((CHARGE_VOLUME_RATIO - THROTTLE_BODY_VOLUME_RATIO) * swept, 1e-4);
    for r in rotors.iter_mut() {
        let share = piped * (r.inducer_area / math::max(inducer_area, 1e-12));
        let pipe_area = (PI * r.pipe_dia * r.pipe_dia) / 4.0;
        let core = math::max(share - pipe_area * (HOT_PIPE_LENGTH + COLD_PIPE_LENGTH), pipe_area * INTERCOOLER_LENGTH);
        r.intercooler_dia = math::sqrt((4.0 * core) / (PI * INTERCOOLER_LENGTH));
    }
    let charge = ChargeSizing {
        volume,
        blow_off_area: BLOW_OFF_AREA_RATIO * inducer_area,
        inducer_area,
        noise: math::max(spec.turbo_noise, 0.0),
    };
    (charge, rotors)
}

/// The charge pipe from a set's compressors to the throttle body, compressor end first: the hot pipe,
/// the intercooler and the cold pipe.
fn charge_segments(pipe_dia: f64, intercooler_dia: f64) -> Vec<PipeSegment> {
    let pipe = |length: f64| {
        make_segment(SegmentPartial {
            kind: Some(SegmentKind::Pipe),
            length: Some(length),
            d_in: Some(pipe_dia),
            ..Default::default()
        })
    };
    vec![
        pipe(HOT_PIPE_LENGTH),
        make_segment(SegmentPartial {
            kind: Some(SegmentKind::Chamber),
            length: Some(INTERCOOLER_LENGTH),
            d_in: Some(pipe_dia),
            d_out: Some(intercooler_dia),
            ..Default::default()
        }),
        pipe(COLD_PIPE_LENGTH),
    ]
}

/// A set's charge pipe, its gas still at the atmosphere's: the compressor meets its first end, and its
/// last opens into the throttle body.
fn charge_pipe(size: &RotorSizing, sample_rate: f64, opts: &EulerPipeOptions) -> EulerPipe {
    let segments = charge_segments(size.pipe_dia, size.intercooler_dia);
    let length: f64 = segments.iter().map(|s| s.length).sum();
    let c = speed_of_sound(gas::T_AMB, GAMMA_AIR);
    let pipe_opts = EulerPipeOptions {
        inlet_kind: Some(InletKind::Junction),
        outlet_kind: Some(OutletKind::Mouth),
        heat_transfer: Some(false),
        initial_port_temp: Some(gas::T_AMB),
        // The throttle body's air taken in at its own pressure, as the intake runners take the plenum's.
        nozzle_inflow: Some(false),
        linear_damping: Some(runner_damping(size.pipe_dia / 2.0, c / (4.0 * length))),
        port: None,
        inherit_wall: None,
        // The exhaust's material is the exhaust's alone.
        material: None,
        cell_size: Some(math::max(opts.cell_size.unwrap_or(0.0), CHARGE_PIPE_CELL)),
        ..opts.clone()
    };
    let mut pipe = EulerPipe::new(&segments, sample_rate, gas::T_AMB, &pipe_opts);
    // The intercooler's core is a bank of narrow tubes, not an open can: nothing resonates across it.
    pipe.cross_modes = None;
    pipe
}

/// The gauge pressure, Pa, at which the face at a charge pipe's compressor end passes `m`, kg/s, into
/// it from gas at `t`, K: the acoustic estimate, then one Newton step on the flow it actually passes.
fn face_gauge(pipe: &mut EulerPipe, st: &EndState, m: f64, t: f64) -> f64 {
    let area = pipe.face_area(0);
    let rho = math::max(st.rho, 1e-3);
    let mut g = 2.0 * st.toward + (st.rho_c * m) / (rho * area);
    let passed = pipe.probe_junction(DuctEnd::Inlet, g, t, st);
    g += ((m - passed) * st.c) / area;
    g
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

/// What the turbo hands the engine each sample.
#[derive(Clone, Copy, Debug)]
pub struct TurboOut {
    /// Pressure and temperature of the charge air the throttle draws from, Pa and K.
    pub charge_p: f64,
    pub charge_t: f64,
    /// What the turbo radiates this sample, Pa at 1 m.
    pub sound: f64,
}

/// Flow out through an open blow-off valve of effective `area`, m^2, from the throttle body at `p2`, Pa,
/// and `t2`, K, kg/s: through it as an orifice, and within `BLOW_OFF_LINEAR` of the atmosphere's
/// pressure in proportion to the difference, as the air in its bore cannot follow the orifice law's
/// infinitely steep start from no difference at all. Nothing comes in through it: with the throttle
/// body below the atmosphere, the pressure across the piston holds it to its seat.
fn vent_flow(area: f64, p2: f64, t2: f64) -> f64 {
    let dp = p2 - gas::P_AMB;
    if dp <= 0.0 {
        0.0
    } else if dp >= BLOW_OFF_LINEAR {
        orifice_mass_flow(area, 0.7, p2, t2, gas::P_AMB, GAMMA_AIR)
    } else {
        orifice_mass_flow(area, 0.7, gas::P_AMB + BLOW_OFF_LINEAR, t2, gas::P_AMB, GAMMA_AIR) * (dp / BLOW_OFF_LINEAR)
    }
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

/// A set of identical turbos, in parallel on the same settings, so one lumped shaft, compressor duct,
/// charge pipe and wastegate stands for them all; the sound gives each its own voice.
struct Rotor {
    size: RotorSizing,
    /// Which turbos, in the order of the exhaust's turbines.
    members: Vec<usize>,
    /// Shaft speed, rad/s.
    omega: f64,
    /// Flow through the compressor duct, kg/s, positive toward the engine: as it is now, and its mean
    /// over the last sample.
    duct_flow: f64,
    compressor_flow: f64,
    /// The pressure rise the wheel makes, lagging its characteristic, Pa; and the squared amplitude of
    /// its rotating stall, `J`.
    rise: f64,
    stall: f64,
    /// The surge line on the present speed line, as a share of its choke flow: see `SURGE_LINE_AT_REST`.
    surge_line: f64,
    /// How much of the cubic's hump the present speed line has, 0 flat to 1: see `FLAT_SPEED`.
    hump: f64,
    /// Temperature of the air the compressor delivers into its charge pipe, past the intercooler, K.
    delivery_t: f64,
    /// The charge pipe from the compressor to the throttle body, and what flowed out of it into the
    /// throttle body over the last sample: mass, kg/s, and enthalpy, W.
    pipe: EulerPipe,
    delivered: f64,
    delivered_h: f64,
    wastegate: f64,
    /// The boost controller's trim on the wastegates' opening, 0..1.
    boost_trim: f64,
    /// Their blow-off valves' opening, 0..1, and the flow out of them last sample, kg/s.
    blow_off: f64,
    vent: f64,
    /// What drove the turbines last sample, all together, with their mean inlet and outlet pressures.
    drive: TurbineResult,

    // Sound
    /// Each turbo's whine harmonics.
    whine_phase: Vec<[f64; 5]>,
    turbine_phase: [f64; 2],
    reverse_jet: JetNoise,
    blow_off_jet: JetNoise,
    stall_lp: [f64; 2],
    stall_hp: f64,
    /// The pressure drop across the turbine, smoothed, Pa, and whether it is above that now: the
    /// pulses the wastegate flap rattles on.
    across_mean: f64,
    across_high: bool,
    rattle: Impact,
    rattle_modes: [Resonator; 2],
}

impl Rotor {
    fn new(size: RotorSizing, members: Vec<usize>, sample_rate: f64, opts: &EulerPipeOptions) -> Rotor {
        Rotor {
            size,
            omega: 0.03 * size.full_speed,
            duct_flow: 0.0,
            compressor_flow: 0.0,
            rise: 0.0,
            stall: STALL_SEED,
            surge_line: 2.0 * MG_W,
            hump: 1.0,
            delivery_t: gas::T_AMB,
            pipe: charge_pipe(&size, sample_rate, opts),
            delivered: 0.0,
            delivered_h: 0.0,
            wastegate: 0.0,
            boost_trim: 0.0,
            blow_off: 0.0,
            vent: 0.0,
            drive: TurbineResult { inlet: gas::P_AMB, outlet: gas::P_AMB, ..TurbineResult::default() },
            whine_phase: vec![[0.0; 5]; members.len()],
            turbine_phase: [0.0; 2],
            reverse_jet: JetNoise::new(),
            blow_off_jet: JetNoise::new(),
            stall_lp: [0.0; 2],
            stall_hp: 0.0,
            across_mean: 0.0,
            across_high: false,
            rattle: Impact::new(0.0002, sample_rate),
            rattle_modes: [Resonator::new(1850.0, 12.0, sample_rate), Resonator::new(3300.0, 15.0, sample_rate)],
            members,
        }
    }

    /// Choke flow of its compressors on the speed line at `s` of full speed, kg/s.
    fn choke_at(&self, s: f64) -> f64 {
        let s = math::max(s, MIN_SPEED_FRACTION);
        let over = CHOKE_CEILING - 1.0;
        let line = if s <= 1.0 { s } else { 1.0 + over * math::tanh((s - 1.0) / over) };
        line * self.size.choke_flow
    }

    /// Isentropic efficiency of the compressor at flow `m` (kg/s) at its present speed.
    fn compressor_efficiency(&self, m: f64) -> f64 {
        let phi = m.abs() / self.choke_at(self.omega / self.size.full_speed);
        let d = (phi - ETA_BEST_FLOW) / (1.0 - ETA_BEST_FLOW);
        ETA_COMPRESSOR * math::max(1.0 - ETA_FALLOFF * d * d, ETA_FLOOR)
    }

    /// The pressure rise the wheel's blades settle to at flow `m`, kg/s, in a rotating stall of squared
    /// amplitude `j`, Pa: its characteristic.
    fn characteristic(&self, m: f64, j: f64) -> f64 {
        let s = self.omega / self.size.full_speed;
        let y = math::max(self.y_of(m), REVERSE_Y_MIN);
        let cubic = |y: f64| MG_F0 + MG_H * (1.0 + 1.5 * y * (1.0 - 0.5 * j) - 0.5 * y * y * y);
        let (peak, g) = (cubic(1.0), self.hump);
        let shape = if y > 1.0 {
            cubic(y)
        } else if y >= -1.0 {
            peak - g * (peak - cubic(y))
        } else {
            let d = y + 1.0;
            peak - g * (peak - cubic(-1.0)) + MG_H * REVERSE_RISE * (d * d + 2.0 * REVERSE_DIP * g * d)
        };
        self.size.peak_rise * s * s * shape
    }

    /// The flow `m`, kg/s, as the cubic takes it, `phi / W - 1`: forwards with `phi` the flow
    /// coefficient, backwards the plain share of the choke flow on the present speed line.
    fn y_of(&self, m: f64) -> f64 {
        let phi = if m > 0.0 { self.flow_coefficient(m) } else { m / self.choke_at(self.omega / self.size.full_speed) };
        phi / MG_W - 1.0
    }

    /// The flow `m`, kg/s, 0 or more, as the characteristic takes it: as a share of the choke flow on the
    /// present speed line, bent so the surge line there falls at `2 W`.
    fn flow_coefficient(&self, m: f64) -> f64 {
        let phi = math::min(m / self.choke_at(self.omega / self.size.full_speed), 5.0);
        let (surge, at) = (self.surge_line, 2.0 * MG_W);
        if phi < surge { phi * (at / surge) } else { at + (phi - surge) * ((1.0 - at) / (1.0 - surge)) }
    }

    /// Set `surge_line` and `hump` for the present shaft speed.
    fn set_speed_line(&mut self) {
        let s = clamp(self.omega / self.size.full_speed, 0.0, 1.0);
        self.surge_line = 2.0 * MG_W * (SURGE_LINE_AT_REST + (1.0 - SURGE_LINE_AT_REST) * s);
        self.hump = clamp((s - FLAT_SPEED) / (FULL_HUMP_SPEED - FLAT_SPEED), 0.0, 1.0);
    }

    /// The loss of air forced through the wheel's passages at flow `m`, kg/s, where the wheel does no
    /// work on it, Pa, with the sign of the flow.
    fn passage_loss(&self, m: f64) -> f64 {
        let rho = gas::P_AMB / (gas::R * gas::T_AMB);
        let a = self.size.inducer_area;
        (WINDMILL_LOSS * m * m.abs()) / (2.0 * rho * a * a)
    }

    /// Pressure rise across the compressor, Pa, at flow `m` (kg/s), with the wheel making `rise`. Past
    /// its choke the wheel does no more work and is only a restriction; backwards, the loss of forcing
    /// air the wrong way through its passages resists the reversed flow.
    fn net_rise(&self, m: f64, rise: f64) -> f64 {
        let loss = self.passage_loss(m);
        if m <= 0.0 { rise - loss } else { math::max(rise, -loss) }
    }

    /// The pressure rise the wheel does work for at flow `m`, kg/s, Pa: its characteristic unstalled,
    /// as a stall spends the same work for less pressure. Past its choke it does none.
    fn wheel_rise(&self, m: f64) -> f64 {
        math::max(self.characteristic(m, 0.0), 0.0)
    }

    /// Advance the compressor by `h`, s, its charge pipe having taken `m`, kg/s, at `p_face`, Pa: the
    /// wheel's pressure rise following its characteristic, the stall growing or dying away, and the
    /// duct's air accelerated by the rise against the pipe.
    fn advance_compressor(&mut self, h: f64, m: f64, p_face: f64) {
        let full = self.size.full_speed;
        let revs = math::max(self.omega, MIN_SPEED_FRACTION * full) / (2.0 * PI);
        let tau = math::min(COMPRESSOR_LAG_REVS / revs, COMPRESSOR_LAG_MAX);
        let settled = self.characteristic(m, self.stall);
        let target = if m > 0.0 { math::max(settled, -self.passage_loss(m)) } else { settled };
        self.rise += (1.0 - math::exp(-h / tau)) * (target - self.rise);

        let y = self.y_of(m);
        // On a flatter speed line a stall grows more slowly, and to less: to nothing on a flat one.
        let room = 1.0 - y * y;
        let drive = if room > 0.0 { self.hump * room } else { room } - 0.25 * self.stall;
        let growth = drive * (revs / STALL_GROWTH_REVS);
        self.stall = clamp(self.stall * math::exp(math::max(growth * h, -50.0)), STALL_SEED, 4.0);

        self.duct_flow = m + h * self.size.duct_a_over_l * (gas::P_AMB + self.net_rise(m, self.rise) - p_face);
    }

    /// The stall's amplitude, 0 unstalled to 1 fully stalled.
    fn stall_amplitude(&self) -> f64 {
        0.5 * math::sqrt(math::max(self.stall - STALL_SEED, 0.0))
    }

    /// How far the flow through the wheel has broken down, 0 to 1: its rotating stall, or the air
    /// forced backwards through it, which leaves its passages as fully separated as `W` of its choke
    /// flow backwards does.
    fn breakdown(&self) -> f64 {
        let s = self.omega / self.size.full_speed;
        let reverse = -self.compressor_flow / (MG_W * self.choke_at(s));
        clamp(math::max(self.stall_amplitude(), reverse), 0.0, 1.0)
    }
}

pub struct Turbo {
    sample_rate: f64,
    charge: ChargeSizing,
    rotors: Vec<Rotor>,
    /// Which of `rotors` each turbo is in.
    rotor_of: Vec<usize>,
    /// Each turbine's setting this sample, handed to the exhaust.
    settings: Vec<TurbineSetting>,
    /// The throttle body's gas: mass, kg, and sensible internal energy, J.
    mass: f64,
    energy: f64,
    /// Seconds since a compressor's flow last ran backwards.
    since_reverse: f64,
    /// The boost the wastegate actuators feel, gauge, Pa.
    sensed_boost: f64,

    // Sound
    noise: Noise,
    /// The compressor inlets and the atmospheric blow-off valve's outlet, radiating as the tailpipe
    /// does.
    inlet: FarField,
    vent: FarField,
    last_sound: f64,
}

impl Turbo {
    /// One turbo for each of `settings`, in the order the exhaust has their turbines in. Their charge
    /// pipes are solved on `opts`.
    pub fn new(spec: &EngineSpec, settings: &[TurboSettings], sample_rate: f64, opts: &EulerPipeOptions) -> Turbo {
        let groups = groups(settings);
        let (charge, sizes) = sizing(spec, &groups);
        let mass = (gas::P_AMB * charge.volume) / (gas::R * gas::T_AMB);
        let c = math::sqrt(GAMMA_AIR * gas::R * gas::T_AMB);
        let inlet_radius = math::max(math::sqrt(charge.inducer_area / PI), 0.01);
        let vent_radius = math::max(math::sqrt(charge.blow_off_area / PI), 0.005);
        Turbo {
            sample_rate,
            rotor_of: rotor_of(&groups, settings.len()),
            rotors: sizes
                .into_iter()
                .zip(groups)
                .map(|(size, (_, m))| Rotor::new(size, m, sample_rate, opts))
                .collect(),
            settings: Vec::with_capacity(settings.len()),
            mass,
            energy: mass * gas_energy(gas::T_AMB),
            since_reverse: f64::INFINITY,
            sensed_boost: 0.0,
            noise: Noise::new(0x7ab0_c3d1 as f64),
            inlet: FarField::new(sample_rate, (2.0 * c) / inlet_radius),
            vent: FarField::new(sample_rate, (2.0 * c) / vent_radius),
            last_sound: 0.0,
            charge,
        }
    }

    /// Resize for `spec` and one turbo for each of `settings`, their charge pipes solved on `opts`,
    /// keeping the charge air and each turbo's shaft speed, compressor and wastegate: a turbo added
    /// starts as a new one does.
    pub fn configure(&mut self, spec: &EngineSpec, settings: &[TurboSettings], opts: &EulerPipeOptions) {
        let groups = groups(settings);
        let (charge, sizes) = sizing(spec, &groups);
        let scale = charge.volume / self.charge.volume;
        self.mass *= scale;
        self.energy *= scale;
        let old = std::mem::take(&mut self.rotors);
        for (size, (_, members)) in sizes.into_iter().zip(groups.iter()) {
            let n = members.len() as f64;
            let prev = self.rotor_of.get(members[0]).map(|&r| &old[r]);
            let mut r = Rotor::new(size, members.clone(), self.sample_rate, opts);
            if let Some(p) = prev {
                r.omega = clamp(p.omega, 0.0, 1.2 * size.full_speed);
                r.duct_flow = p.duct_flow * (n / p.size.count);
                r.compressor_flow = p.compressor_flow * (n / p.size.count);
                r.rise = p.rise;
                r.stall = p.stall;
                r.delivery_t = p.delivery_t;
                r.pipe.resample_from(&p.pipe);
                r.wastegate = p.wastegate;
                r.boost_trim = p.boost_trim;
                r.blow_off = p.blow_off;
                r.drive = p.drive;
                for (v, phase) in r.whine_phase.iter_mut().enumerate() {
                    *phase = p.whine_phase.get(v).copied().unwrap_or([0.0; 5]);
                }
                r.turbine_phase = p.turbine_phase;
            }
            self.rotors.push(r);
        }
        self.rotor_of = rotor_of(&groups, settings.len());
        self.charge = charge;
    }

    /// How many turbos there are.
    pub fn count(&self) -> usize {
        self.rotor_of.len()
    }

    pub fn charge_temp(&self) -> f64 {
        clamp(gas_temperature(self.energy / math::max(self.mass, 1e-9)), 150.0, 1000.0)
    }

    pub fn charge_pressure(&self) -> f64 {
        (math::max(self.mass, 1e-9) * gas::R * self.charge_temp()) / self.charge.volume
    }

    /// Boost, gauge, Pa.
    pub fn boost(&self) -> f64 {
        self.charge_pressure() - gas::P_AMB
    }

    /// The mean of `f` over the turbos.
    fn mean(&self, f: impl Fn(&Rotor) -> f64) -> f64 {
        self.rotors.iter().map(|r| r.size.count * f(r)).sum::<f64>() / math::max(self.count() as f64, 1.0)
    }

    /// Mean speed of the turbos' shafts, rev/min.
    pub fn shaft_rpm(&self) -> f64 {
        self.mean(|r| r.omega) * 60.0 / (2.0 * PI)
    }

    /// Speed of turbo `i`'s shaft, rev/min.
    pub fn shaft_rpm_of(&self, i: usize) -> f64 {
        (self.rotors[self.rotor_of[i]].omega * 60.0) / (2.0 * PI)
    }

    /// Frequency of the compressor blade-pass tone at the mean shaft speed, Hz.
    pub fn blade_pass_hz(&self) -> f64 {
        (self.mean(|r| r.omega) / (2.0 * PI)) * COMPRESSOR_BLADES
    }

    /// Flow through all the compressors together, kg/s.
    pub fn compressor_flow(&self) -> f64 {
        self.rotors.iter().map(|r| r.compressor_flow).sum()
    }

    /// Mean opening of the wastegates, 0..1.
    pub fn wastegate(&self) -> f64 {
        self.mean(|r| r.wastegate)
    }

    /// Opening of turbo `i`'s wastegate, 0..1.
    pub fn wastegate_of(&self, i: usize) -> f64 {
        self.rotors[self.rotor_of[i]].wastegate
    }

    /// Mean opening of the blow-off valves, 0..1.
    pub fn blow_off(&self) -> f64 {
        self.mean(|r| r.blow_off)
    }

    /// Opening of turbo `i`'s blow-off valve, 0..1.
    pub fn blow_off_of(&self, i: usize) -> f64 {
        self.rotors[self.rotor_of[i]].blow_off
    }

    /// Mean amplitude of the compressors' rotating stall, 0 unstalled to 1 fully stalled.
    pub fn stall(&self) -> f64 {
        self.mean(|r| r.stall_amplitude())
    }

    /// Whether a compressor has run backwards in the last tenth of a second.
    pub fn surging(&self) -> bool {
        self.since_reverse < 0.1
    }

    /// What the turbos themselves radiated last sample, Pa at 1 m, before the listener: their own
    /// sounds alone, without the exhaust's.
    pub fn last_sound(&self) -> f64 {
        self.last_sound
    }

    /// Mean pressure at the turbine inlets, Pa.
    pub fn back_pressure(&self) -> f64 {
        self.mean(|r| r.drive.inlet)
    }

    /// The turbines' isentropic efficiency last sample, averaged over the flow through them all: below
    /// nothing when they were braking their shafts.
    pub fn turbine_efficiency(&self) -> f64 {
        let flow: f64 = self.rotors.iter().map(|r| r.drive.flow).sum();
        let isentropic: f64 = self.rotors.iter().map(|r| r.drive.isentropic_power).sum();
        if flow > 1e-9 && isentropic > 1e-6 {
            self.rotors.iter().map(|r| r.drive.power).sum::<f64>() / isentropic
        } else {
            0.0
        }
    }

    /// The turbines' blade speed ratio last sample, their tip speed, averaged over the flow through
    /// them, over the spouting velocity of the mean isentropic drop across them: 0.7 is where their map
    /// is best.
    pub fn blade_speed_ratio(&self) -> f64 {
        let flow: f64 = self.rotors.iter().map(|r| r.drive.flow).sum();
        let isentropic: f64 = self.rotors.iter().map(|r| r.drive.isentropic_power).sum();
        if flow > 1e-9 && isentropic > 1e-6 {
            let tip = self.rotors.iter().map(|r| r.drive.flow * r.omega * r.size.turbine_radius).sum::<f64>() / flow;
            tip / math::sqrt((2.0 * isentropic) / flow)
        } else {
            0.0
        }
    }

    /// The turbines as they sit in the exhaust this sample, in the order of the turbos: their flow
    /// constants, their wheels' tip speeds and the pulsation of their blades. Call once a sample.
    pub fn turbine_settings(&mut self) -> &[TurbineSetting] {
        let noise = self.charge.noise;
        let sample_rate = self.sample_rate;
        self.settings.clear();
        self.settings.resize(self.rotor_of.len(), TurbineSetting::default());
        for r in self.rotors.iter_mut() {
            let size = r.size;
            let s = r.omega / size.full_speed;
            let shaft_hz = r.omega / (2.0 * PI);
            let mut pulse = 0.0;
            for (h, &(order, rel)) in TURBINE_ORDERS.iter().enumerate() {
                let hz = shaft_hz * order;
                let fade = fade(hz, sample_rate);
                let ph = &mut r.turbine_phase[h];
                *ph += hz / sample_rate;
                *ph -= ph.floor();
                pulse += rel * fade * math::sin(2.0 * PI * *ph);
            }
            let setting = TurbineSetting {
                k_turbine: size.turbine_k,
                k_wastegate: size.wastegate_k * r.wastegate,
                tip_speed: math::max(r.omega, MIN_SPEED_FRACTION * size.full_speed) * size.turbine_radius,
                pulsation: 1.0 + TURBINE_PULSATION * noise * s * s * pulse,
                bypass_noise: noise,
            };
            for &m in &r.members {
                self.settings[m] = setting;
            }
        }
        &self.settings
    }

    /// Advance by `dt`, each turbine driven as `turbines` has it, in the order of the turbos.
    /// `throttle_flow` is the flow the plenum drew through the throttle this sample, kg/s; `plenum_p`
    /// its pressure, Pa.
    pub fn step(&mut self, dt: f64, turbines: &[TurbineResult], throttle_flow: f64, plenum_p: f64) -> TurboOut {
        let charge = self.charge;
        self.sensed_boost += (1.0 - math::exp(-dt / WASTEGATE_SENSE_TAU)) * (self.boost() - self.sensed_boost);
        let boost = self.sensed_boost;
        let p2 = self.charge_pressure();
        let t2 = self.charge_temp();
        let h2 = gas_enthalpy(t2);
        let mut m_in = 0.0;
        let mut h_in = 0.0;
        let mut reversed = false;

        for r in self.rotors.iter_mut() {
            let size = r.size;
            // Its turbines all together, and their mean pressures.
            let mut d = TurbineResult::default();
            let mut fed = 0.0;
            for &m in &r.members {
                if let Some(t) = turbines.get(m) {
                    d.power += t.power;
                    d.isentropic_power += t.isentropic_power;
                    d.flow += t.flow;
                    d.bypass_flow += t.bypass_flow;
                    d.inlet += t.inlet;
                    d.outlet += t.outlet;
                    fed += 1.0;
                }
            }
            if fed > 0.0 {
                d.inlet /= fed;
                d.outlet /= fed;
            } else {
                d.inlet = gas::P_AMB;
                d.outlet = gas::P_AMB;
            }
            r.drive = d;
            let turbine = d;

            // --- Wastegate: opened by the boost controller on the boost its actuator feels, in proportion to
            // how far that is over the target, with a trim that brings the mean onto it ---
            let error = (boost - size.boost_target) / (WASTEGATE_BAND_SHARE * size.boost_target);
            r.boost_trim = clamp(r.boost_trim + (dt / BOOST_TRIM_TAU) * error, 0.0, 1.0);
            let wg_target = clamp(r.boost_trim + error, 0.0, 1.0);
            r.wastegate += (dt / WASTEGATE_TAU) * (wg_target - r.wastegate);
            r.wastegate = clamp(r.wastegate, 0.0, 1.0);

            // --- Compressor and its charge pipe: each substep, the face of the pipe passes what the duct
            // delivers, and the wheel and the duct's air answer the pressure there ---
            r.set_speed_line();
            let rho2 = p2 / (gas::R * t2);
            r.pipe.set_reservoir(p2, rho2, math::sqrt((gas::GAMMA_EXH * p2) / rho2));
            let substeps = r.pipe.substeps_for(dt);
            let h = dt / substeps as f64;
            let (mut flow, mut delivered, mut delivered_h) = (0.0, 0.0, 0.0);
            let still = ValveState::default();
            for _ in 0..substeps {
                let pipe = &mut r.pipe;
                pipe.begin_step(h);
                let st = pipe.end_state(DuctEnd::Inlet);
                let g = face_gauge(pipe, &st, r.duct_flow, r.delivery_t);
                let m = pipe.apply_junction(DuctEnd::Inlet, g, r.delivery_t, &st);
                pipe.apply_own_boundaries(h);
                let out = pipe.mouth_mass_flow;
                delivered += out;
                delivered_h += out * if out >= 0.0 { gas_enthalpy(pipe.read_mouth().1) } else { h2 };
                pipe.set_end_step(h, 0.0);
                pipe.end_step_set(&still);
                pipe.after_step(h);
                flow += m;
                r.advance_compressor(h, m, gas::P_AMB + g);
            }
            let inv = 1.0 / substeps as f64;
            if r.pipe.recover_if_broken() {
                (flow, delivered, delivered_h) = (0.0, 0.0, 0.0);
                r.duct_flow = 0.0;
            }
            r.compressor_flow = flow * inv;
            r.delivered = delivered * inv;
            r.delivered_h = delivered_h * inv;
            let m_c = r.compressor_flow;
            let pr = (gas::P_AMB + r.wheel_rise(m_c)) / gas::P_AMB;
            let heating = (math::pow(pr, (GAMMA_AIR - 1.0) / GAMMA_AIR) - 1.0) / r.compressor_efficiency(m_c);
            let p_compressor = m_c.abs() * CP_AIR * gas::T_AMB * heating;
            let t_out = gas::T_AMB * (1.0 + heating);
            r.delivery_t = t_out - size.intercooler * (t_out - gas::T_AMB);
            reversed |= m_c < 0.0;

            // --- Shaft ---
            let drag = size.friction * r.omega * r.omega;
            let omega = math::max(r.omega, MIN_SPEED_FRACTION * size.full_speed);
            r.omega += (dt * (turbine.power - p_compressor - drag)) / (size.inertia * omega);
            r.omega = clamp(r.omega, 0.0, 2.0 * size.full_speed);

            // --- What its charge pipe delivers into the throttle body ---
            m_in += r.delivered;
            h_in += r.delivered_h;
        }
        if reversed {
            self.since_reverse = 0.0;
        } else {
            self.since_reverse += dt;
        }

        // --- Blow-off valves: each opens on the pressure across a shut throttle ---
        let across = p2 - plenum_p;
        let (mut vent, mut vent_h) = (0.0, 0.0);
        for r in self.rotors.iter_mut() {
            let bov_target = match r.size.blow_off {
                BlowOff::None => 0.0,
                _ => clamp((across - BLOW_OFF_CRACK) / BLOW_OFF_SPAN, 0.0, 1.0),
            };
            r.blow_off += (dt / BLOW_OFF_TAU) * (bov_target - r.blow_off);
            r.blow_off = clamp(r.blow_off, 0.0, 1.0);
            // The air in its bore cannot stop or start at once.
            let settled = vent_flow(r.size.blow_off_area * r.blow_off, p2, t2);
            r.vent += (1.0 - math::exp(-dt / BLOW_OFF_FLOW_TAU)) * (settled - r.vent);
            vent += r.vent;
            vent_h += r.vent * h2;
        }

        // --- The throttle body: in from the charge pipes, out through the throttle ---
        let out = throttle_flow + vent;
        let h_out = throttle_flow * h2;
        self.energy += (h_in - h_out - vent_h) * dt;
        self.mass += (m_in - out) * dt;
        let floor = 0.2 * (gas::P_AMB * charge.volume) / (gas::R * gas::T_AMB);
        if self.mass < floor || !self.mass.is_finite() || !self.energy.is_finite() {
            self.mass = math::max(if self.mass.is_finite() { self.mass } else { floor }, floor);
            self.energy = self.mass * gas_energy(gas::T_AMB);
        }
        let e_min = self.mass * gas_energy(200.0);
        if self.energy < e_min {
            self.energy = e_min;
        }

        let sound = self.sound(p2, t2);
        self.last_sound = sound;
        TurboOut { charge_p: self.charge_pressure(), charge_t: self.charge_temp(), sound }
    }

    /// What the turbos radiate, Pa at 1 m: the compressor inlets' flow, carrying the blades' whine and
    /// a stalled wheel's turbulence; the blow-off valves' jets; the air reversing through a wheel in a
    /// surge; and each wastegate flap rattling on the pulses across its turbine.
    fn sound(&mut self, p2: f64, t2: f64) -> f64 {
        let charge = self.charge;
        let level = charge.noise;
        let rho = gas::P_AMB / (gas::R * gas::T_AMB);
        let sample_rate = self.sample_rate;
        let noise = &mut self.noise;
        let mut drawn = 0.0;
        let mut rest = 0.0;
        let mut detune = 1.0;

        for r in self.rotors.iter_mut() {
            let size = r.size;
            let m_c = r.compressor_flow;
            let s = r.omega / size.full_speed;
            let shaft_hz = r.omega / (2.0 * PI);

            // --- The blades' modulation of the air drawn in, each turbo's a little apart in speed, as
            // two real ones are ---
            let mut whine = 0.0;
            for voice in r.whine_phase.iter_mut() {
                let speed = shaft_hz * detune;
                for (h, &(order, rel)) in WHINE_ORDERS.iter().enumerate() {
                    let hz = speed * order;
                    let fade = fade(hz, sample_rate);
                    let ph = &mut voice[h];
                    *ph += hz / sample_rate;
                    *ph -= ph.floor();
                    if fade > 0.0 {
                        whine += rel * fade * math::sin(2.0 * PI * *ph);
                    }
                }
                detune *= TURBO_DETUNE;
            }
            whine *= (WHINE_DEPTH * s * s) / size.count;

            // --- Stall: in a rotating stall, or forced backwards, the wheel's flow breaks up, around a few
            // shaft orders ---
            let stall = if s > 0.1 { r.breakdown() } else { 0.0 };
            // Two poles above, as the radiation from the inlet rises with the frequency.
            let stall_hz = clamp(shaft_hz, 50.0, 0.2 * sample_rate);
            let c_lp = 1.0 - math::exp((-2.0 * PI * 3.0 * stall_hz) / sample_rate);
            let c_hp = 1.0 - math::exp((-2.0 * PI * 0.3 * stall_hz) / sample_rate);
            r.stall_lp[0] += c_lp * (noise.next() - r.stall_lp[0]);
            r.stall_lp[1] += c_lp * (r.stall_lp[0] - r.stall_lp[1]);
            r.stall_hp += c_hp * (r.stall_lp[1] - r.stall_hp);
            let turbulence = (r.stall_lp[1] - r.stall_hp) * STALL_INTENSITY * stall * m_c.abs();
            drawn += m_c * (1.0 + whine) + turbulence;

            // --- A surge: the air forced backwards through the inducer ---
            let d_inducer = math::sqrt((4.0 * size.inducer_area) / PI);
            let u_reverse = math::max(-m_c, 0.0) / (rho * size.inducer_area);
            let reverse_pa = lighthill_pa(u_reverse, d_inducer);
            rest += r.reverse_jet.next(noise, u_reverse, d_inducer, reverse_pa, sample_rate);

            // --- The wastegate flap, just open, knocked on its seat by each pulse across the turbine ---
            let across_turbine = r.drive.inlet - r.drive.outlet;
            r.across_mean += (1.0 - math::exp((-2.0 * PI * 5.0) / sample_rate)) * (across_turbine - r.across_mean);
            let rising = across_turbine - r.across_mean;
            let high = rising > 0.15 * math::max(r.across_mean, 1e3);
            let wg = r.wastegate;
            if high && !r.across_high && wg > 0.0 && wg < WASTEGATE_RATTLE_OPENING {
                let open = 1.0 - wg / WASTEGATE_RATTLE_OPENING;
                r.rattle.trigger(WASTEGATE_RATTLE_PA * open * (rising / 1e4));
            }
            r.across_high = high;
            let hit = r.rattle.next();
            rest += r.rattle_modes[0].process(hit) + 0.6 * r.rattle_modes[1].process(hit);
        }

        // --- The compressor inlets: what the wheels draw from the air, less what a recirculating
        // blow-off valve hands back to them ---
        let recirculated: f64 =
            self.rotors.iter().filter(|r| matches!(r.size.blow_off, BlowOff::Recirculating)).map(|r| r.vent).sum();
        let inlet = self.inlet.process((drawn - recirculated) / rho);

        // --- The blow-off valves: an atmospheric one's outflow radiating, and each one's jet noise ---
        let mut vented = 0.0;
        let mut jet = 0.0;
        for r in self.rotors.iter_mut() {
            let d_vent = math::sqrt((4.0 * r.size.blow_off_area * r.blow_off) / PI);
            let u_vent = if r.vent > 0.0 { jet_velocity(p2, t2, gas::P_AMB) } else { 0.0 };
            let jet_share = match r.size.blow_off {
                BlowOff::Atmospheric => {
                    vented += r.vent;
                    1.0
                }
                _ => RECIRCULATING_TRANSMISSION,
            };
            let jet_pa = if r.vent > 0.0 { jet_share * lighthill_pa(u_vent, d_vent) } else { 0.0 };
            jet += r.blow_off_jet.next(&mut self.noise, u_vent, d_vent, jet_pa, sample_rate);
        }
        let vented = self.vent.process(vented / rho);

        level * (inlet + vented + jet + rest)
    }
}

/// Which of `groups` each of `count` turbos is in.
fn rotor_of(groups: &[(TurboSettings, Vec<usize>)], count: usize) -> Vec<usize> {
    let mut out = vec![0; count];
    for (g, (_, members)) in groups.iter().enumerate() {
        for &m in members {
            out[m] = g;
        }
    }
    out
}

/// 1 for a tone well below the Nyquist frequency, fading to 0 before it, so nothing folds back.
fn fade(hz: f64, sample_rate: f64) -> f64 {
    let top = math::min(0.45 * sample_rate, 18_000.0);
    let start = 0.7 * top;
    if hz >= top {
        0.0
    } else if hz <= start {
        1.0
    } else {
        1.0 - (hz - start) / (top - start)
    }
}
