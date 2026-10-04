//! The shared data model: the engine, its exhaust segments, the snapshot sent to the UI, gas
//! properties and the geometry helpers the physics uses.
//!
//! Units are SI throughout: metres, kilograms, seconds, kelvin, pascals.
//!
//! Crank-angle convention: 0 deg = TDC at the start of the power stroke, increasing with rotation,
//! cycle = 720 deg.
//!
//! ```text
//!     0-180    power / expansion   (piston down)
//!   180-360    exhaust            (piston up)
//!   360-540    intake             (piston down)
//!   540-720    compression        (piston up)
//! ```
//!
//! The types serialise to the same JSON as the web app's `src/model/spec.ts`, which is what the UI
//! sends and receives.

use serde::{Deserialize, Serialize};

use crate::exhaust_graph::ExhaustGraph;
use crate::listener::{SoundSources, Vec3};
use crate::math::{self, PI};

/// Shape of one length of exhaust plumbing.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum SegmentKind {
    /// Tube, tapering in a straight line from `d_in` to `d_out` where they differ.
    Pipe,
    /// Linear taper from `d_in` to `d_out`.
    Cone,
    /// Sudden expansion into a large-diameter volume, then back down: a muffler can.
    Chamber,
}

/// Cross-section of a chamber's body. The throats at either end are always round.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum ChamberSection {
    Round,
    Oval,
    /// Rounded corners, of radius `RECT_CORNER` times the shorter side.
    Rect,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PipeSegment {
    #[serde(default)]
    pub id: String,
    pub kind: SegmentKind,
    /// Axial length, m.
    pub length: f64,
    /// Inlet diameter, m.
    pub d_in: f64,
    /// Outlet diameter, m. For a chamber, the body's diameter if round and its width otherwise.
    pub d_out: f64,
    /// Routing only: a sharp corner at the segment's start. Never affects the 1D acoustics.
    #[serde(default)]
    pub yaw: f64,
    #[serde(default)]
    pub pitch: f64,
    /// Chamber only: body cross-section. Round when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub section: Option<ChamberSection>,
    /// Chamber only: body height, m, for an oval or rect body. Width is `d_out`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub height: Option<f64>,
    /// Chamber only: how far the inlet and outlet pipes sit off the body's centreline, m.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub offset_in: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub offset_out: Option<f64>,
}

/// Crank arrangement, where an engine has a choice of one. See `firing_plan`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum CrankType {
    Shared,
    Flatplane,
    Crossplane,
    Boxer,
}

/// Which way a header's primaries run to their merge, as drawn. The scene's only: see the web app's `EngineSpec`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum HeaderRun {
    Outward,
    Lengthways,
}

/// Where a turbocharged engine's blow-off valve vents, or `None` for no valve at all. See `turbo`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum BlowOff {
    Atmospheric,
    Recirculating,
    None,
}

/// What the exhaust is made of. See `PipeMaterial::wall`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum PipeMaterial {
    MildSteel,
    Stainless,
    CastIron,
    Titanium,
}

/// The properties of a pipe wall the gas dynamics feel.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WallMaterial {
    /// Density, kg/m^3, and specific heat, J/(kg K): the wall's thermal mass.
    pub density: f64,
    pub specific_heat: f64,
    /// Emissivity of the outer surface, as it is once the pipe has run hot.
    pub emissivity: f64,
    /// Equivalent sand-grain roughness of the bore, m: what sets the friction factor and lifts the
    /// heat transfer of the turbulent flow along it.
    pub roughness: f64,
}

impl PipeMaterial {
    /// Handbook values. The roughnesses are Moody's: commercial steel tube 0.045 mm, as-cast iron
    /// 0.26 mm, drawn stainless and titanium tube far smoother.
    pub fn wall(self) -> WallMaterial {
        match self {
            PipeMaterial::MildSteel => {
                WallMaterial { density: 7800.0, specific_heat: 490.0, emissivity: 0.8, roughness: 45e-6 }
            }
            PipeMaterial::Stainless => {
                WallMaterial { density: 8000.0, specific_heat: 500.0, emissivity: 0.6, roughness: 15e-6 }
            }
            PipeMaterial::CastIron => {
                WallMaterial { density: 7200.0, specific_heat: 460.0, emissivity: 0.9, roughness: 260e-6 }
            }
            PipeMaterial::Titanium => {
                WallMaterial { density: 4500.0, specific_heat: 520.0, emissivity: 0.5, roughness: 5e-6 }
            }
        }
    }
}

/// How the exhausts are plumbed, as `exhaust_layout_of` normalises it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ExhaustLayout {
    Open,
    PerBank,
    Merged,
}

/// Accepted layout values, including the names a single or a twin is described by.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ExhaustLayoutSpec {
    #[serde(rename = "open")]
    Open,
    #[serde(rename = "perBank")]
    PerBank,
    #[serde(rename = "merged")]
    Merged,
    #[serde(rename = "single")]
    Single,
    #[serde(rename = "2into2")]
    TwoIntoTwo,
    #[serde(rename = "2into1")]
    TwoIntoOne,
}

/// Everything about the engine. The web app's `EngineSpec` documents each field.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct EngineSpec {
    // --- Layout ---
    pub cylinders: u32,
    pub v_angle: f64,
    pub firing_offset: Option<f64>,
    pub firing_order: Option<Vec<u32>>,
    pub firing_intervals: Option<Vec<f64>>,
    pub crank_type: CrankType,
    pub exhaust_layout: ExhaustLayoutSpec,
    pub exhaust_headers: bool,
    pub header_run: HeaderRun,

    // --- Geometry ---
    pub bore: f64,
    pub stroke: f64,
    pub rod_length: f64,
    pub compression_ratio: f64,

    // --- Valves ---
    pub ex_valve_dia: f64,
    pub ex_valve_count: f64,
    pub port_length: f64,
    pub in_valve_dia: f64,
    pub in_valve_count: f64,
    pub max_lift: f64,
    pub evo: f64,
    pub evc: f64,
    pub ivo: f64,
    pub ivc: f64,
    pub cam_switch_rpm: f64,
    pub high_evo: f64,
    pub high_evc: f64,
    pub high_ivo: f64,
    pub high_ivc: f64,
    pub high_max_lift: f64,
    pub vvt_intake_low: f64,
    pub vvt_intake_high: f64,
    pub vvt_exhaust_low: f64,
    pub vvt_exhaust_high: f64,
    pub vvt_low_rpm: f64,
    pub vvt_high_rpm: f64,
    pub vvt_linked: bool,

    // --- Combustion ---
    pub ignition: f64,
    pub advance_curve: bool,
    pub burn_duration: f64,
    pub lambda: f64,
    pub fuel_cut: bool,
    pub overrun_crackle: bool,
    pub crackle_intensity: f64,
    pub combustion_variability: f64,
    pub recip_mass: f64,
    pub throttle: f64,
    pub throttle_dia: f64,
    pub plenum_volume: f64,
    pub plenum_length: f64,
    pub plenum_width: f64,
    pub plenum_height: f64,
    pub plenum_taper: f64,
    pub dual_plenum: bool,
    pub plenum_balance_rpm: f64,
    pub plenum_balance_shut_rpm: f64,
    pub plenum_balance_reopen_rpm: f64,
    pub airbox_volume: f64,
    pub snorkel_length: f64,
    pub snorkel_dia: f64,
    pub intake_runner_length: f64,
    pub intake_runner_dia: f64,
    pub intake_runner_short_length: f64,
    pub intake_switch_rpm: f64,

    // --- Turbocharger ---
    pub boost_target: f64,
    pub turbo_size: f64,
    pub intercooler: f64,
    pub blow_off: BlowOff,
    pub turbo_noise: f64,

    // --- Operating point ---
    pub rpm: f64,
    pub rev_limit: f64,
    /// Speed the idle air valve holds with the throttle shut, rev/min; 0 for no idle control.
    pub idle_rpm: f64,
    pub free_running: bool,
    pub flywheel_inertia: f64,
    pub load: f64,

    // --- Acoustics / output ---
    pub port_gas_temp: f64,
    pub pipe_cell_size: f64,
    pub pipe_wall_thickness: f64,
    pub pipe_material: PipeMaterial,
    pub air_speed: f64,
    pub exhaust_height: f64,
    pub cylinder_spread: f64,
    pub ground_reflection: f64,
    pub output_gain: f64,
    pub mech_noise: f64,
    pub throat_noise: f64,
    pub jet_noise: f64,
}

/// A 500cc-ish thumper: 89 mm bore, 80 mm stroke. The web app's `DEFAULT_ENGINE`.
impl Default for EngineSpec {
    fn default() -> Self {
        EngineSpec {
            cylinders: 1,
            v_angle: 45.0,
            firing_offset: None,
            firing_order: None,
            firing_intervals: None,
            crank_type: CrankType::Shared,
            exhaust_layout: ExhaustLayoutSpec::Open,
            exhaust_headers: false,
            header_run: HeaderRun::Outward,
            bore: 0.089,
            stroke: 0.08,
            rod_length: 0.145,
            compression_ratio: 10.5,

            ex_valve_dia: 0.034,
            ex_valve_count: 1.0,
            port_length: 0.055,
            in_valve_dia: 0.04,
            in_valve_count: 1.0,
            max_lift: 0.0095,
            evo: 128.0,
            evc: 378.0,
            ivo: 342.0,
            ivc: 576.0,
            cam_switch_rpm: 0.0,
            high_evo: 128.0,
            high_evc: 378.0,
            high_ivo: 342.0,
            high_ivc: 576.0,
            high_max_lift: 0.0095,
            vvt_intake_low: 0.0,
            vvt_intake_high: 0.0,
            vvt_exhaust_low: 0.0,
            vvt_exhaust_high: 0.0,
            vvt_low_rpm: 2000.0,
            vvt_high_rpm: 6000.0,
            vvt_linked: false,

            ignition: 695.0,
            burn_duration: 55.0,
            advance_curve: true,
            lambda: 1.0,
            fuel_cut: true,
            overrun_crackle: false,
            crackle_intensity: 0.6,
            combustion_variability: 1.0,
            recip_mass: 0.55,
            throttle: 0.75,
            throttle_dia: 0.0,
            plenum_volume: 0.0,
            plenum_length: 0.0,
            plenum_width: 0.0,
            plenum_height: 0.0,
            plenum_taper: 0.4,
            dual_plenum: false,
            plenum_balance_rpm: 0.0,
            plenum_balance_shut_rpm: 0.0,
            plenum_balance_reopen_rpm: 0.0,
            airbox_volume: 0.0,
            snorkel_length: 0.3,
            snorkel_dia: 0.0,
            intake_runner_length: 0.0,
            intake_runner_dia: 0.0,
            intake_runner_short_length: 0.0,
            intake_switch_rpm: 5000.0,

            boost_target: 0.7e5,
            turbo_size: 0.0,
            intercooler: 0.7,
            blow_off: BlowOff::Atmospheric,
            turbo_noise: 1.0,

            rpm: 3200.0,
            rev_limit: 7000.0,
            idle_rpm: 800.0,
            free_running: false,
            flywheel_inertia: 0.25,
            load: 0.46,

            port_gas_temp: 950.0,
            pipe_cell_size: 0.035,
            pipe_wall_thickness: 0.0012,
            pipe_material: PipeMaterial::MildSteel,
            air_speed: 0.0,
            exhaust_height: 0.35,
            cylinder_spread: 0.3,
            ground_reflection: 0.7,
            output_gain: 0.77,
            mech_noise: 0.45,
            throat_noise: 0.5,
            jet_noise: 0.5,
        }
    }
}

impl EngineSpec {
    /// This spec with the fields present in `patch`, a JSON object in the same shape, replaced.
    pub fn merged(&self, patch: &serde_json::Value) -> Result<EngineSpec, serde_json::Error> {
        let mut base = serde_json::to_value(self)?;
        if let (Some(obj), Some(p)) = (base.as_object_mut(), patch.as_object()) {
            for (k, v) in p {
                obj.insert(k.clone(), v.clone());
            }
        }
        serde_json::from_value(base)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct EngineConfig {
    pub engine: EngineSpec,
    /// The duct each cylinder's exhaust valve feeds: the runner every compiled graph starts from.
    #[serde(default)]
    pub pipe: Vec<PipeSegment>,
    /// The shared duct downstream of where a group of cylinders merges, one copy per collector.
    #[serde(default)]
    pub collector: Vec<PipeSegment>,
    /// The duct graph the exhaust is built from. Authoritative when present.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub graph: Option<ExhaustGraph>,
    /// Where the engine makes its sound, as drawn. Without it, or for a mouth it does not place, see
    /// `EngineSim::set_sources`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sources: Option<SoundSources>,
    /// Where the listener's ear is, m, in the same frame. Without it, see `EngineSim::set_listener`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub listener: Option<Vec3>,
}

/// Per-cylinder state, so the renderer can animate each bank.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BankSnapshot {
    pub crank_angle: f64,
    pub cyl_pressure: f64,
    pub cyl_temp: f64,
    pub ex_lift: f64,
    pub in_lift: f64,
}

/// A launch's state, sent with each snapshot while it runs.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct LaunchSnapshot {
    pub phase: String,
    pub gear: f64,
    pub speed_kmh: f64,
    /// Seconds since the clock started, once the car had rolled a foot.
    pub elapsed: f64,
    /// Distance covered, m.
    pub distance: f64,
    pub finished: bool,
    /// Seconds from the clock starting to 60 mph, once the car has reached it.
    pub zero_to_sixty: Option<f64>,
    /// Seconds to the quarter mile and the speed there, km/h, once the car has covered it.
    pub quarter_mile: Option<f64>,
    pub quarter_mile_kmh: Option<f64>,
    /// The same at the half mile.
    pub half_mile: Option<f64>,
    pub half_mile_kmh: Option<f64>,
    /// The throttle's opening the run holds, 0..1: open through a pull, shut through a shift that lifts off.
    pub throttle: f64,
    /// Engine cycles recorded since the last snapshot, `LAUNCH_POINT_STRIDE` values each.
    pub points: Vec<f32>,
}

fn is_zero(n: &u32) -> bool {
    *n == 0
}

/// Snapshot pushed from the audio thread to the UI at about 60 Hz for drawing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct EngineSnapshot {
    pub banks: Vec<BankSnapshot>,
    pub crank_angle: f64,
    pub rpm: f64,
    pub limiter: bool,
    pub fuel_cut: bool,
    pub intake_cam_advance: f64,
    pub exhaust_cam_retard: f64,
    pub short_runners: bool,
    pub high_cam: bool,
    pub cyl_pressure: f64,
    pub cyl_temp: f64,
    pub ex_lift: f64,
    pub in_lift: f64,
    pub torque: f64,
    /// Gauge pressure along the exhaust, Pa, `PIPE_PRESSURE_TAPS` long, port first.
    pub pipe_pressure: Vec<f32>,
    /// Gauge pressure in every cell of every exhaust duct, Pa: the ducts in the graph's order, each
    /// from its port end, taking `duct_cells` values in turn.
    pub duct_pressure: Vec<f32>,
    /// How many cells of `duct_pressure` each duct has.
    pub duct_cells: Vec<u32>,
    /// Each duct's id, in the same order.
    pub duct_ids: Vec<String>,
    /// Gauge pressure in every cell of the inlet tract, Pa, from the throttle out to the snorkel's mouth:
    /// the throttle's as it sees it. Empty on an engine with a turbo.
    pub inlet_pressure: Vec<f32>,
    /// Air speed in every cell of the inlet tract, m/s, the same way: positive out towards the snorkel's
    /// mouth, so negative as the engine draws.
    pub inlet_velocity: Vec<f32>,
    /// Gauge pressure in the plenum, Pa, over its volume: below zero, the manifold's vacuum.
    pub plenum_pressure: f64,
    /// Gauge pressure in each zone along the plenum, Pa, from the throttle at its front to its back: with
    /// dual plenums, bank 0's and then bank 1's, as many each.
    pub plenum_zones: Vec<f32>,
    /// Whether dual plenums' balance valve is open, joining them.
    pub plenum_balanced: bool,
    /// Gauge pressure in every cell of every intake runner, Pa, in cylinder order, each from its valve
    /// end, taking `runner_cells` values in turn.
    pub runner_pressure: Vec<f32>,
    pub runner_cells: Vec<u32>,
    pub peak: f64,
    pub pipe_cells: f64,
    pub substeps: f64,
    pub wall_temp: f64,
    pub launch: Option<LaunchSnapshot>,
    /// Afterfires since the last snapshot, and whether the overrun crackle map is running.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub afterfires: u32,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub crackle: bool,
    /// The turbocharger's state, on a turbocharged engine only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub turbo: Option<TurboSnapshot>,
}

/// A turbocharger's state, sent with each snapshot.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TurboSnapshot {
    /// Charge-air pressure, gauge, Pa.
    pub boost: f64,
    /// Plenum pressure, gauge, Pa: below zero is vacuum.
    pub manifold: f64,
    /// Pressure at the turbine inlet, gauge, Pa: what the exhaust works against.
    pub turbine_inlet: f64,
    /// The turbos' mean shaft speed, rev/min.
    pub shaft_rpm: f64,
    /// The wastegates' and the blow-off valves' mean openings, 0..1.
    pub wastegate: f64,
    pub blow_off: f64,
    /// Whether a compressor is surging.
    pub surging: bool,
    /// Each turbo's own, in the order of the exhaust's turbines.
    pub turbos: Vec<TurboUnitSnapshot>,
}

/// One turbo's state, sent with each snapshot: its shaft speed, rev/min, and its wastegate's and
/// blow-off valve's openings, 0..1.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TurboUnitSnapshot {
    pub id: String,
    pub shaft_rpm: f64,
    pub wastegate: f64,
    pub blow_off: f64,
}

/// The car and gearbox a launch drives through. See `LaunchRun`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct LaunchConfig {
    /// Gearbox ratios, first gear first.
    pub ratios: Vec<f64>,
    pub final_drive: f64,
    /// Tyre rolling radius, m.
    pub tyre_radius: f64,
    /// The tyres' friction coefficient at their peak: about 1.1 for a road tyre, 1.35 for a track tyre.
    pub tyre_grip: f64,
    /// The car's mass, kg.
    pub mass: f64,
    /// Share of the car's weight on the driven wheels at rest: 1 driving all four. Driving the rear
    /// wheels, more moves onto them as the car accelerates; driving the front, it moves off them.
    pub driven_load: f64,
    /// Whether the driven wheels are the front ones, which the car's weight moves off as it accelerates.
    #[serde(default)]
    pub front_wheel_drive: bool,
    /// Whether traction control keeps the driven wheels from spinning: see `LaunchRun`.
    pub traction_control: bool,
    /// Engine speed the clutch is slipped at off the line, rev/min.
    pub launch_rpm: f64,
    /// Engine speed each gear is pulled to before the shift, rev/min.
    pub shift_rpm: f64,
    /// How long each shift takes, from lifting off to full throttle in the next gear, s.
    pub shift_time: f64,
    /// Whether the gearbox is a dual clutch, which shifts with no gap in the drive.
    pub dual_clutch: bool,
    /// A dyno pull rather than a launch: the crank drives a dyno's absorber through one gear at 1:1, from
    /// `launch_rpm` to `shift_rpm`, and the car, tyres and gearbox are not used. See `LaunchRun`.
    #[serde(default)]
    pub dyno: bool,
    /// How fast a dyno pull sweeps the engine up, rev/min per s.
    #[serde(default = "default_sweep_rate")]
    pub sweep_rate: f64,
}

fn default_sweep_rate() -> f64 {
    500.0
}

pub const PIPE_PRESSURE_TAPS: usize = 128;

/// Brake mean effective pressure that `load = 1` stands for, Pa.
pub const FULL_LOAD_BMEP: f64 = 11e5;

/// How far below `rev_limit` the free-running crank must fall before the spark returns, rev/min.
pub const REV_LIMIT_HYSTERESIS_RPM: f64 = 200.0;

// ---------------------------------------------------------------------------
// Gas properties
// ---------------------------------------------------------------------------

pub mod gas {
    /// Specific gas constant for air / exhaust, J/(kg*K).
    pub const R: f64 = 287.0;
    /// Ratio of specific heats of cylinder gas at compression temperatures.
    pub const GAMMA_CYL: f64 = 1.35;
    /// Ratio of specific heats in the exhaust pipe.
    pub const GAMMA_EXH: f64 = 1.33;
    /// Ratio of specific heats in ambient air.
    pub const GAMMA_AIR: f64 = 1.4;
    /// Ambient pressure, Pa.
    pub const P_AMB: f64 = 101325.0;
    /// Ambient temperature, K.
    pub const T_AMB: f64 = 293.0;
    /// Cylinder wall temperature, K.
    pub const T_WALL: f64 = 450.0;
    /// Lower heating value of gasoline, J/kg.
    pub const FUEL_LHV: f64 = 43.2e6;
    /// Stoichiometric air-fuel ratio of gasoline, by mass.
    pub const AFR_STOICH: f64 = 14.7;
}

/// Throttle opening at or below which the throttle counts as shut for the overrun fuel cut.
pub const FUEL_CUT_THROTTLE: f64 = 0.005;

/// Speeds at which the overrun fuel cut acts, rev/min, and how far above the idle speed each is at least:
/// an engine that idles high has to be clear of its idle before the fuel stops. See `fuel_cut_rpms`.
pub const FUEL_CUT_RPM: f64 = 1500.0;
pub const FUEL_RESUME_RPM: f64 = 1200.0;
pub const FUEL_CUT_ABOVE_IDLE: f64 = 700.0;
pub const FUEL_RESUME_ABOVE_IDLE: f64 = 400.0;

/// The speeds `spec`'s overrun fuel cut stops the fuel above and brings it back below, rev/min.
pub fn fuel_cut_rpms(spec: &EngineSpec) -> (f64, f64) {
    (
        math::max(FUEL_CUT_RPM, spec.idle_rpm + FUEL_CUT_ABOVE_IDLE),
        math::max(FUEL_RESUME_RPM, spec.idle_rpm + FUEL_RESUME_ABOVE_IDLE),
    )
}

/// Fuel mass fraction of a charge mixed at `lambda`, 0..1.
pub fn fuel_fraction_at(lambda: f64) -> f64 {
    1.0 / (1.0 + math::max(lambda, 0.05) * gas::AFR_STOICH)
}

/// Specific heat of the gas in the cylinder and the plenum, rising linearly with temperature:
/// `cv(T) = CV_REF + CV_SLOPE (T - T_REF)`.
///
/// Temperature, not composition, is what moves gamma. As a gas heats up its molecules' vibrational
/// modes take up energy, so cv climbs: burned gas expanding at 1500-2500 K sits near gamma 1.25-1.30,
/// while the same gas compressed at 500 K is back near 1.35. Fixed at the compression value, the
/// burned gas's cv would come out a quarter too small and overheat the combustion by several hundred
/// kelvin. Anchored at gamma 1.35 at 500 K and 1.28 at 1800 K.
const CV_AT_500: f64 = gas::R / (1.35 - 1.0);
const CV_AT_1800: f64 = gas::R / (1.28 - 1.0);
pub const CV_SLOPE: f64 = (CV_AT_1800 - CV_AT_500) / (1800.0 - 500.0);

/// Datum of the sensible internal energy, K.
pub const T_REF: f64 = 298.0;
pub const CV_REF: f64 = CV_AT_500 + CV_SLOPE * (T_REF - 500.0);

/// cv at `t` (K), J/(kg*K).
pub fn gas_cv(t: f64) -> f64 {
    CV_REF + CV_SLOPE * (t - T_REF)
}

/// Ratio of specific heats at `t` (K).
pub fn gas_gamma(t: f64) -> f64 {
    1.0 + gas::R / gas_cv(t)
}

/// Sensible internal energy at `t` (K), J/kg.
pub fn gas_energy(t: f64) -> f64 {
    let d = t - T_REF;
    d * (CV_REF + 0.5 * CV_SLOPE * d)
}

/// Temperature, K, at sensible internal energy `u` (J/kg): `gas_energy` inverted.
pub fn gas_temperature(u: f64) -> f64 {
    T_REF + (2.0 * u) / (CV_REF + math::sqrt(CV_REF * CV_REF + 2.0 * CV_SLOPE * u))
}

/// Specific enthalpy at `t` (K), J/kg, on the same datum.
pub fn gas_enthalpy(t: f64) -> f64 {
    gas_energy(t) + gas::R * t
}

/// Speed of sound at temperature `t` (K), m/s.
pub fn speed_of_sound(t: f64, gamma: f64) -> f64 {
    math::sqrt(gamma * gas::R * t)
}

/// Speed of sound in exhaust gas at `t`.
pub fn speed_of_sound_exh(t: f64) -> f64 {
    speed_of_sound(t, gas::GAMMA_EXH)
}

/// Speed of sound in ambient air, m/s.
pub fn ambient_sound_speed() -> f64 {
    speed_of_sound(gas::T_AMB, gas::GAMMA_AIR)
}

/// Gas density at pressure `p` (Pa) and temperature `t` (K), kg/m^3.
pub fn density(p: f64, t: f64) -> f64 {
    p / (gas::R * t)
}

// ---------------------------------------------------------------------------
// Derived geometry
// ---------------------------------------------------------------------------

/// Swept volume of one cylinder, m^3.
pub fn displacement(spec: &EngineSpec) -> f64 {
    (PI * spec.bore * spec.bore) / 4.0 * spec.stroke
}

/// Nominal full-throttle torque of the whole engine, N*m: `boosted`, with a turbocharger, in
/// proportion to the charge pressure the wastegate holds.
pub fn full_load_torque(spec: &EngineSpec, boosted: bool) -> f64 {
    let torque = (FULL_LOAD_BMEP * displacement(spec) * spec.cylinders as f64) / (4.0 * PI);
    if boosted { torque * (gas::P_AMB + spec.boost_target) / gas::P_AMB } else { torque }
}

/// The braking torque `load` asks for, N*m.
pub fn load_torque_of(spec: &EngineSpec, boosted: bool) -> f64 {
    spec.load * full_load_torque(spec, boosted)
}

/// Diameter of the exhaust port, m: of the same area as the exhaust valve heads together.
pub fn exhaust_port_diameter(spec: &EngineSpec) -> f64 {
    spec.ex_valve_dia * math::sqrt(spec.ex_valve_count)
}

/// Length and bore of the intake runner, m: the spec's own, or sized for the engine.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RunnerSize {
    pub length: f64,
    pub diameter: f64,
}

/// Multiple of the crank speed an auto-sized runner's quarter-wave resonance is tuned to.
const RUNNER_TUNE_ORDER: f64 = 2.3;

pub fn intake_runner_of(spec: &EngineSpec) -> RunnerSize {
    let diameter = if spec.intake_runner_dia > 0.0 {
        spec.intake_runner_dia
    } else {
        0.9 * spec.in_valve_dia * math::sqrt(spec.in_valve_count)
    };
    if spec.intake_runner_length > 0.0 {
        return RunnerSize { length: spec.intake_runner_length, diameter };
    }
    let tuned_hz = RUNNER_TUNE_ORDER * ((0.75 * spec.rev_limit) / 60.0);
    RunnerSize { length: speed_of_sound(gas::T_AMB, gas::GAMMA_AIR) / (4.0 * tuned_hz), diameter }
}

/// Clearance (TDC) volume, m^3.
pub fn clearance_volume(spec: &EngineSpec) -> f64 {
    displacement(spec) / (spec.compression_ratio - 1.0)
}

const DEG_TO_RAD: f64 = PI / 180.0;

/// Spec-derived crank geometry: everything the crank evaluation needs that does not depend on angle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CrankGeometry {
    /// Crank throw, m.
    pub a: f64,
    pub a2: f64,
    pub l2: f64,
    /// Bore cross-section, m^2.
    pub area: f64,
    /// Clearance volume, m^3.
    pub vc: f64,
    /// `a + rod_length`, the piston position at TDC.
    pub top: f64,
}

impl CrankGeometry {
    pub fn of(spec: &EngineSpec) -> CrankGeometry {
        let a = spec.stroke / 2.0;
        let l = spec.rod_length;
        CrankGeometry {
            a,
            a2: a * a,
            l2: l * l,
            area: (PI * spec.bore * spec.bore) / 4.0,
            vc: clearance_volume(spec),
            top: a + l,
        }
    }
}

/// Distance from crank centre to the piston pin, m, at crank angle `deg`.
pub fn piston_position(g: &CrankGeometry, deg: f64) -> f64 {
    let th = deg * DEG_TO_RAD;
    let s = math::sin(th);
    g.a * math::cos(th) + math::sqrt(math::max(g.l2 - g.a2 * s * s, 0.0))
}

/// Cylinder volume, m^3, at crank angle `deg`.
pub fn cylinder_volume(spec: &EngineSpec, deg: f64) -> f64 {
    let g = CrankGeometry::of(spec);
    let a = spec.stroke / 2.0;
    let area = (PI * spec.bore * spec.bore) / 4.0;
    let top = a + spec.rod_length;
    clearance_volume(spec) + area * (top - piston_position(&g, deg))
}

/// Everything the cylinder model needs from the crank at one angle.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CrankState {
    /// Distance from crank centre to the piston pin, m.
    pub position: f64,
    /// dx/dtheta, m per radian.
    pub d_position: f64,
    /// d2x/dtheta^2, m per radian^2.
    pub d2_position: f64,
    /// Cylinder volume, m^3.
    pub volume: f64,
    /// dV/dtheta, m^3 per radian.
    pub d_volume: f64,
}

/// The crank at `angle` degrees, from one `sin`, one `cos` and one `sqrt`.
#[inline]
pub fn crank_at(g: &CrankGeometry, angle: f64) -> CrankState {
    let th = angle * DEG_TO_RAD;
    let (sin, cos) = math::sincos(th);
    let root = math::sqrt(math::max(g.l2 - g.a2 * sin * sin, 1e-12));
    let inv_root = 1.0 / root;

    let position = g.a * cos + root;
    let d_position = -g.a * sin - g.a2 * sin * cos * inv_root;
    let cos2 = cos * cos - sin * sin;
    let d2_position =
        -g.a * cos - g.a2 * (cos2 * inv_root + g.a2 * sin * sin * cos * cos * inv_root * inv_root * inv_root);
    CrankState {
        position,
        d_position,
        d2_position,
        volume: g.vc + g.area * (g.top - position),
        d_volume: -g.area * d_position,
    }
}

// ---------------------------------------------------------------------------
// Pipe helpers
// ---------------------------------------------------------------------------

/// Fraction of a chamber's length taken by each of its throats.
pub const CHAMBER_THROAT: f64 = 0.08;

/// Corner radius of a `Rect` section, as a fraction of its shorter side.
pub const RECT_CORNER: f64 = 0.15;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Section {
    pub section: ChamberSection,
    pub width: f64,
    pub height: f64,
}

/// Diameter, m, at normalised position `u` (0..1) within a segment. For a non-round chamber body,
/// the diameter of the circle with the same area.
pub fn segment_diameter(seg: &PipeSegment, u: f64) -> f64 {
    match seg.kind {
        SegmentKind::Pipe | SegmentKind::Cone => seg.d_in + (seg.d_out - seg.d_in) * u,
        SegmentKind::Chamber => {
            if u < CHAMBER_THROAT || u > 1.0 - CHAMBER_THROAT {
                return seg.d_in;
            }
            if seg.section.unwrap_or(ChamberSection::Round) == ChamberSection::Round {
                return seg.d_out;
            }
            math::sqrt((4.0 * section_area(&chamber_body(seg))) / PI)
        }
    }
}

/// The body cross-section of a chamber.
pub fn chamber_body(seg: &PipeSegment) -> Section {
    let section = seg.section.unwrap_or(ChamberSection::Round);
    let width = seg.d_out;
    Section {
        section,
        width,
        height: if section == ChamberSection::Round { width } else { seg.height.unwrap_or(width) },
    }
}

/// Cross-section at normalised position `u` within a segment.
pub fn segment_section(seg: &PipeSegment, u: f64) -> Section {
    if seg.kind == SegmentKind::Chamber && (CHAMBER_THROAT..=1.0 - CHAMBER_THROAT).contains(&u) {
        return chamber_body(seg);
    }
    let d = segment_diameter(seg, u);
    Section { section: ChamberSection::Round, width: d, height: d }
}

pub fn section_area(s: &Section) -> f64 {
    match s.section {
        ChamberSection::Round => (PI * s.width * s.width) / 4.0,
        ChamberSection::Oval => (PI * s.width * s.height) / 4.0,
        ChamberSection::Rect => {
            let r = RECT_CORNER * math::min(s.width, s.height);
            s.width * s.height - (4.0 - PI) * r * r
        }
    }
}

/// Wetted perimeter, m. Ramanujan's second approximation for the ellipse.
pub fn section_perimeter(s: &Section) -> f64 {
    match s.section {
        ChamberSection::Round => PI * s.width,
        ChamberSection::Oval => {
            let a = s.width / 2.0;
            let b = s.height / 2.0;
            let h = ((a - b) * (a - b)) / ((a + b) * (a + b));
            PI * (a + b) * (1.0 + (3.0 * h) / (10.0 + math::sqrt(4.0 - 3.0 * h)))
        }
        ChamberSection::Rect => {
            let r = RECT_CORNER * math::min(s.width, s.height);
            2.0 * (s.width + s.height) - (8.0 - 2.0 * PI) * r
        }
    }
}

/// Whether the point `(y, z)`, measured from the section's centre, is inside it.
pub fn inside_section(s: &Section, y: f64, z: f64) -> bool {
    let a = s.width / 2.0;
    let b = s.height / 2.0;
    match s.section {
        ChamberSection::Round | ChamberSection::Oval => (y * y) / (a * a) + (z * z) / (b * b) <= 1.0,
        ChamberSection::Rect => {
            let r = RECT_CORNER * math::min(s.width, s.height);
            let dy = y.abs() - (a - r);
            let dz = z.abs() - (b - r);
            if y.abs() > a || z.abs() > b {
                return false;
            }
            if dy <= 0.0 || dz <= 0.0 {
                return true;
            }
            dy * dy + dz * dz <= r * r
        }
    }
}

/// A chamber's inlet and outlet offsets, m, held to what fits in its body.
pub fn chamber_offsets(seg: &PipeSegment) -> (f64, f64) {
    if seg.kind != SegmentKind::Chamber {
        return (0.0, 0.0);
    }
    let room = math::max(0.0, (seg.d_out - seg.d_in) / 2.0);
    let hold = |v: Option<f64>| math::clamp(v.unwrap_or(0.0), -room, room);
    (hold(seg.offset_in), hold(seg.offset_out))
}

/// Gas temperature, K, at distance `x` along the pipe: the exhaust's starting guess.
pub fn pipe_temperature(port_temp: f64, x: f64) -> f64 {
    let decay_length = 1.6;
    gas::T_AMB + (port_temp - gas::T_AMB) * math::exp(-x / decay_length)
}

static SEGMENT_IDS: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

pub fn new_segment_id() -> String {
    let n = SEGMENT_IDS.fetch_add(1, std::sync::atomic::Ordering::Relaxed) + 1;
    format!("seg{n}")
}

/// The fields of a segment to be made, each defaulting as `make_segment` fills it in.
#[derive(Clone, Debug, Default)]
pub struct SegmentPartial {
    pub id: Option<String>,
    pub kind: Option<SegmentKind>,
    pub length: Option<f64>,
    pub d_in: Option<f64>,
    pub d_out: Option<f64>,
    pub yaw: Option<f64>,
    pub pitch: Option<f64>,
    pub section: Option<ChamberSection>,
    pub height: Option<f64>,
    pub offset_in: Option<f64>,
    pub offset_out: Option<f64>,
}

impl From<&PipeSegment> for SegmentPartial {
    fn from(s: &PipeSegment) -> Self {
        SegmentPartial {
            id: Some(s.id.clone()),
            kind: Some(s.kind),
            length: Some(s.length),
            d_in: Some(s.d_in),
            d_out: Some(s.d_out),
            yaw: Some(s.yaw),
            pitch: Some(s.pitch),
            section: s.section,
            height: s.height,
            offset_in: s.offset_in,
            offset_out: s.offset_out,
        }
    }
}

pub fn make_segment(partial: SegmentPartial) -> PipeSegment {
    let kind = partial.kind.unwrap_or(SegmentKind::Pipe);
    let d_in = partial.d_in.unwrap_or(0.042);
    let mut seg = PipeSegment {
        id: partial.id.unwrap_or_else(new_segment_id),
        kind,
        length: partial.length.unwrap_or(0.3),
        d_in,
        d_out: partial.d_out.unwrap_or(d_in),
        yaw: partial.yaw.unwrap_or(0.0),
        pitch: partial.pitch.unwrap_or(0.0),
        section: None,
        height: None,
        offset_in: None,
        offset_out: None,
    };
    if kind == SegmentKind::Chamber {
        if let Some(section) = partial.section {
            if section != ChamberSection::Round {
                seg.section = Some(section);
                seg.height = Some(match partial.height {
                    Some(h) if h.is_finite() && h > 1e-3 => h,
                    _ => seg.d_out,
                });
            }
        }
        if let Some(v) = partial.offset_in {
            if v.is_finite() && v != 0.0 {
                seg.offset_in = Some(v);
            }
        }
        if let Some(v) = partial.offset_out {
            if v.is_finite() && v != 0.0 {
                seg.offset_out = Some(v);
            }
        }
    }
    seg
}

/// A copy of `seg` through `make_segment`, as the web app's `makeSegment(seg)` makes one.
pub fn copy_segment(seg: &PipeSegment) -> PipeSegment {
    make_segment(SegmentPartial::from(seg))
}

// ---------------------------------------------------------------------------
// Firing plans
// ---------------------------------------------------------------------------

/// Crank degrees from cylinder 1 firing to cylinder 2 firing.
pub fn firing_offset_deg(spec: &EngineSpec) -> f64 {
    let derived = 360.0 + spec.v_angle;
    (spec.firing_offset.unwrap_or(derived) % 720.0 + 720.0) % 720.0
}

/// Which firings happen when, and which bank each belongs to.
#[derive(Clone, Debug, PartialEq)]
pub struct FiringPlan {
    /// Crank degrees after cylinder 1 at which each cylinder fires, in [0, 720).
    pub offsets: Vec<f64>,
    /// Bank index per cylinder, 0 or 1.
    pub banks: Vec<u32>,
    /// Number of banks actually used.
    pub bank_count: u32,
    /// Which crank throw each cylinder is on, where that is stated rather than recovered.
    pub throws: Option<Vec<usize>>,
}

struct PinCrank {
    pins: &'static [f64],
    revs: &'static [f64],
    banks: &'static [u32],
}

const V8_CROSSPLANE: PinCrank = PinCrank {
    pins: &[0.0, 0.0, 270.0, 270.0, 90.0, 90.0, 180.0, 180.0],
    revs: &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0],
    banks: &[0, 1, 0, 1, 0, 1, 0, 1],
};

const V8_FLATPLANE: PinCrank = PinCrank {
    pins: &[0.0, 0.0, 180.0, 180.0, 180.0, 180.0, 0.0, 0.0],
    revs: &[0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 1.0, 1.0],
    banks: &[0, 1, 0, 1, 0, 1, 0, 1],
};

const V6_SPLIT_PIN: PinCrank = PinCrank {
    pins: &[0.0, 60.0, 240.0, 300.0, 120.0, 180.0],
    revs: &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0],
    banks: &[0, 1, 0, 1, 0, 1],
};
const V6_THROWS: [usize; 6] = [0, 0, 1, 1, 2, 2];

/// The Honda VFR's V4: a 180-degree crank, each throw shared by both banks.
const V4_180: PinCrank =
    PinCrank { pins: &[0.0, 0.0, 180.0, 180.0], revs: &[0.0, 1.0, 0.0, 1.0], banks: &[0, 1, 0, 1] };

const BOXER_4: PinCrank =
    PinCrank { pins: &[0.0, 180.0, 180.0, 0.0], revs: &[0.0, 0.0, 0.0, 1.0], banks: &[0, 1, 0, 1] };
const BOXER_6: PinCrank = PinCrank {
    pins: &[0.0, 180.0, 240.0, 60.0, 120.0, 300.0],
    revs: &[0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    banks: &[0, 1, 0, 1, 0, 1],
};

/// The pins of the first bank of a flat engine with `per_bank` cylinders a side and no crank of its own
/// in the tables: spread evenly round the shaft, so with each opposite pin half a turn round from its
/// partner's the engine fires evenly.
fn boxer_bank_pins(per_bank: usize) -> &'static [f64] {
    match per_bank {
        1 => &[0.0],
        4 => &[0.0, 180.0, 90.0, 270.0],
        5 => &[0.0, 144.0, 288.0, 72.0, 216.0],
        _ => &[0.0, 240.0, 120.0, 300.0, 60.0, 180.0],
    }
}

/// Inline engines' firing offsets by cylinder, read off the usual firing orders.
fn inline_offsets(cylinders: u32) -> Option<&'static [f64]> {
    match cylinders {
        1 => Some(&[0.0]),
        3 => Some(&[0.0, 480.0, 240.0]),
        4 => Some(&[0.0, 540.0, 180.0, 360.0]),
        5 => Some(&[0.0, 144.0, 576.0, 288.0, 432.0]),
        6 => Some(&[0.0, 480.0, 240.0, 600.0, 120.0, 360.0]),
        _ => None,
    }
}

fn wrap720(deg: f64) -> f64 {
    ((deg % 720.0) + 720.0) % 720.0
}

fn pin_offsets(crank: &PinCrank, v_angle: f64) -> Vec<f64> {
    crank
        .pins
        .iter()
        .enumerate()
        .map(|(i, &pin)| (((pin + v_angle * crank.banks[i] as f64 + 360.0 * crank.revs[i]) % 720.0) + 720.0) % 720.0)
        .collect()
}

/// Whether the engine has two banks of cylinders: a V or a flat engine, with an even count and a vee.
pub fn has_two_banks(spec: &EngineSpec) -> bool {
    spec.v_angle > 0.0 && spec.cylinders >= 2 && spec.cylinders.is_multiple_of(2)
}

/// Cylinders on each bank.
pub fn cylinders_per_bank(spec: &EngineSpec) -> u32 {
    if has_two_banks(spec) { spec.cylinders / 2 } else { spec.cylinders }
}

/// Whether the layout is one the engine can be: one bank of 1 to 6, or two of 1 to 6 each.
pub fn valid_layout(spec: &EngineSpec) -> bool {
    (1..=6).contains(&cylinders_per_bank(spec))
}

/// Whether the spec is a flat engine: two banks laid flat, 180 degrees apart, on a boxer crank.
pub fn is_boxer(spec: &EngineSpec) -> bool {
    spec.crank_type == CrankType::Boxer && has_two_banks(spec) && (spec.v_angle - 180.0).abs() < 1e-9
}

fn boxer_plan(spec: &EngineSpec) -> FiringPlan {
    let per_bank = cylinders_per_bank(spec) as usize;
    let offsets = match per_bank {
        2 => pin_offsets(&BOXER_4, spec.v_angle),
        3 => pin_offsets(&BOXER_6, spec.v_angle),
        _ => {
            // Each opposed pair fires a revolution apart: the first bank's pin, then its partner's.
            let mut offsets = Vec::with_capacity(2 * per_bank);
            for &pin in boxer_bank_pins(per_bank) {
                offsets.push(wrap720(pin));
                offsets.push(wrap720(pin + 180.0 + spec.v_angle));
            }
            offsets
        }
    };
    let n = offsets.len();
    FiringPlan {
        offsets,
        banks: (0..n).map(|i| (i % 2) as u32).collect(),
        bank_count: 2,
        throws: Some((0..n).collect()),
    }
}

/// A V of two banks of an inline engine's crank, each throw shared: the second bank's cylinder fires the
/// bank angle after its partner, in the same revolution. On an inline five's crank that is the even 72-degree
/// firing of a 72-degree V10 and the 54-90 of the Viper's 90; on an inline six's, a 60-degree V12's even 60.
fn doubled_inline(inline: &[f64], v_angle: f64) -> FiringPlan {
    let mut offsets = Vec::with_capacity(2 * inline.len());
    for &o in inline {
        offsets.push(o);
        offsets.push(wrap720(o + v_angle));
    }
    let n = offsets.len();
    FiringPlan { offsets, banks: (0..n).map(|i| (i % 2) as u32).collect(), bank_count: 2, throws: None }
}

/// The layout's own firing plan, from real engines' cranks, before any firing order the spec sets.
pub fn default_firing_plan(spec: &EngineSpec) -> FiringPlan {
    if is_boxer(spec) {
        return boxer_plan(spec);
    }
    let crank = |crank: &PinCrank, throws: Option<Vec<usize>>| FiringPlan {
        offsets: pin_offsets(crank, spec.v_angle),
        banks: crank.banks.to_vec(),
        bank_count: 2,
        throws,
    };
    // Two of one is a twin either way: a V-twin's cylinders are each their own bank, and a parallel twin's
    // firing offset is expressed the same way.
    if spec.cylinders == 2 {
        return FiringPlan {
            offsets: vec![0.0, firing_offset_deg(spec)],
            banks: vec![0, 1],
            bank_count: 2,
            throws: None,
        };
    }
    if has_two_banks(spec) {
        match spec.cylinders {
            4 => return crank(&V4_180, None),
            6 => return crank(&V6_SPLIT_PIN, Some(V6_THROWS.to_vec())),
            8 => {
                return crank(
                    if spec.crank_type == CrankType::Flatplane { &V8_FLATPLANE } else { &V8_CROSSPLANE },
                    None,
                );
            }
            10 | 12 => return doubled_inline(inline_offsets(spec.cylinders / 2).unwrap_or(&[0.0]), spec.v_angle),
            _ => {}
        }
    }
    match inline_offsets(spec.cylinders) {
        Some(offsets) => {
            FiringPlan { offsets: offsets.to_vec(), banks: vec![0; offsets.len()], bank_count: 1, throws: None }
        }
        // A layout the engine cannot be: an even firing, one bank, so it still runs.
        None => {
            let n = spec.cylinders.max(1) as usize;
            FiringPlan {
                offsets: (0..n).map(|i| i as f64 * 720.0 / n as f64).collect(),
                banks: vec![0; n],
                bank_count: 1,
                throws: None,
            }
        }
    }
}

/// The firing offsets the spec's own firing order and intervals give, cylinder 1 at 0, or `None` where it
/// sets neither or they do not make a cycle: an order that is not every cylinder once, or intervals that
/// are not one per cylinder, none negative, summing to 720. A gap of 0 fires two cylinders together.
pub fn custom_offsets(spec: &EngineSpec, default: &FiringPlan) -> Option<Vec<f64>> {
    if spec.firing_order.is_none() && spec.firing_intervals.is_none() {
        return None;
    }
    let n = default.offsets.len();
    let order: Vec<usize> = match &spec.firing_order {
        Some(order) => {
            let mut seen = vec![false; n];
            for &c in order {
                let c = c as usize;
                if c == 0 || c > n || seen[c - 1] {
                    return None;
                }
                seen[c - 1] = true;
            }
            if order.len() != n {
                return None;
            }
            order.iter().map(|&c| c as usize - 1).collect()
        }
        None => {
            // The layout's own order: its cylinders by when they fire.
            let mut order: Vec<usize> = (0..n).collect();
            order.sort_by(|&a, &b| default.offsets[a].total_cmp(&default.offsets[b]));
            order
        }
    };
    let even = 720.0 / n as f64;
    let intervals: Vec<f64> = match &spec.firing_intervals {
        Some(gaps) => {
            let sum: f64 = gaps.iter().sum();
            if gaps.len() != n || gaps.iter().any(|g| !g.is_finite() || *g < 0.0) || (sum - 720.0).abs() > 1e-6 {
                return None;
            }
            gaps.clone()
        }
        None => vec![even; n],
    };
    let mut offsets = vec![0.0; n];
    let mut at = 0.0;
    for (k, &c) in order.iter().enumerate() {
        offsets[c] = at;
        at += intervals[k];
    }
    let first = offsets[0];
    Some(offsets.iter().map(|&o| wrap720(o - first)).collect())
}

/// Which firings happen when: the layout's own plan, or the spec's firing order and intervals on it.
pub fn firing_plan(spec: &EngineSpec) -> FiringPlan {
    let default = default_firing_plan(spec);
    match custom_offsets(spec, &default) {
        // Every cylinder of a flat engine keeps its own throw; on any other a pin is shared where the
        // order lets two cylinders share one.
        Some(offsets) => FiringPlan { offsets, throws: if is_boxer(spec) { default.throws } else { None }, ..default },
        None => default,
    }
}

/// One crankpin: where it sits round the shaft, and which cylinders hang off it.
#[derive(Clone, Debug, PartialEq)]
pub struct CrankPin {
    pub angle_deg: f64,
    pub cylinders: Vec<usize>,
    pub angles: Vec<f64>,
}

/// Which side of the engine a cylinder physically sits on: 0 or 1 in a V, 0 otherwise.
pub fn physical_bank(spec: &EngineSpec, cylinder: usize) -> u32 {
    if !(spec.v_angle > 0.0) {
        return 0;
    }
    firing_plan(spec).banks.get(cylinder).copied().unwrap_or(0)
}

/// How many physical banks the engine has: 2 for a V, 1 otherwise.
pub fn physical_bank_count(spec: &EngineSpec) -> u32 {
    if spec.v_angle > 0.0 { firing_plan(spec).bank_count } else { 1 }
}

/// How far apart along the crank the cylinders sharing a throw sit, m: a rod's width, so their rods run
/// side by side on a shared pin, or each on its own pin of a split one.
pub const ROD_STAGGER: f64 = 0.016;

/// Where `cylinder` sits along the crank, m, the engine centred on the origin: at its throw, and on a
/// throw it shares, staggered from the other cylinders on it by `ROD_STAGGER`.
pub fn cylinder_z(spec: &EngineSpec, cylinder: usize) -> f64 {
    let pins = crank_pins(spec);
    let index = pins.iter().position(|p| p.cylinders.contains(&cylinder)).unwrap_or(0);
    let along = (index as f64 - (pins.len() as f64 - 1.0) / 2.0) * cylinder_spacing(spec);
    match pins.get(index) {
        Some(pin) if pin.cylinders.len() >= 2 => {
            let k = pin.cylinders.iter().position(|&c| c == cylinder).unwrap_or(0) as f64;
            along + (k - (pin.cylinders.len() as f64 - 1.0) / 2.0) * ROD_STAGGER
        }
        _ => along,
    }
}

/// Centre-to-centre spacing of the crank pins along the crankshaft, m.
pub fn cylinder_spacing(spec: &EngineSpec) -> f64 {
    if is_boxer(spec) {
        return spec.bore * 0.75;
    }
    spec.bore * 1.45
}

pub fn crank_pins(spec: &EngineSpec) -> Vec<CrankPin> {
    let plan = firing_plan(spec);
    let mut pins: Vec<CrankPin> = Vec::new();
    let mut taken: Vec<bool> = vec![false; plan.offsets.len()];

    let pin_angle = |i: usize| (((plan.offsets[i] - spec.v_angle * plan.banks[i] as f64) % 360.0) + 360.0) % 360.0;

    let vee = ((spec.v_angle % 360.0) + 360.0) % 360.0;
    let can_share_a_pin = math::min(vee, 360.0 - vee) > 1e-6;

    if let Some(throws) = &plan.throws {
        let count = throws.iter().copied().max().unwrap_or(0) + 1;
        for t in 0..count {
            let cylinders: Vec<usize> = (0..plan.offsets.len()).filter(|&i| throws[i] == t).collect();
            if cylinders.is_empty() {
                continue;
            }
            let angles: Vec<f64> = cylinders.iter().map(|&i| pin_angle(i)).collect();
            pins.push(CrankPin { angle_deg: angles[0], cylinders, angles });
        }
        return pins;
    }

    for i in 0..plan.offsets.len() {
        if taken[i] || plan.banks[i] != 0 {
            continue;
        }
        taken[i] = true;
        let mut pin = CrankPin { angle_deg: pin_angle(i), cylinders: vec![i], angles: vec![pin_angle(i)] };
        if can_share_a_pin {
            for j in 0..plan.offsets.len() {
                if taken[j] || plan.banks[j] == 0 {
                    continue;
                }
                let delta = (((plan.offsets[j] - plan.offsets[i]) % 360.0) + 360.0) % 360.0;
                if (delta - vee).abs() < 1e-6 {
                    pin.cylinders.push(j);
                    pin.angles.push(pin.angle_deg);
                    taken[j] = true;
                    break;
                }
            }
        }
        pins.push(pin);
    }
    for i in 0..plan.offsets.len() {
        if taken[i] {
            continue;
        }
        pins.push(CrankPin { angle_deg: pin_angle(i), cylinders: vec![i], angles: vec![pin_angle(i)] });
        taken[i] = true;
    }
    pins
}

/// Normalised exhaust layout, accepting the single's and twins' names for it.
pub fn exhaust_layout_of(spec: &EngineSpec) -> ExhaustLayout {
    match spec.exhaust_layout {
        ExhaustLayoutSpec::TwoIntoOne => return ExhaustLayout::Merged,
        ExhaustLayoutSpec::Single | ExhaustLayoutSpec::TwoIntoTwo => return ExhaustLayout::Open,
        _ => {}
    }
    if spec.cylinders == 1 {
        return ExhaustLayout::Open;
    }
    if spec.exhaust_layout == ExhaustLayoutSpec::PerBank && firing_plan(spec).bank_count == 1 {
        return ExhaustLayout::Merged;
    }
    match spec.exhaust_layout {
        ExhaustLayoutSpec::PerBank => ExhaustLayout::PerBank,
        ExhaustLayoutSpec::Merged => ExhaustLayout::Merged,
        _ => ExhaustLayout::Open,
    }
}

/// Which collector each cylinder feeds, or -1 for a cylinder that vents straight out.
pub fn collector_groups(spec: &EngineSpec) -> Vec<i32> {
    let layout = exhaust_layout_of(spec);
    let plan = firing_plan(spec);
    match layout {
        ExhaustLayout::Open => vec![-1; plan.offsets.len()],
        ExhaustLayout::Merged => vec![0; plan.offsets.len()],
        ExhaustLayout::PerBank => plan.banks.iter().map(|&b| b as i32).collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crank_state_matches_the_separate_functions() {
        let spec = EngineSpec::default();
        let g = CrankGeometry::of(&spec);
        for k in 0..720 {
            let deg = k as f64 + 0.25;
            let s = crank_at(&g, deg);
            let v = cylinder_volume(&spec, deg);
            assert!((s.volume - v).abs() <= 1e-15 * v, "volume at {deg}");
        }
    }

    #[test]
    fn spec_round_trips_through_json() {
        let spec = EngineSpec::default();
        let json = serde_json::to_string(&spec).unwrap();
        assert!(json.contains("\"exhaustLayout\":\"open\""));
        let back: EngineSpec = serde_json::from_str(&json).unwrap();
        assert_eq!(back, spec);
    }

    #[test]
    fn merged_replaces_only_what_the_patch_has() {
        let spec = EngineSpec::default();
        let patch = serde_json::json!({ "cylinders": 8, "exhaustLayout": "perBank" });
        let next = spec.merged(&patch).unwrap();
        assert_eq!(next.cylinders, 8);
        assert_eq!(next.exhaust_layout, ExhaustLayoutSpec::PerBank);
        assert_eq!(next.bore, spec.bore);
    }

    #[test]
    fn v8_crossplane_fires_every_90() {
        let spec = EngineSpec { cylinders: 8, v_angle: 90.0, crank_type: CrankType::Crossplane, ..Default::default() };
        let mut offsets = firing_plan(&spec).offsets;
        offsets.sort_by(|a, b| a.partial_cmp(b).unwrap());
        for (k, o) in offsets.iter().enumerate() {
            assert_eq!(*o, 90.0 * k as f64);
        }
    }
}
