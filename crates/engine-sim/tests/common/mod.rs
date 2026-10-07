//! Shared helpers for the physics tests: the web app's presets and the sweeps over them, running checks
//! at once, holding an engine to measure its torque, launching it, building exhausts, and a little
//! spectral analysis.
#![allow(dead_code)]

use engine_sim::EngineSim;
use engine_sim::euler_pipe::EulerPipe;
use engine_sim::spec::{
    EngineConfig, EngineSpec, ExhaustLayout, LaunchConfig, LaunchSnapshot, PipeSegment, SegmentKind, SegmentPartial,
    collector_groups, displacement, exhaust_layout_of, exhaust_port_diameter, make_segment,
};
use serde::Deserialize;
use serde_json::{Value, json};
use std::sync::OnceLock;

pub const FS: f64 = 48000.0;

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Presets {
    pub default_engine: EngineSpec,
    pub default_config: EngineConfig,
    pub default_collector: Vec<PipeSegment>,
    pub engine_presets: Vec<EnginePreset>,
    pub pipe_presets: Vec<PipePreset>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct EnginePreset {
    pub name: String,
    /// The fields the preset sets, over the defaults.
    pub engine: serde_json::Value,
    /// The whole config the app loads for it.
    pub config: EngineConfig,
    /// The car and gearing it launches through: `presetLaunch`.
    pub launch: LaunchConfig,
    pub fitted_exhaust: FittedExhaust,
}

#[derive(Deserialize)]
pub struct FittedExhaust {
    pub pipe: Vec<PipeSegment>,
    pub collector: Vec<PipeSegment>,
}

#[derive(Deserialize)]
pub struct PipePreset {
    pub name: String,
    pub description: String,
    pub segments: Vec<PipeSegment>,
}

/// The web app's presets, from `tests/fixtures/presets.json` (`npm run export:presets` in apps/web).
pub fn presets() -> &'static Presets {
    static P: OnceLock<Presets> = OnceLock::new();
    P.get_or_init(|| {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/presets.json");
        serde_json::from_str(&std::fs::read_to_string(path).expect("presets fixture")).expect("presets parse")
    })
}

/// `defaultConfig()`: the default single on the megaphone.
pub fn default_config() -> EngineConfig {
    presets().default_config.clone()
}

/// The engine preset called `name`.
pub fn engine_preset(name: &str) -> &'static EnginePreset {
    presets().engine_presets.iter().find(|p| p.name == name).unwrap_or_else(|| panic!("no engine preset {name:?}"))
}

/// `PIPE_PRESETS[i].build()`.
pub fn pipe_preset(i: usize) -> Vec<PipeSegment> {
    presets().pipe_presets[i].segments.clone()
}

/// `defaultEngine()`: the default single.
pub fn default_engine() -> EngineSpec {
    presets().default_engine.clone()
}

/// `spec` with the fields in `patch`, a JSON object in the web app's shape, replaced.
pub fn with(spec: &EngineSpec, patch: serde_json::Value) -> EngineSpec {
    spec.merged(&patch).expect("patch applies")
}

/// `actual` within half a unit of `expected` in the `digits`th decimal place.
#[track_caller]
pub fn assert_close(actual: f64, expected: f64, digits: f64) {
    let tol = 10f64.powf(-digits) / 2.0;
    assert!((actual - expected).abs() < tol, "expected {actual} to be within {tol} of {expected}");
}

// --- the sweeps ---

/// The presets the sweeps run on: one naturally aspirated, one turbocharged and one diesel.
pub const SWEPT: [&str; 3] = ["V8, Chevrolet LT6", "Inline four, Toyota 3S-GTE", "Inline six diesel, Cummins 6CT"];

/// The `SWEPT` presets.
pub fn swept_presets() -> Vec<&'static EnginePreset> {
    SWEPT.iter().map(|name| engine_preset(name)).collect()
}

/// `check` on every swept preset, all at once.
pub fn sweep(check: impl Fn(&EnginePreset) + Sync) {
    each(&swept_presets(), |p| check(p));
}

/// `check` on each of `items`, each on its own thread, all at once. A failure on any fails the caller
/// with that failure's message.
pub fn each<T: Sync>(items: &[T], check: impl Fn(&T) + Sync) {
    std::thread::scope(|s| {
        let check = &check;
        let handles: Vec<_> = items.iter().map(|item| s.spawn(move || check(item))).collect();
        for h in handles {
            h.join().unwrap_or_else(|e| std::panic::resume_unwind(e));
        }
    });
}

/// `f` of each of `items`, each on its own thread, all at once, in order.
pub fn par<T: Sync, R: Send, const N: usize>(items: [T; N], f: impl Fn(&T) -> R + Sync) -> [R; N] {
    std::thread::scope(|s| {
        let f = &f;
        items
            .each_ref()
            .map(|item| s.spawn(move || f(item)))
            .map(|h| h.join().unwrap_or_else(|e| std::panic::resume_unwind(e)))
    })
}

// --- held on the dyno ---

/// `name`'s preset held at `rpm` on full throttle, with no cycle-to-cycle scatter, and `over` on top.
pub fn held(name: &str, rpm: f64, over: Value) -> EngineSim {
    let mut cfg = engine_preset(name).config.clone();
    let hold = json!({ "freeRunning": false, "combustionVariability": 0, "throttle": 1, "rpm": rpm });
    cfg.engine = with(&with(&cfg.engine, hold), over);
    EngineSim::new(FS, &cfg)
}

/// The gas's mean torque on the crank, N*m, over the next half second.
pub fn gas_torque(sim: &mut EngineSim) -> f64 {
    mean_torque(sim, |_| 0.0)
}

/// The mean torque at the flywheel, the gas's less friction, N*m, over the next half second.
pub fn brake_torque(sim: &mut EngineSim) -> f64 {
    mean_torque(sim, EngineSim::friction_torque)
}

fn mean_torque(sim: &mut EngineSim, friction: impl Fn(&EngineSim) -> f64) -> f64 {
    let n = FS as usize / 2;
    let mut t = 0.0;
    for _ in 0..n {
        sim.render(1);
        t += sim.snapshot().torque - friction(sim);
    }
    t / n as f64
}

/// `name`'s brake torque, N*m, `held` at each of `points`' speeds with its fields on top, after `settle`
/// seconds there: all at once.
pub fn brake_torques<const N: usize>(name: &str, settle: f64, points: [(f64, Value); N]) -> [f64; N] {
    par(points, |(rpm, over)| {
        let mut sim = held(name, *rpm, over.clone());
        sim.render((settle * FS) as usize);
        brake_torque(&mut sim)
    })
}

/// `name`'s brake torque, N*m, `held` at each of `rpms` as the preset has it, after `settle` seconds.
pub fn brake_torques_at<const N: usize>(name: &str, settle: f64, rpms: [f64; N]) -> [f64; N] {
    brake_torques(name, settle, rpms.map(|rpm| (rpm, json!({}))))
}

/// Power, kW, from `torque` N*m at `rpm`.
pub fn kw(torque: f64, rpm: f64) -> f64 {
    torque * rpm * 2.0 * std::f64::consts::PI / 60.0 / 1000.0
}

/// Power, metric horsepower, from `torque` N*m at `rpm`.
pub fn ps(torque: f64, rpm: f64) -> f64 {
    kw(torque, rpm) * 1000.0 / 735.5
}

/// Power, mechanical horsepower, from `torque` N*m at `rpm`.
pub fn hp(torque: f64, rpm: f64) -> f64 {
    kw(torque, rpm) * 1000.0 / 745.7
}

// --- launched ---

/// `config` idled for half a second, then launched through `launch`: a snapshot every 20 ms while the run
/// lasts, up to three minutes, or until `enough` says the last one has what is wanted.
pub fn launch(
    config: &EngineConfig,
    launch: LaunchConfig,
    enough: impl Fn(&LaunchSnapshot) -> bool,
) -> Vec<LaunchSnapshot> {
    let mut sim = EngineSim::new(FS, config);
    sim.render(FS as usize / 2);
    sim.start_launch(launch);
    let mut out = Vec::new();
    let block = (FS / 50.0) as usize;
    for _ in 0..(180 * 50) {
        sim.render(block);
        match sim.snapshot().launch {
            Some(s) => {
                let done = enough(&s);
                out.push(s);
                if done {
                    break;
                }
            }
            None => break,
        }
    }
    out
}

/// Launches run to the end.
pub fn to_the_end(_: &LaunchSnapshot) -> bool {
    false
}

/// Launches run until 60 mph.
pub fn to_sixty(s: &LaunchSnapshot) -> bool {
    s.zero_to_sixty.is_some()
}

/// `name`'s preset launched through its own car to the end. Each is run once, however many tests read it.
pub fn preset_launch(name: &str) -> &'static [LaunchSnapshot] {
    static RUNS: OnceLock<Vec<OnceLock<Vec<LaunchSnapshot>>>> = OnceLock::new();
    let runs = RUNS.get_or_init(|| presets().engine_presets.iter().map(|_| OnceLock::new()).collect());
    let i = presets()
        .engine_presets
        .iter()
        .position(|p| p.name == name)
        .unwrap_or_else(|| panic!("no engine preset {name:?}"));
    let preset = &presets().engine_presets[i];
    runs[i].get_or_init(|| launch(&preset.config, preset.launch.clone(), to_the_end))
}

// --- exhausts ---

/// A segment of `kind`, `length` long, from `d_in` to `d_out`.
pub fn segment(kind: SegmentKind, length: f64, d_in: f64, d_out: f64) -> PipeSegment {
    make_segment(SegmentPartial {
        kind: Some(kind),
        length: Some(length),
        d_in: Some(d_in),
        d_out: Some(d_out),
        ..Default::default()
    })
}

/// A straight pipe `length` long of diameter `d`.
pub fn pipe(length: f64, d: f64) -> PipeSegment {
    segment(SegmentKind::Pipe, length, d, d)
}

/// The preset's own collector, or `None` where it has none and the app falls back to the default.
///
/// The fixture's configs fill a missing collector in with `defaultCollector()`, so a collector of
/// exactly that geometry is one the preset did not draw.
pub fn preset_collector(preset: &EnginePreset) -> Option<Vec<PipeSegment>> {
    let shape = |s: &[PipeSegment]| -> Vec<(SegmentKind, f64, f64, f64)> {
        s.iter().map(|g| (g.kind, g.length, g.d_in, g.d_out)).collect()
    };
    let own = &preset.config.collector;
    if shape(own) == shape(&presets().default_collector) { None } else { Some(own.clone()) }
}

/// Primaries then collectors: every duct in the system.
pub fn ducts(sim: &EngineSim) -> Vec<&EulerPipe> {
    let sys = sim.pipe_solver();
    sys.primaries().iter().chain(sys.collectors()).collect()
}

/// The web app's `fittedExhaust`: a header runner bored for the valve and, where the layout merges, a
/// constant-velocity collector with a silencer can. `(pipe, collector)`.
pub fn fitted_exhaust(spec: &EngineSpec) -> (Vec<PipeSegment>, Vec<PipeSegment>) {
    let layout = exhaust_layout_of(spec);
    let groups = collector_groups(spec);
    let collector_count = groups.iter().fold(0, |max, &g| i32::max(max, g + 1));
    let per_collector = if collector_count > 0 { spec.cylinders as f64 / collector_count as f64 } else { 1.0 };

    let d_primary = f64::max(0.85 * exhaust_port_diameter(spec), 0.02);
    let primary_length = if layout == ExhaustLayout::Open { 0.75 } else { 0.45 };
    let mut pipe = vec![segment(SegmentKind::Pipe, primary_length, d_primary, d_primary)];
    if layout == ExhaustLayout::Open {
        pipe.push(segment(SegmentKind::Cone, 0.25, d_primary, d_primary * 1.7));
        return (pipe, Vec::new());
    }

    let d_collector = d_primary * per_collector.sqrt() * 0.92;
    let served_disp = displacement(spec) * per_collector;
    let can_dia = f64::min(d_collector * 2.5, 0.2);
    let can_area = (std::f64::consts::PI * can_dia * can_dia) / 4.0;
    let can_length = ((8.0 * served_disp) / can_area).clamp(0.25, 0.6);
    let run_length = f64::max(2.2 - primary_length - can_length - 0.5, 0.35);
    let collector = vec![
        segment(SegmentKind::Cone, 0.16, d_primary * 1.25, d_collector),
        segment(SegmentKind::Pipe, run_length, d_collector, d_collector),
        segment(SegmentKind::Chamber, can_length, d_collector, can_dia),
        segment(SegmentKind::Pipe, 0.5, d_collector, d_collector),
    ];
    (pipe, collector)
}

// --- the sound ---

pub fn rms(buf: &[f32]) -> f64 {
    let s: f64 = buf.iter().map(|&v| v as f64 * v as f64).sum();
    (s / buf.len() as f64).sqrt()
}

/// In-place iterative radix-2 FFT. Lengths must match and be a power of two.
pub fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    assert!(n == im.len() && n.is_power_of_two(), "fft: length must match and be a power of two");
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = (-2.0 * std::f64::consts::PI) / len as f64;
        let (w_re, w_im) = (ang.cos(), ang.sin());
        for i in (0..n).step_by(len) {
            let (mut cur_re, mut cur_im) = (1.0, 0.0);
            for k in 0..len / 2 {
                let (u_re, u_im) = (re[i + k], im[i + k]);
                let (a, b) = (re[i + k + len / 2], im[i + k + len / 2]);
                let v_re = a * cur_re - b * cur_im;
                let v_im = a * cur_im + b * cur_re;
                re[i + k] = u_re + v_re;
                im[i + k] = u_im + v_im;
                re[i + k + len / 2] = u_re - v_re;
                im[i + k + len / 2] = u_im - v_im;
                let next_re = cur_re * w_re - cur_im * w_im;
                cur_im = cur_re * w_im + cur_im * w_re;
                cur_re = next_re;
            }
        }
        len <<= 1;
    }
}

/// Magnitude spectrum of `signal`, zero-padded or truncated to `size`: bins 0..=size/2.
pub fn magnitude_spectrum(signal: &[f32], size: usize) -> Vec<f64> {
    let mut re = vec![0.0; size];
    let mut im = vec![0.0; size];
    for (r, &s) in re.iter_mut().zip(signal) {
        *r = s as f64;
    }
    fft(&mut re, &mut im);
    (0..size / 2 + 1).map(|i| re[i].hypot(im[i])).collect()
}

/// Hann window.
pub fn hann(signal: &[f32]) -> Vec<f32> {
    let n = signal.len();
    signal
        .iter()
        .enumerate()
        .map(|(i, &v)| {
            (v as f64 * 0.5 * (1.0 - ((2.0 * std::f64::consts::PI * i as f64) / (n as f64 - 1.0)).cos())) as f32
        })
        .collect()
}

#[derive(Clone, Copy, Debug)]
pub struct Peak {
    pub hz: f64,
    pub mag: f64,
}

/// Local maxima of a magnitude spectrum between `lo_hz` and `hi_hz`, strongest first, with
/// parabolically interpolated frequencies.
pub fn find_peaks(mag: &[f64], sample_rate: f64, size: usize, lo_hz: f64, hi_hz: f64, rel_threshold: f64) -> Vec<Peak> {
    let bin_hz = sample_rate / size as f64;
    let lo = ((lo_hz / bin_hz).floor() as usize).max(1);
    let hi = ((hi_hz / bin_hz).ceil() as usize).min(mag.len() - 2);
    let mut max_mag = 0.0f64;
    for &m in &mag[lo..=hi] {
        max_mag = max_mag.max(m);
    }
    let mut peaks = Vec::new();
    for i in lo..=hi {
        let m = mag[i];
        if m < max_mag * rel_threshold || m <= mag[i - 1] || m < mag[i + 1] {
            continue;
        }
        let (a, b, c) = (mag[i - 1], m, mag[i + 1]);
        let denom = a - 2.0 * b + c;
        let shift = if denom == 0.0 { 0.0 } else { (0.5 * (a - c)) / denom };
        peaks.push(Peak { hz: (i as f64 + shift) * bin_hz, mag: m });
    }
    peaks.sort_by(|x, y| y.mag.partial_cmp(&x.mag).unwrap());
    peaks
}

/// Energy in a band `width_hz` wide around `hz`.
pub fn band_energy(mag: &[f64], sample_rate: f64, size: usize, hz: f64, width_hz: f64) -> f64 {
    let bin_hz = sample_rate / size as f64;
    let lo = ((hz - width_hz / 2.0) / bin_hz).floor().max(0.0) as usize;
    let hi = (((hz + width_hz / 2.0) / bin_hz).ceil() as usize).min(mag.len() - 1);
    mag[lo..=hi].iter().map(|m| m * m).sum()
}
