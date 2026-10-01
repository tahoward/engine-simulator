//! An engine preset set against a recording of the real engine at the same point: held at one speed,
//! or swept through a dyno pull. `cargo run --release -p engine-sim --example compare -- --help`.
//!
//! Both are reduced to the same two views, each normalised so a recording's gain and distance drop
//! out: the level of every half engine order against the firing order's, and third-octave band levels
//! against their total. Orders are tracked: each frame's are taken at that frame's own speed, so a
//! sweep's frames pool by order rather than by frequency.
//!
//! The recording's speed comes from where the firing order's harmonics line up, near the speed asked
//! for: once for a hold, so a tachometer a few percent out does not misplace every order, and frame by
//! frame for a sweep, following it up from the speed it starts at. The simulation sweeps the same
//! range at the same rate on its dyno, and the two are compared over each band of speed as well as
//! over the whole pull.
//!
//! Without `--reference` it reports the simulation alone; `--wav` writes what it rendered.

use std::collections::{BTreeMap, HashMap};
use std::f64::consts::PI;
use std::process::exit;

use engine_sim::{EngineConfig, EngineSim, LaunchConfig};
use serde::Deserialize;

const FS: f64 = 48000.0;

/// FFT frame for a hold: 0.68 s at 48 kHz, 0.73 Hz bins.
const HOLD_FRAME: usize = 32768;

/// FFT frame for a sweep: 0.17 s at 48 kHz, 5.9 Hz bins. A pull at 500 rpm/s moves under 90 rpm
/// within one.
const SWEEP_FRAME: usize = 8192;

/// Highest frequency an order is reported at, Hz. Above it a recording's speed wobble smears
/// neighbouring half orders into each other.
const ORDER_LIMIT_HZ: f64 = 3000.0;

/// Half orders tracked, up to order 48.
const HALF_ORDERS: usize = 96;

/// Third-octave bands from 25 Hz to 16 kHz, as their nominal centres, Hz. The bands themselves are
/// the exact base-two ones, `1000 * 2^(k/3)`.
const BANDS: [f64; 29] = [
    25.0, 31.5, 40.0, 50.0, 63.0, 80.0, 100.0, 125.0, 160.0, 200.0, 250.0, 315.0, 400.0, 500.0, 630.0, 800.0, 1000.0,
    1250.0, 1600.0, 2000.0, 2500.0, 3150.0, 4000.0, 5000.0, 6300.0, 8000.0, 10000.0, 12500.0, 16000.0,
];

const USAGE: &str = "\
compare: an engine preset against a recording of the real engine.

  --preset NAME       engine preset, matched by substring (required)
  --rpm RPM           a hold: the speed to hold the simulation at, and the recording's nominal speed.
                      A sweep: the recording's speed where the part used starts (required)
  --sweep             the recording is a dyno pull: track its speed and pull the simulation through it
  --bin RPM           a sweep: width of the speed bands it is compared over (default 500)
  --throttle T        a hold: throttle, 0..1 (default 1). A pull is at full throttle
  --seconds S         a hold: seconds of simulation to analyse (default 4)
  --settle S          seconds to run before analysing (default 2)
  --set JSON          engine fields to replace, in the web app's shape, e.g. '{\"exhaustHeight\": 0.5}'
  --listener X,Y,Z    where the ear is, m, in the drawn engine's frame: x across the crank, y up, z along
                      it, rearwards (default: 1.5 m from the tailpipes, 45 degrees off the car's rear axis,
                      1.2 m above the ground)
  --reference FILE    recording to compare against, WAV: PCM 16/24/32 or float, any rate
  --ref-start S       where the part of the recording to use starts, s (default 0)
  --ref-seconds S     how much of it to use, s (default: the rest)
  --ref-rpm-fixed     a hold: take --rpm as the recording's exact speed rather than refining it
  --wav FILE          write the simulation's render, 32-bit float WAV
  --csv FILE          write the tables over the whole run";

fn main() {
    let args = parse_args();
    let get = |k: &str| args.get(k).map(String::as_str);
    let num = |k: &str, default: Option<f64>| -> f64 {
        match get(k) {
            Some(v) => v.parse().unwrap_or_else(|_| fail(&format!("--{k}: not a number: {v}"))),
            None => default.unwrap_or_else(|| fail(&format!("--{k} is required"))),
        }
    };
    let preset_name = get("preset").unwrap_or_else(|| fail("--preset is required"));
    let rpm = num("rpm", None);
    let settle = num("settle", Some(2.0));
    let sweep = args.contains_key("sweep");

    let (name, mut cfg, launch) = find_preset(preset_name);
    if let Some(patch) = get("set") {
        let patch: serde_json::Value =
            serde_json::from_str(patch).unwrap_or_else(|e| fail(&format!("--set: not JSON: {e}")));
        cfg.engine = cfg.engine.merged(&patch).unwrap_or_else(|e| fail(&format!("--set: {e}")));
    }
    if let Some(ear) = get("listener") {
        let xyz: Vec<f64> = ear.split(',').map(|v| v.trim().parse().unwrap_or(f64::NAN)).collect();
        if xyz.len() != 3 || xyz.iter().any(|v| !v.is_finite()) {
            fail(&format!("--listener: not three numbers: {ear}"));
        }
        cfg.listener = Some([xyz[0], xyz[1], xyz[2]]);
    }
    let cylinders = cfg.engine.cylinders.max(1) as f64;
    // Crank orders: a four-stroke fires each cylinder once every two turns.
    let firing_order = cylinders / 2.0;

    let recording = get("reference").map(|path| {
        let (samples, fs) = read_wav(path);
        let start = (num("ref-start", Some(0.0)) * fs) as usize;
        let end = match get("ref-seconds") {
            Some(_) => start + (num("ref-seconds", None) * fs) as usize,
            None => samples.len(),
        };
        if start >= samples.len() || end <= start {
            fail(&format!("{path}: --ref-start/--ref-seconds leave nothing of its {} s", samples.len() as f64 / fs));
        }
        (path.to_string(), samples[start..end.min(samples.len())].to_vec(), fs)
    });

    if sweep {
        let (path, samples, fs) = recording.unwrap_or_else(|| fail("--sweep needs a --reference to follow"));
        let bin = num("bin", Some(500.0));
        compare_sweep(
            &name,
            &cfg,
            &launch,
            settle,
            firing_order,
            bin,
            (&path, &samples, fs),
            rpm,
            get("wav"),
            get("csv"),
        );
        return;
    }

    let throttle = num("throttle", Some(1.0));
    let seconds = num("seconds", Some(4.0));
    cfg.engine.rpm = rpm;
    cfg.engine.throttle = throttle;
    cfg.engine.free_running = false;
    eprintln!("rendering {name} at {rpm} rpm, throttle {throttle:.2}: {settle} s to settle, {seconds} s to analyse");
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render((settle * FS) as usize);
    let render: Vec<f64> = sim.render((seconds * FS) as usize).into_iter().map(f64::from).collect();
    if let Some(path) = get("wav") {
        write_wav(path, &render, FS);
        eprintln!("wrote {path}");
    }
    let mut sim_pool = Pool::default();
    for (_, psd) in frames(&render, HOLD_FRAME) {
        sim_pool.add(&psd, FS, HOLD_FRAME, rpm);
    }
    let sim_view = sim_pool.view(firing_order);

    let reference = recording.map(|(path, used, fs)| {
        let psds: Vec<Vec<f64>> = frames(&used, HOLD_FRAME).into_iter().map(|f| f.1).collect();
        let ref_rpm = if args.contains_key("ref-rpm-fixed") {
            rpm
        } else {
            best_rpm(&mean(&psds), fs / HOLD_FRAME as f64, fs, rpm, 0.07, firing_order)
        };
        let mut pool = Pool::default();
        for psd in &psds {
            pool.add(psd, fs, HOLD_FRAME, ref_rpm);
        }
        (path, fs, used.len() as f64 / fs, ref_rpm, pool.view(firing_order))
    });

    println!("{name} at {rpm} rpm, throttle {throttle:.2}, {seconds} s");
    if let Some((path, fs, secs, ref_rpm, _)) = &reference {
        let moved = (ref_rpm / rpm - 1.0) * 100.0;
        println!("reference {path}: {fs} Hz, {secs:.1} s, {ref_rpm:.0} rpm ({moved:+.1}% from the speed asked for)");
        if moved.abs() > 4.0 {
            println!("  the recording's speed is far from the one asked for: its orders may be misplaced");
        }
    }
    let ref_view = reference.as_ref().map(|r| &r.4);
    report(&sim_view, ref_view, firing_order);
    if let Some(path) = get("csv") {
        write_csv(path, &sim_view, ref_view);
        eprintln!("wrote {path}");
    }
}

/// Track the recording's speed through its pull, pull the simulation through the same range at the same
/// rate, and compare the two band by band and over the whole pull.
#[allow(clippy::too_many_arguments)]
fn compare_sweep(
    name: &str,
    cfg: &EngineConfig,
    launch: &LaunchConfig,
    settle: f64,
    firing_order: f64,
    bin: f64,
    (path, samples, fs): (&str, &[f64], f64),
    seed_rpm: f64,
    wav: Option<&str>,
    csv: Option<&str>,
) {
    let ref_frames = frames(samples, SWEEP_FRAME);
    let hop_s = (SWEEP_FRAME / 2) as f64 / fs;
    let ref_rpms = track_rpm(&ref_frames, fs, seed_rpm, firing_order);
    let (from, to) = (ref_rpms[0], *ref_rpms.last().unwrap());
    let span_s = hop_s * (ref_rpms.len() - 1).max(1) as f64;
    let rate = (to - from) / span_s;
    println!("{name}, dyno pull");
    println!(
        "reference {path}: {fs} Hz, {:.1} s, tracked {from:.0} to {to:.0} rpm, {rate:.0} rpm/s",
        samples.len() as f64 / fs
    );
    let per_second = (1.0 / hop_s).round().max(1.0) as usize;
    let marks: Vec<String> = ref_rpms.iter().step_by(per_second).map(|r| format!("{r:.0}")).collect();
    println!("  speed each second: {}", marks.join(" "));
    if rate <= 0.0 {
        fail("the recording's speed does not rise over the part used: check --rpm, --ref-start and --ref-seconds");
    }

    eprintln!("pulling {name} from {from:.0} to {to:.0} rpm at {rate:.0} rpm/s");
    let (render, block_rpms) = pull(cfg, launch, settle, from, to, rate);
    if let Some(path) = wav {
        write_wav(path, &render, FS);
        eprintln!("wrote {path}");
    }
    let sim_rpm_at = |start: usize| -> f64 {
        let a = start / PULL_BLOCK;
        let b = ((start + SWEEP_FRAME) / PULL_BLOCK).min(block_rpms.len());
        let s = &block_rpms[a.min(b.saturating_sub(1))..b.max(a + 1).min(block_rpms.len())];
        s.iter().sum::<f64>() / s.len().max(1) as f64
    };
    let sim_frames: Vec<(f64, Vec<f64>)> =
        frames(&render, SWEEP_FRAME).into_iter().map(|(start, psd)| (sim_rpm_at(start), psd)).collect();
    if let (Some(first), Some(last)) = (sim_frames.first(), sim_frames.last()) {
        println!("simulation: dyno pull {:.0} to {:.0} rpm, {:.1} s", first.0, last.0, render.len() as f64 / FS);
    }

    let band_of = |rpm: f64| (rpm / bin).floor() as i64;
    let (mut sim_bins, mut ref_bins) = (BTreeMap::<i64, Pool>::new(), BTreeMap::<i64, Pool>::new());
    let (mut sim_all, mut ref_all) = (Pool::default(), Pool::default());
    for (rpm, psd) in &sim_frames {
        sim_bins.entry(band_of(*rpm)).or_default().add(psd, FS, SWEEP_FRAME, *rpm);
        sim_all.add(psd, FS, SWEEP_FRAME, *rpm);
    }
    for (rpm, (_, psd)) in ref_rpms.iter().zip(&ref_frames) {
        ref_bins.entry(band_of(*rpm)).or_default().add(psd, fs, SWEEP_FRAME, *rpm);
        ref_all.add(psd, fs, SWEEP_FRAME, *rpm);
    }

    // Bands of speed both cover with a few frames each.
    let shared: Vec<(i64, View, View)> = sim_bins
        .iter()
        .filter_map(|(k, s)| {
            let r = ref_bins.get(k)?;
            (s.frames >= 3 && r.frames >= 3).then(|| (*k, s.view(firing_order), r.view(firing_order)))
        })
        .collect();
    if shared.is_empty() {
        fail("the simulation's pull and the recording share no band of speed");
    }
    let sim_mean = shared.iter().map(|s| s.1.level).sum::<f64>() / shared.len() as f64;
    let ref_mean = shared.iter().map(|s| s.2.level).sum::<f64>() / shared.len() as f64;

    println!("\nby speed: level against the pull's mean, and how far apart the spectra are");
    println!(
        "{:>11} {:>9} {:>13} {:>7} {:>7} {:>13} {:>13}",
        "rpm", "frames", "level dB", "orders", "bands", "tilt dB/oct", "centroid Hz"
    );
    println!(
        "{:>11} {:>4} {:>4} {:>6} {:>6} {:>7} {:>7} {:>6} {:>6} {:>6} {:>6}",
        "", "sim", "ref", "sim", "ref", "rms", "rms", "sim", "ref", "sim", "ref"
    );
    for (k, s, r) in &shared {
        let (orders, _) = stats(&order_diffs(s, r));
        let (bands, _) = stats(&band_diffs(s, r));
        println!(
            "{:>5}-{:<5} {:>4} {:>4} {:>+6.1} {:>+6.1} {orders:>7.1} {bands:>7.1} {:>+6.1} {:>+6.1} {:>6.0} {:>6.0}",
            *k as f64 * bin,
            (*k + 1) as f64 * bin,
            sim_bins[k].frames,
            ref_bins[k].frames,
            s.level - sim_mean,
            r.level - ref_mean,
            s.tilt(),
            r.tilt(),
            s.centroid,
            r.centroid,
        );
    }

    println!("\nover the whole pull");
    let (sim_view, ref_view) = (sim_all.view(firing_order), ref_all.view(firing_order));
    report(&sim_view, Some(&ref_view), firing_order);
    if let Some(path) = csv {
        write_csv(path, &sim_view, Some(&ref_view));
        eprintln!("wrote {path}");
    }
}

/// Samples per block the simulation's speed is read over during a pull.
const PULL_BLOCK: usize = 256;

/// The simulation pulled on its dyno from `from` to `to` rpm at `rate` rpm/s: the pull's audio, and
/// the engine's speed over each `PULL_BLOCK` of it.
fn pull(cfg: &EngineConfig, launch: &LaunchConfig, settle: f64, from: f64, to: f64, rate: f64) -> (Vec<f64>, Vec<f64>) {
    let mut sim = EngineSim::new(FS, cfg);
    sim.render((settle * FS) as usize);
    sim.start_launch(LaunchConfig {
        dyno: true,
        ratios: vec![1.0],
        final_drive: 1.0,
        launch_rpm: from,
        shift_rpm: to,
        sweep_rate: rate,
        ..launch.clone()
    });
    let (mut audio, mut rpms) = (Vec::new(), Vec::new());
    let mut buf = [0.0; PULL_BLOCK];
    // A minute is far longer than any pull: the hold, then the sweep, end it well before.
    for _ in 0..(60.0 * FS) as usize / PULL_BLOCK {
        // The crank's own speed averaged over the block: the readout is smoothed, and lags a sweep.
        let mut rpm = 0.0;
        for b in buf.iter_mut() {
            *b = sim.tick();
            rpm += sim.rpm_instant();
        }
        let Some(state) = sim.snapshot().launch else { break };
        if state.finished {
            break;
        }
        if state.phase == "pull" {
            audio.extend(buf);
            rpms.push(rpm / PULL_BLOCK as f64);
        }
    }
    if audio.len() < SWEEP_FRAME {
        fail("the simulation's pull ended before it began: is the recording's range inside the engine's?");
    }
    (audio, rpms)
}

/// One sound reduced to what is compared.
struct View {
    /// Each half order measured: order, Hz at the mean speed, dB against the firing order.
    orders: Vec<(f64, f64, f64)>,
    /// Each third-octave band below the Nyquist frequency: centre, dB against their total.
    bands: Vec<(f64, f64)>,
    /// Energy-weighted mean frequency, Hz.
    centroid: f64,
    /// Total band power, dB on the sound's own scale.
    level: f64,
}

impl View {
    /// Slope of the band levels against octaves from 100 Hz to 8 kHz, dB/octave.
    fn tilt(&self) -> f64 {
        let pts: Vec<(f64, f64)> =
            self.bands.iter().filter(|b| (100.0..=8000.0).contains(&b.0)).map(|b| (b.0.log2(), b.1)).collect();
        let n = pts.len() as f64;
        let mx = pts.iter().map(|p| p.0).sum::<f64>() / n;
        let my = pts.iter().map(|p| p.1).sum::<f64>() / n;
        let sxy: f64 = pts.iter().map(|p| (p.0 - mx) * (p.1 - my)).sum();
        let sxx: f64 = pts.iter().map(|p| (p.0 - mx) * (p.0 - mx)).sum();
        sxy / sxx
    }
}

/// Order and band powers pooled over frames, each frame's orders taken at its own speed.
struct Pool {
    orders: Vec<(f64, usize)>,
    bands: Vec<(f64, usize)>,
    centroid: (f64, f64),
    rpm: f64,
    frames: usize,
}

impl Default for Pool {
    fn default() -> Pool {
        Pool {
            orders: vec![(0.0, 0); HALF_ORDERS],
            bands: vec![(0.0, 0); BANDS.len()],
            centroid: (0.0, 0.0),
            rpm: 0.0,
            frames: 0,
        }
    }
}

impl Pool {
    /// Add the power spectrum `psd` of one `size`-sample frame at `fs`, with the engine at `rpm`.
    fn add(&mut self, psd: &[f64], fs: f64, size: usize, rpm: f64) {
        let bin_hz = fs / size as f64;
        let crank_hz = rpm / 60.0;
        for (i, slot) in self.orders.iter_mut().enumerate() {
            let hz = (i + 1) as f64 * 0.5 * crank_hz;
            if hz > ORDER_LIMIT_HZ.min(0.45 * fs) {
                break;
            }
            let half_width = (0.01 * hz).max(2.5 * bin_hz).min(0.2 * crank_hz);
            slot.0 += power_between(psd, bin_hz, hz - half_width, hz + half_width);
            slot.1 += 1;
        }
        let edge = 2f64.powf(1.0 / 6.0);
        for (i, slot) in self.bands.iter_mut().enumerate() {
            let c = 1000.0 * 2f64.powf((i as f64 - 16.0) / 3.0);
            if c * edge < 0.5 * fs {
                slot.0 += power_between(psd, bin_hz, c / edge, c * edge);
                slot.1 += 1;
            }
        }
        for (i, p) in psd.iter().enumerate().skip((20.0 / bin_hz) as usize) {
            self.centroid.0 += i as f64 * bin_hz * p;
            self.centroid.1 += p;
        }
        self.rpm += rpm;
        self.frames += 1;
    }

    fn view(&self, firing_order: f64) -> View {
        let rpm = self.rpm / self.frames.max(1) as f64;
        let mean = |s: &(f64, usize)| s.0 / s.1.max(1) as f64;
        let measured = |s: &&(f64, usize)| s.1 * 2 >= self.frames.max(1);
        let firing_index = ((firing_order * 2.0) as usize).saturating_sub(1);
        let firing = mean(&self.orders[firing_index.min(HALF_ORDERS - 1)]).max(1e-30);
        let orders = (0..HALF_ORDERS)
            .filter(|&i| measured(&&self.orders[i]))
            .map(|i| {
                let order = (i + 1) as f64 * 0.5;
                (order, order * rpm / 60.0, db(mean(&self.orders[i]) / firing))
            })
            .collect();
        let total: f64 = self.bands.iter().filter(|s| s.1 > 0).map(mean).sum::<f64>().max(1e-30);
        let bands = BANDS
            .iter()
            .zip(&self.bands)
            .filter(|(_, s)| s.1 > 0)
            .map(|(&nominal, s)| (nominal, db(mean(s) / total)))
            .collect();
        View { orders, bands, centroid: self.centroid.0 / self.centroid.1.max(1e-30), level: db(total) }
    }
}

fn order_diffs(sim: &View, reference: &View) -> Vec<f64> {
    sim.orders.iter().filter_map(|&(o, _, s)| reference.orders.iter().find(|r| r.0 == o).map(|r| s - r.2)).collect()
}

fn band_diffs(sim: &View, reference: &View) -> Vec<f64> {
    sim.bands.iter().filter_map(|&(hz, s)| reference.bands.iter().find(|r| r.0 == hz).map(|r| s - r.1)).collect()
}

fn report(sim: &View, reference: Option<&View>, firing_order: f64) {
    println!("\nengine orders, dB against the firing order ({firing_order})");
    println!("{:>6} {:>8} {:>7} {:>7} {:>7}", "order", "Hz", "sim", "ref", "diff");
    for &(order, hz, s) in &sim.orders {
        let r = reference.and_then(|v| v.orders.iter().find(|o| o.0 == order)).map(|o| o.2);
        println!("{order:>6.1} {hz:>8.1} {s:>7.1} {:>7} {:>7}", opt(r), opt(r.map(|r| s - r)));
    }

    println!("\nthird-octave bands, dB against their total");
    println!("{:>8} {:>7} {:>7} {:>7}", "Hz", "sim", "ref", "diff");
    for &(hz, s) in &sim.bands {
        let r = reference.and_then(|v| v.bands.iter().find(|b| b.0 == hz)).map(|b| b.1);
        println!("{hz:>8} {s:>7.1} {:>7} {:>7}", opt(r), opt(r.map(|r| s - r)));
    }

    println!("\nsummary");
    match reference {
        Some(r) => {
            let (od, bd) = (order_diffs(sim, r), band_diffs(sim, r));
            let (orms, omean) = stats(&od);
            let (brms, bmean) = stats(&bd);
            println!("  orders    rms diff {orms:.1} dB, mean |diff| {omean:.1} dB, over {}", od.len());
            println!("  bands     rms diff {brms:.1} dB, mean |diff| {bmean:.1} dB, over {}", bd.len());
            println!("  tilt      sim {:+.1} dB/octave, ref {:+.1}", sim.tilt(), r.tilt());
            println!("  centroid  sim {:.0} Hz, ref {:.0}", sim.centroid, r.centroid);
        }
        None => {
            println!("  tilt      {:+.1} dB/octave", sim.tilt());
            println!("  centroid  {:.0} Hz", sim.centroid);
        }
    }
}

fn write_csv(path: &str, sim: &View, reference: Option<&View>) {
    let mut out = String::from("table,order,hz,sim_db,ref_db\n");
    for &(order, hz, s) in &sim.orders {
        let r = reference
            .and_then(|v| v.orders.iter().find(|o| o.0 == order))
            .map_or(String::new(), |o| format!("{:.2}", o.2));
        out += &format!("order,{order},{hz:.2},{s:.2},{r}\n");
    }
    for &(hz, s) in &sim.bands {
        let r =
            reference.and_then(|v| v.bands.iter().find(|b| b.0 == hz)).map_or(String::new(), |b| format!("{:.2}", b.1));
        out += &format!("band,,{hz},{s:.2},{r}\n");
    }
    std::fs::write(path, out).unwrap_or_else(|e| fail(&format!("{path}: {e}")));
}

/// The speed within `span` of `rpm`, as a fraction, at which the firing order's first eight harmonics,
/// summed in log power, are strongest. Narrower than an octave, so it cannot lock onto twice the speed.
fn best_rpm(psd: &[f64], bin_hz: f64, fs: f64, rpm: f64, span: f64, firing_order: f64) -> f64 {
    let score = |r: f64| -> f64 {
        (1..=8)
            .map(|k| {
                let hz = k as f64 * firing_order * r / 60.0;
                if hz >= 0.45 * fs { 0.0 } else { power_between(psd, bin_hz, hz - bin_hz, hz + bin_hz).max(1e-30).ln() }
            })
            .sum()
    };
    let mut best = (rpm, f64::NEG_INFINITY);
    let mut r = rpm * (1.0 - span);
    while r <= rpm * (1.0 + span) {
        let s = score(r);
        if s > best.1 {
            best = (r, s);
        }
        r *= 1.0002;
    }
    best.0
}

/// The speed through a sweep, one per frame: found within 7% of `seed` in the first, then within 3% of
/// the frame before, and median-filtered over five frames against single frames that lose it.
fn track_rpm(frames: &[(usize, Vec<f64>)], fs: f64, seed: f64, firing_order: f64) -> Vec<f64> {
    let bin_hz = fs / SWEEP_FRAME as f64;
    let mut raw = Vec::with_capacity(frames.len());
    let mut r = seed;
    for (i, (_, psd)) in frames.iter().enumerate() {
        r = best_rpm(psd, bin_hz, fs, r, if i == 0 { 0.07 } else { 0.03 }, firing_order);
        raw.push(r);
    }
    (0..raw.len())
        .map(|i| {
            let mut w: Vec<f64> = raw[i.saturating_sub(2)..(i + 3).min(raw.len())].to_vec();
            w.sort_by(f64::total_cmp);
            w[w.len() / 2]
        })
        .collect()
}

/// Power spectra of Hann frames of `size` overlapping by half, bins 0..=size/2, with where each starts.
/// A signal shorter than one frame is zero-padded into one.
fn frames(signal: &[f64], size: usize) -> Vec<(usize, Vec<f64>)> {
    let window: Vec<f64> = (0..size).map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / (size - 1) as f64).cos())).collect();
    let mut out = Vec::new();
    let mut start = 0;
    loop {
        let mut re = vec![0.0; size];
        let mut im = vec![0.0; size];
        for (i, r) in re.iter_mut().enumerate() {
            *r = signal.get(start + i).copied().unwrap_or(0.0) * window[i];
        }
        fft(&mut re, &mut im);
        out.push((start, (0..=size / 2).map(|k| re[k] * re[k] + im[k] * im[k]).collect()));
        start += size / 2;
        if start + size > signal.len() {
            break;
        }
    }
    out
}

fn mean(psds: &[Vec<f64>]) -> Vec<f64> {
    let mut out = vec![0.0; psds[0].len()];
    for psd in psds {
        out.iter_mut().zip(psd).for_each(|(o, p)| *o += p / psds.len() as f64);
    }
    out
}

/// Summed power of the bins centred in `lo..=hi` Hz, or of the nearest bin where none is.
fn power_between(psd: &[f64], bin_hz: f64, lo: f64, hi: f64) -> f64 {
    let a = ((lo / bin_hz).ceil().max(1.0) as usize).min(psd.len() - 1);
    let b = ((hi / bin_hz).floor() as usize).min(psd.len() - 1);
    if b < a {
        return psd[(((lo + hi) / 2.0 / bin_hz).round() as usize).min(psd.len() - 1)];
    }
    psd[a..=b].iter().sum()
}

/// In-place iterative radix-2 FFT.
fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
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
        let ang = -2.0 * PI / len as f64;
        let (w_re, w_im) = (ang.cos(), ang.sin());
        for i in (0..n).step_by(len) {
            let (mut c_re, mut c_im) = (1.0, 0.0);
            for k in 0..len / 2 {
                let (u_re, u_im) = (re[i + k], im[i + k]);
                let (a, b) = (re[i + k + len / 2], im[i + k + len / 2]);
                let v_re = a * c_re - b * c_im;
                let v_im = a * c_im + b * c_re;
                re[i + k] = u_re + v_re;
                im[i + k] = u_im + v_im;
                re[i + k + len / 2] = u_re - v_re;
                im[i + k + len / 2] = u_im - v_im;
                let next = c_re * w_re - c_im * w_im;
                c_im = c_re * w_im + c_im * w_re;
                c_re = next;
            }
        }
        len <<= 1;
    }
}

/// A WAV file's samples mixed to mono, and its sample rate.
fn read_wav(path: &str) -> (Vec<f64>, f64) {
    let bytes = std::fs::read(path).unwrap_or_else(|e| fail(&format!("{path}: {e}")));
    let bad = |why: &str| -> ! { fail(&format!("{path}: {why}")) };
    if bytes.len() < 12 || &bytes[0..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
        bad("not a WAV file (convert other formats first, e.g. ffmpeg -i in.m4a out.wav)");
    }
    let u16_at = |i: usize| u16::from_le_bytes([bytes[i], bytes[i + 1]]);
    let u32_at = |i: usize| u32::from_le_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]]);
    let mut fmt = None;
    let mut data = None;
    let mut at = 12;
    while at + 8 <= bytes.len() {
        let size = u32_at(at + 4) as usize;
        let body = at + 8;
        let end = (body + size).min(bytes.len());
        match &bytes[at..at + 4] {
            b"fmt " if size >= 16 => {
                let mut tag = u16_at(body);
                // WAVE_FORMAT_EXTENSIBLE: the real format leads the sub-format GUID.
                if tag == 0xFFFE && size >= 26 {
                    tag = u16_at(body + 24);
                }
                fmt = Some((tag, u16_at(body + 2) as usize, u32_at(body + 4) as f64, u16_at(body + 14) as usize));
            }
            b"data" => data = Some(&bytes[body..end]),
            _ => {}
        }
        at = body + size + (size & 1);
    }
    let (tag, channels, fs, bits) = fmt.unwrap_or_else(|| bad("no fmt chunk"));
    let data = data.unwrap_or_else(|| bad("no data chunk"));
    let width = bits / 8;
    let sample = |s: &[u8]| -> f64 {
        match (tag, bits) {
            (1, 16) => i16::from_le_bytes([s[0], s[1]]) as f64 / 32768.0,
            (1, 24) => (i32::from_le_bytes([0, s[0], s[1], s[2]]) >> 8) as f64 / 8388608.0,
            (1, 32) => i32::from_le_bytes([s[0], s[1], s[2], s[3]]) as f64 / 2147483648.0,
            (3, 32) => f32::from_le_bytes([s[0], s[1], s[2], s[3]]) as f64,
            (3, 64) => f64::from_le_bytes(s[..8].try_into().unwrap()),
            _ => bad(&format!("unsupported format {tag}, {bits} bits")),
        }
    };
    if channels == 0 || width == 0 {
        bad("no channels");
    }
    let frame = channels * width;
    let out =
        data.chunks_exact(frame).map(|f| f.chunks_exact(width).map(sample).sum::<f64>() / channels as f64).collect();
    (out, fs)
}

fn write_wav(path: &str, samples: &[f64], fs: f64) {
    let data_len = (samples.len() * 4) as u32;
    let mut out = Vec::with_capacity(44 + data_len as usize);
    out.extend(b"RIFF");
    out.extend((36 + data_len).to_le_bytes());
    out.extend(b"WAVEfmt ");
    out.extend(16u32.to_le_bytes());
    out.extend(3u16.to_le_bytes());
    out.extend(1u16.to_le_bytes());
    out.extend((fs as u32).to_le_bytes());
    out.extend((fs as u32 * 4).to_le_bytes());
    out.extend(4u16.to_le_bytes());
    out.extend(32u16.to_le_bytes());
    out.extend(b"data");
    out.extend(data_len.to_le_bytes());
    for &s in samples {
        out.extend((s as f32).to_le_bytes());
    }
    std::fs::write(path, out).unwrap_or_else(|e| fail(&format!("{path}: {e}")));
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Fixture {
    engine_presets: Vec<Preset>,
}

#[derive(Deserialize)]
struct Preset {
    name: String,
    config: EngineConfig,
    /// The car and gearing it launches through, whose dyno settings a pull starts from.
    launch: LaunchConfig,
}

/// The one preset whose name contains `name`, case-insensitively.
fn find_preset(name: &str) -> (String, EngineConfig, LaunchConfig) {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/presets.json");
    let fixture: Fixture = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let all: Vec<String> = fixture.engine_presets.iter().map(|p| p.name.clone()).collect();
    let lower = name.to_lowercase();
    let hits: Vec<Preset> =
        fixture.engine_presets.into_iter().filter(|p| p.name.to_lowercase().contains(&lower)).collect();
    match hits.len() {
        1 => {
            let p = hits.into_iter().next().unwrap();
            (p.name, p.config, p.launch)
        }
        0 => fail(&format!("no preset matches {name:?}; presets are:\n  {}", all.join("\n  "))),
        _ => {
            let names: Vec<&str> = hits.iter().map(|p| p.name.as_str()).collect();
            fail(&format!("{name:?} matches several presets:\n  {}", names.join("\n  ")))
        }
    }
}

/// `--key value` pairs, and `--flag` with no value.
fn parse_args() -> HashMap<String, String> {
    let mut out = HashMap::new();
    let mut it = std::env::args().skip(1).peekable();
    while let Some(a) = it.next() {
        if a == "--help" || a == "-h" {
            println!("{USAGE}");
            exit(0);
        }
        let key = a.strip_prefix("--").unwrap_or_else(|| fail(&format!("unexpected argument {a:?}\n\n{USAGE}")));
        let value = match it.peek() {
            Some(v) if !v.starts_with("--") => it.next().unwrap(),
            _ => String::new(),
        };
        out.insert(key.to_string(), value);
    }
    out
}

fn stats(diffs: &[f64]) -> (f64, f64) {
    let n = diffs.len().max(1) as f64;
    ((diffs.iter().map(|d| d * d).sum::<f64>() / n).sqrt(), diffs.iter().map(|d| d.abs()).sum::<f64>() / n)
}

fn db(ratio: f64) -> f64 {
    10.0 * ratio.max(1e-30).log10()
}

fn opt(v: Option<f64>) -> String {
    v.map_or(String::new(), |v| format!("{v:.1}"))
}

fn fail(msg: &str) -> ! {
    eprintln!("{msg}");
    exit(2)
}
