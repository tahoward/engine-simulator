//! End-to-end tests on the assembled simulation. These check the claims a listener would actually
//! notice: that the note tracks rpm, that it is a four-stroke, that the exhaust the user builds
//! changes what they hear, and that nothing blows up.

mod common;

use common::{FS, band_energy, find_peaks, hann, magnitude_spectrum, rms};
use engine_sim::EngineSim;
use engine_sim::spec::{PipeSegment, SegmentKind, SegmentPartial, make_segment};
use serde_json::json;

const FFT_SIZE: usize = 65536;
const FS_N: usize = FS as usize;

/// The default single with `overrides` over its engine, on pipe preset `pipe_index`, with the
/// startup transients washed out of the pipe.
fn run(overrides: serde_json::Value, pipe_index: usize) -> EngineSim {
    let mut cfg = common::default_config();
    cfg.engine = common::with(&cfg.engine, overrides);
    cfg.pipe = common::pipe_preset(pipe_index);
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS_N);
    sim
}

fn spectrum_of(sim: &mut EngineSim) -> Vec<f64> {
    magnitude_spectrum(&hann(&sim.render(FFT_SIZE)), FFT_SIZE)
}

fn segment(kind: SegmentKind, length: f64, d_in: f64, d_out: Option<f64>) -> PipeSegment {
    make_segment(SegmentPartial {
        kind: Some(kind),
        length: Some(length),
        d_in: Some(d_in),
        d_out,
        ..Default::default()
    })
}

// --- firing frequency ---

/// A four-stroke single fires once per two revolutions, so f = rpm / 120: the harmonic comb sits
/// on multiples of it.
fn check_harmonic_comb(rpm: f64) {
    let f0 = rpm / 120.0;
    let mag = spectrum_of(&mut run(json!({ "rpm": rpm }), 1));

    // Compare energy *at* each harmonic with energy *between* harmonics, rather than checking peak
    // positions. Cycle-to-cycle combustion scatter and the within-cycle crank speed ripple put real
    // amplitude and frequency sidebands around every harmonic, so picked peaks wander by a fraction
    // of a percent — correctly. The comb itself is the claim worth testing, and it is unaffected.
    for order in 1..=8 {
        let order = order as f64;
        let on = band_energy(&mag, FS, FFT_SIZE, f0 * order, f0 * 0.3);
        let between = band_energy(&mag, FS, FFT_SIZE, f0 * (order + 0.5), f0 * 0.3);
        assert!(on > between, "order {order} ({:.1} Hz) is not above the gap beside it: {on} <= {between}", f0 * order);
    }

    // The fundamental itself must be present, not merely implied by harmonics.
    let at_f0 = band_energy(&mag, FS, FFT_SIZE, f0, 3.0);
    let off_f0 = band_energy(&mag, FS, FFT_SIZE, f0 * 1.5, 3.0);
    assert!(at_f0 > off_f0 * 4.0, "fundamental {at_f0} vs off {off_f0}");

    // And it must be in the right *place* — within a hair of rpm/120 in absolute terms, which
    // peak-picking still resolves well.
    let near = find_peaks(&mag, FS, FFT_SIZE, f0 * 0.5, f0 * 1.5, 0.2)[0];
    assert!((near.hz - f0).abs() < (1.5f64).max(f0 * 0.03), "fundamental at {} Hz, expected {f0}", near.hz);
}

/// At 1800 rpm the harmonic comb sits on multiples of 15.00 Hz.
#[test]
fn firing_frequency_at_1800_rpm() {
    check_harmonic_comb(1800.0);
}

/// At 3200 rpm the harmonic comb sits on multiples of 26.67 Hz.
#[test]
fn firing_frequency_at_3200_rpm() {
    check_harmonic_comb(3200.0);
}

/// At 4800 rpm the harmonic comb sits on multiples of 40.00 Hz.
#[test]
fn firing_frequency_at_4800_rpm() {
    check_harmonic_comb(4800.0);
}

/// Has no half-order component, confirming a four-stroke cycle.
#[test]
fn has_no_half_order_component_confirming_a_four_stroke_cycle() {
    let rpm = 3200.0;
    let mag = spectrum_of(&mut run(json!({ "rpm": rpm }), 1));
    let firing = band_energy(&mag, FS, FFT_SIZE, rpm / 120.0, 3.0);
    // rpm/60 would be a two-stroke firing every revolution; rpm/240 would be a spurious
    // sub-harmonic from the cycle bookkeeping drifting.
    let half = band_energy(&mag, FS, FFT_SIZE, rpm / 240.0, 3.0);
    assert!(firing > half * 20.0, "firing {firing} vs half order {half}");
}

/// Doubling rpm doubles the firing frequency.
#[test]
fn doubling_rpm_doubles_the_firing_frequency() {
    // Locate the fundamental specifically, by searching a window centred on where it is
    // predicted. Taking "the strongest peak in a wide range" instead is fragile: which harmonic
    // happens to win depends on where the pipe resonance falls, so it can silently compare order 1
    // at one speed against order 2 at the other.
    let fundamental = |rpm: f64| {
        let f0 = rpm / 120.0;
        let mag = spectrum_of(&mut run(json!({ "rpm": rpm }), 1));
        find_peaks(&mag, FS, FFT_SIZE, f0 * 0.6, f0 * 1.4, 0.2)[0].hz
    };
    let a = fundamental(2000.0);
    let b = fundamental(4000.0);
    assert!(b / a > 1.9 && b / a < 2.1, "ratio {}", b / a);
}

// --- the exhaust the user builds changes the sound ---

/// A muffler is quieter than an open header, by about what theory allows.
#[test]
fn a_muffler_is_quieter_than_an_open_header_by_about_what_theory_allows() {
    let open = rms(&run(json!({}), 0).render(FFT_SIZE));
    let muffled = rms(&run(json!({}), 3).render(FFT_SIZE));
    let db = 20.0 * (open / muffled).log10();

    // Around 8 dB, and that is the right order for a *single* expansion chamber. Its transmission
    // loss is 10*log10(1 + 0.25*(m - 1/m)^2 * sin^2(kL)); with an area ratio of 9.6 the peak is
    // about 13 dB near 330 Hz, but it falls to exactly zero at c/2L (~660 Hz) and every multiple,
    // so broadband attenuation is far lower than the peak. Real single-chamber boxes behave the
    // same way, which is why silencers use several chambers of different lengths.
    //
    // The floor depends on the discretised can keeping its drawn volume; see `limit_area_ratio`.
    // Taking the ramps out of the body would drop it to about 6.4 dB.
    assert!(db > 7.2 && db < 14.0, "muffler attenuation {db} dB");
}

// A sharper test — that attenuation peaks in the chamber's tuned band and falls to nothing at
// c/2L — is deliberately not asserted here. Placing those bands requires the chamber's *body*
// length and the local gas temperature, and picking them from an approximation would mean tuning
// the bands until the test passed rather than measuring anything.

/// Every preset produces a distinct spectrum.
#[test]
fn every_preset_produces_a_distinct_spectrum() {
    let count = common::presets().pipe_presets.len();
    // Compare normalised spectra so this measures tonal difference, not just level.
    let norm: Vec<Vec<f64>> = (0..count)
        .map(|i| {
            let s = spectrum_of(&mut run(json!({}), i));
            let e: f64 = s.iter().map(|v| v * v).sum();
            let k = 1.0 / e.sqrt();
            s.iter().map(|v| v * k).collect()
        })
        .collect();
    for i in 0..norm.len() {
        for j in i + 1..norm.len() {
            let dot: f64 = norm[i].iter().zip(&norm[j]).map(|(a, b)| a * b).sum();
            // Cosine similarity of 1.0 would mean the geometry had no effect at all.
            assert!(dot < 0.995, "presets {i} and {j} are spectrally identical: {dot}");
        }
    }
}

/// A longer primary pipe lowers the exhaust resonance.
#[test]
fn a_longer_primary_pipe_lowers_the_exhaust_resonance() {
    let resonance_of = |length: f64| {
        let mut cfg = common::default_config();
        cfg.engine.rpm = 3200.0;
        // Deliberately quiet mechanical sources so the measurement sees only the pipe.
        cfg.engine.mech_noise = 0.0;
        cfg.engine.throat_noise = 0.0;
        cfg.pipe = vec![segment(SegmentKind::Pipe, length, 0.042, None)];
        let mut sim = EngineSim::new(FS, &cfg);
        sim.render(FS_N);
        let mag = magnitude_spectrum(&hann(&sim.render(FFT_SIZE)), FFT_SIZE);

        // Strongest radiated component in the band the pipe resonance lives in.
        //
        // Not an energy-weighted centroid over 40-1200 Hz, which looks more robust than
        // peak-picking against a harmonic comb but is not: with a finite intake plenum the low
        // orders carry realistic weight, so a centroid is dominated by the firing comb rather than
        // by the resonance and goes non-monotonic with length, even while the duct's actual
        // quarter-wave frequency tracks c/4L.
        let bin_hz = FS / FFT_SIZE as f64;
        let mut best = 0.0;
        let mut best_hz = 0.0;
        for i in (40.0 / bin_hz).floor() as usize..(1200.0 / bin_hz).floor() as usize {
            if mag[i] > best {
                best = mag[i];
                best_hz = i as f64 * bin_hz;
            }
        }
        (best_hz, sim.duct_quarter_wave_hz())
    };

    let (short_peak, short_qw) = resonance_of(0.35);
    let (long_peak, long_qw) = resonance_of(1.4);
    // The duct's own resonance, integrated over the solved temperature field. Quadrupling the
    // length must quarter it, give or take the cell quantisation and the fact that the longer duct
    // runs cooler at its far end.
    assert!(long_qw < short_qw * 0.35, "quarter wave {long_qw} vs {short_qw}");
    // And it must reach the output, not just the solver.
    assert!(long_peak < short_peak, "peak {long_peak} vs {short_peak}");
}

/// Editing the pipe mid-run stays finite and recovers level.
#[test]
fn editing_the_pipe_mid_run_stays_finite_and_recovers_level() {
    let mut sim = run(json!({}), 1);
    sim.render(2000);
    sim.set_pipe(&common::pipe_preset(2), None);
    let buf = sim.render(FS_N);
    assert!(buf.iter().all(|v| v.is_finite()));
    // The rebuild ramp is ~8 ms, so by the end of a second it must be audible again.
    assert!(rms(&buf[FS_N / 2..]) > 1e-4);
}

// --- output conditioning ---

/// Is audible but never clips, across rpm and every preset.
#[test]
fn is_audible_but_never_clips_across_rpm_and_every_preset() {
    for p in 0..common::presets().pipe_presets.len() {
        for rpm in [900.0, 3200.0, 8000.0] {
            let buf = run(json!({ "rpm": rpm }), p).render(FS_N / 2);
            let mut peak = 0.0f32;
            for &v in &buf {
                assert!(v.is_finite(), "preset {p} at {rpm} rpm produced a non-finite sample");
                peak = peak.max(v.abs());
            }
            assert!(peak > 1e-3, "preset {p} at {rpm} rpm was silent");
            assert!(peak < 1.0, "preset {p} at {rpm} rpm pinned the output");
        }
    }
}

/// Survives extreme and degenerate configurations.
#[test]
fn survives_extreme_and_degenerate_configurations() {
    let nasty: Vec<(&str, serde_json::Value, Vec<PipeSegment>)> = vec![
        ("no pipe at all", json!({}), vec![]),
        ("a 10 mm stub", json!({}), vec![segment(SegmentKind::Pipe, 0.01, 0.01, None)]),
        ("huge chamber", json!({}), vec![segment(SegmentKind::Chamber, 2.0, 0.02, Some(0.4))]),
        ("closed throttle", json!({ "throttle": 0 }), common::pipe_preset(0)),
        ("zero lift", json!({ "maxLift": 0 }), common::pipe_preset(0)),
        ("valves never close", json!({ "evo": 0, "evc": 719, "ivo": 0, "ivc": 719 }), common::pipe_preset(0)),
        ("12000 rpm", json!({ "rpm": 12000 }), common::pipe_preset(0)),
        ("cold exhaust", json!({ "portGasTemp": 300 }), common::pipe_preset(1)),
    ];

    for (name, spec, pipe) in nasty {
        let mut cfg = common::default_config();
        cfg.engine = common::with(&cfg.engine, spec);
        cfg.pipe = pipe;
        let mut sim = EngineSim::new(FS, &cfg);
        let buf = sim.render(FS_N);
        for &v in &buf {
            assert!(v.is_finite(), "{name} produced a non-finite sample");
            assert!(v.abs() <= 1.0, "{name} exceeded full scale");
        }
    }
}

/// Produces a usable snapshot.
#[test]
fn produces_a_usable_snapshot() {
    let mut sim = run(json!({}), 1);
    sim.render(512);
    let s = sim.snapshot();
    assert!(s.crank_angle >= 0.0 && s.crank_angle < 720.0);
    assert!((s.rpm - 3200.0).abs() < 0.5, "rpm {}", s.rpm);
    assert!(s.cyl_pressure > 1e4);
    assert!(s.cyl_temp > 300.0);
    assert_eq!(s.pipe_pressure.len(), 128);
    assert!(s.pipe_pressure.iter().all(|v| v.is_finite()));
    assert!(s.pipe_cells > 8.0);
    // Every duct's cells, and no more.
    assert!(!s.duct_cells.is_empty() && s.duct_cells.iter().all(|&n| n > 0));
    assert_eq!(s.duct_pressure.len(), s.duct_cells.iter().sum::<u32>() as usize);
    assert_eq!(s.duct_ids.len(), s.duct_cells.len());
    assert!(s.duct_pressure.iter().all(|v| v.is_finite()));
    // One step per audio sample, for every engine.
    assert_eq!(s.substeps, 1.0);
}

// --- free-running crank dynamics ---

/// The mean speed a free-running default single settles to with `overrides`.
fn settle(overrides: serde_json::Value) -> f64 {
    let mut cfg = common::default_config();
    cfg.engine.free_running = true;
    cfg.engine = common::with(&cfg.engine, overrides);
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS_N * 4); // several seconds to find equilibrium
    // Average over a further second to smooth out cyclic irregularity.
    let mut sum = 0.0;
    let n = 40;
    for _ in 0..n {
        sim.render(FS_N / n);
        sum += sim.rpm();
    }
    sum / n as f64
}

/// Settles to a steady speed instead of running away or stalling.
#[test]
fn settles_to_a_steady_speed_instead_of_running_away_or_stalling() {
    let rpm = settle(json!({ "throttle": 0.7, "load": 0.46 }));
    assert!(rpm > 600.0 && rpm < 11000.0, "settled at {rpm}");
}

/// More load slows it down.
#[test]
fn more_load_slows_it_down() {
    let light = settle(json!({ "throttle": 0.8, "load": 0.28 }));
    let heavy = settle(json!({ "throttle": 0.8, "load": 0.78 }));
    assert!(heavy < light * 0.95, "light {light}, heavy {heavy}");
}

/// More throttle speeds it up.
#[test]
fn more_throttle_speeds_it_up() {
    let low = settle(json!({ "throttle": 0.3, "load": 0.37 }));
    let high = settle(json!({ "throttle": 1.0, "load": 0.37 }));
    assert!(high > low * 1.05, "low {low}, high {high}");
}

// --- slow motion ---

/// In slow motion the simulation takes the same steps, only fewer per output sample, and the sound
/// it plays is those steps drawn out, never a jump.
#[test]
fn slow_motion_takes_the_same_steps_drawn_out() {
    let mut real = run(json!({}), 1);
    let mut slow = run(json!({}), 1);
    let last = *real.render(64).last().unwrap();
    slow.render(64);

    slow.set_time_scale(0.01);
    let drawn = slow.render(48_000);
    real.render(480);
    assert_eq!(slow.snapshot().crank_angle, real.snapshot().crank_angle);

    // It picks up from the last sample real time played, and moves on by no more than a step at a time.
    assert_eq!(drawn[0], last);
    let largest = drawn.windows(2).map(|w| (w[1] - w[0]).abs()).fold(0.0f32, f32::max);
    assert!(largest < 0.05, "a step of {largest}");

    slow.set_time_scale(1.0);
    slow.render(64);
    real.render(64);
    assert_eq!(slow.snapshot().crank_angle, real.snapshot().crank_angle);
}

/// The Superquadro Mono makes about the real engine's rated 63 N*m at 8000 rpm and 77.5 hp (57 kW) at 9750.
#[test]
fn the_superquadro_mono_makes_about_the_real_engines_torque_and_power() {
    let torque = |rpm: f64| {
        let mut cfg = common::engine_preset("Single, Ducati Superquadro Mono").config.clone();
        let held = json!({ "freeRunning": false, "combustionVariability": 0, "throttle": 1, "rpm": rpm });
        cfg.engine = common::with(&cfg.engine, held);
        let mut sim = EngineSim::new(FS, &cfg);
        sim.render(2 * FS as usize);
        let n = FS as usize / 2;
        let mut t = 0.0;
        for _ in 0..n {
            sim.render(1);
            t += sim.snapshot().torque - sim.friction_torque();
        }
        t / n as f64
    };
    let t8000 = torque(8000.0);
    assert!((t8000 - 63.0).abs() < 0.1 * 63.0, "{t8000} N*m at 8000 rpm");
    let hp = torque(9750.0) * 9750.0 * 2.0 * std::f64::consts::PI / 60.0 / 745.7;
    assert!((hp - 77.5).abs() < 0.1 * 77.5, "{hp} hp at 9750 rpm");
}
