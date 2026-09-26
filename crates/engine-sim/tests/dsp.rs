//! The structural mode filter, and the structure-borne sounds it rings.
//!
//! A radiating surface puts out nothing at DC, so a mode is a band-pass. A resonant low-pass would pass
//! everything below it — the combustion drive's firing harmonics and the valve clacks' repetition rate,
//! both of which scale with the cylinder count — and the mechanical noise would rise in pitch as
//! cylinders were added. The band-pass must still ring, decay and peak at the gain the calibration
//! assumes, since the levels driving it are calibrated in pascals.

mod common;

use common::FS;
use engine_sim::EngineSim;
use engine_sim::dsp::Resonator;
use engine_sim::engine_sim::clack_share;
use serde_json::{Value, json};
use std::collections::HashSet;
use std::f64::consts::PI;

/// `expect(actual).toBeCloseTo(expected, digits)`: within half a unit in the `digits`th decimal place.
#[track_caller]
fn assert_close(actual: f64, expected: f64, digits: f64) {
    let tol = 10f64.powf(-digits) / 2.0;
    assert!((actual - expected).abs() < tol, "expected {actual} to be within {tol} of {expected}");
}

/// Steady-state gain at `hz`, measured by driving a sine through and taking the settled amplitude.
fn gain_at(r: &mut Resonator, hz: f64) -> f64 {
    r.reset();
    let mut peak: f64 = 0.0;
    let n = FS as usize / 2;
    for i in 0..n {
        let y = r.process(((2.0 * PI * hz * i as f64) / FS).sin());
        if i > n / 2 {
            peak = peak.max(y.abs());
        }
    }
    peak
}

// --- a structural mode ---

/// Passes nothing at DC.
#[test]
fn structural_mode_passes_nothing_at_dc() {
    for (hz, q) in [(620.0, 9.0), (780.0, 11.0), (2700.0, 14.0), (4700.0, 20.0)] {
        let mut r = Resonator::new(hz, q, FS);
        let mut y = 0.0;
        for _ in 0..FS as usize {
            y = r.process(1.0);
        }
        assert!(y.abs() < 1e-9, "{hz} Hz: {y}");
    }
}

/// Passes far less below its resonance than at it.
///
/// Where the firing harmonics of a slow engine sit: well below every mode.
#[test]
fn structural_mode_passes_far_less_below_its_resonance_than_at_it() {
    let mut r = Resonator::new(780.0, 11.0, FS);
    let at_mode = gain_at(&mut r, 780.0);
    assert!(gain_at(&mut r, 100.0) / at_mode < 0.02);
    assert!(gain_at(&mut r, 200.0) / at_mode < 0.05);
}

/// Peaks where it should, at the gain the calibration assumes.
#[test]
fn structural_mode_peaks_where_it_should_at_the_gain_the_calibration_assumes() {
    let mut r = Resonator::new(780.0, 11.0, FS);
    let rr = ((-PI * 780.0) / (11.0 * FS)).exp();
    // 1 / (2 (1 - r)): the same peak gain as the all-pole form, which the upstream levels assume.
    assert_close(gain_at(&mut r, 780.0), 1.0 / (2.0 * (1.0 - rr)), -0.5);
    assert!(gain_at(&mut r, 780.0) > gain_at(&mut r, 700.0));
    assert!(gain_at(&mut r, 780.0) > gain_at(&mut r, 860.0));
}

/// Rings to a peak of about the impulse it was struck with.
#[test]
fn structural_mode_rings_to_a_peak_of_about_the_impulse_it_was_struck_with() {
    let mut r = Resonator::new(2700.0, 14.0, FS);
    let mut peak: f64 = 0.0;
    peak = peak.max(r.process(1.0).abs());
    for _ in 0..400 {
        peak = peak.max(r.process(0.0).abs());
    }
    assert!(peak > 0.9, "peak {peak}");
    assert!(peak < 1.1, "peak {peak}");
}

// --- structure-borne sound ---
//
// The structure-borne sounds, pitched and levelled for the engine they are in.
//
// Rung at a single cylinder's frequencies and levels, a large engine's mechanical layer would sit far
// too high and too loud. Piston slap is counted outright, because a TDC check that compared each angle
// with itself would never fire and the slap would never sound.

fn sim(engine: Value) -> EngineSim {
    let mut cfg = common::default_config();
    cfg.engine = common::with(&cfg.engine, engine);
    EngineSim::new(FS, &cfg)
}

/// Slaps its pistons, twice a cycle each.
#[test]
fn structure_borne_slaps_its_pistons_twice_a_cycle_each() {
    let mut s = sim(json!({ "cylinders": 1, "rpm": 3000, "throttle": 0.8 }));
    s.render(FS as usize);
    // 3000 rpm is 25 cycles a second, and a piston crosses TDC twice a cycle.
    assert!(s.slap_count >= 48, "slaps {}", s.slap_count);
    assert!(s.slap_count <= 52, "slaps {}", s.slap_count);
}

/// Rings bigger cylinders lower, whatever their number.
///
/// By the size of a cylinder, not by how many there are: the block's radiating modes are its wall panels,
/// and a panel spans a cylinder. Scaled by the whole engine, a V8's lowest mode would sit on its own
/// firing harmonics and boom over the exhaust.
#[test]
fn structure_borne_rings_bigger_cylinders_lower_whatever_their_number() {
    let (small_block, _, small_slap) = sim(json!({ "cylinders": 1 })).structural_frequencies();
    let (big_block, _, big_slap) = sim(json!({
        "cylinders": 8, "vAngle": 90, "crankType": "crossplane", "bore": 0.102, "stroke": 0.084
    }))
    .structural_frequencies();
    let ratio = ((0.102f64.powi(2) * 0.084) / (0.089f64.powi(2) * 0.08)).cbrt();
    let (eight_block, _, _) =
        sim(json!({ "cylinders": 8, "vAngle": 90, "crankType": "crossplane" })).structural_frequencies();
    for (i, &hz) in small_block.iter().enumerate() {
        assert_close(eight_block[i], hz, 6.0);
    }
    for (i, &hz) in small_block.iter().enumerate() {
        assert_close(big_block[i] * ratio, hz, 0.0);
    }
    // Slap is the bore ringing: a wider bore rings lower.
    let mean_slap = big_slap.iter().sum::<f64>() / big_slap.len() as f64;
    assert_close(mean_slap, small_slap[0] * (0.089 / 0.102), -1.0);
}

/// Gives each cylinder its own ring.
#[test]
fn structure_borne_gives_each_cylinder_its_own_ring() {
    let (_, clack, slap) =
        sim(json!({ "cylinders": 8, "vAngle": 90, "crankType": "crossplane" })).structural_frequencies();
    let distinct = |v: &[f64]| v.iter().map(|hz| format!("{hz:.1}")).collect::<HashSet<_>>().len();
    assert_eq!(distinct(&clack), 8);
    assert_eq!(distinct(&slap), 8);
    for &hz in &clack {
        assert!((hz / 2700.0 - 1.0).abs() <= 0.061, "clack {hz}");
    }
}

/// Retunes when the engine changes size.
#[test]
fn structure_borne_retunes_when_the_engine_changes_size() {
    let mut s = sim(json!({ "cylinders": 1 }));
    let before = s.structural_frequencies().0[0];
    s.set_engine_json(&json!({ "bore": 0.1 })).unwrap();
    assert!(s.structural_frequencies().0[0] < before);
}

/// Makes each event quieter the more cylinders share its casting.
#[test]
fn structure_borne_makes_each_event_quieter_the_more_cylinders_share_its_casting() {
    assert_eq!(clack_share(1.0), 1.0);
    // Power per event goes as one over the cylinders sharing: a bank of four is 6 dB down.
    assert_close(20.0 * clack_share(4.0).log10(), -6.02, 2.0);
}
