//! Tests for the things that separate "physically correct" from "sounds like an engine".
//!
//! A simulation can pass every acoustic and thermodynamic test and still sound synthetic if it is
//! perfectly periodic: cycles that correlate almost exactly, peak-pressure scatter far below the
//! 1-3% real engines show, a crank turning at a mathematically constant rate, and a listener
//! hearing a single anechoic monopole with no ground under it. Those are the properties guarded
//! here.

mod common;

use common::{FS, magnitude_spectrum, rms};
use engine_sim::EngineSim;
use engine_sim::cylinder::{Cylinder, SpecInstance};
use engine_sim::listener::{Listener, MouthPlace, SoundSources};
use engine_sim::spec::{CrankGeometry, SegmentPartial, crank_at, cylinder_volume, gas, make_segment};
use serde_json::json;
use std::f64::consts::PI;

const FS_N: usize = FS as usize;

/// The default single at 3200 rpm with `over` over its engine, on pipe preset `preset`, run for
/// two seconds.
fn sim(over: serde_json::Value, preset: usize) -> EngineSim {
    let mut cfg = common::default_config();
    cfg.engine.rpm = 3200.0;
    cfg.engine = common::with(&cfg.engine, over);
    cfg.pipe = common::pipe_preset(preset);
    let mut s = EngineSim::new(FS, &cfg);
    s.render(FS_N * 2);
    s
}

fn cycle_samples(rpm: f64) -> usize {
    ((FS * 120.0) / rpm).round() as usize
}

fn cov(values: &[f64]) -> f64 {
    let n = values.len() as f64;
    let m = values.iter().sum::<f64>() / n;
    let v = values.iter().map(|x| (x - m).powi(2)).sum::<f64>() / n;
    v.sqrt() / m.abs()
}

/// Indicated work per cycle, J, by integrating (p - ambient) dV.
fn indicated_work(s: &mut EngineSim, n: usize) -> Vec<f64> {
    let mut out = Vec::new();
    let len = cycle_samples(s.rpm());
    for _ in 0..n {
        let mut w = 0.0;
        let mut prev_v = cylinder_volume(s.engine(), s.cylinder().angle);
        for _ in 0..len {
            s.tick();
            let v = cylinder_volume(s.engine(), s.cylinder().angle);
            w += (s.cylinder().pressure(s.engine()) - gas::P_AMB) * (v - prev_v);
            prev_v = v;
        }
        out.push(w);
    }
    out
}

/// The lowest and highest instantaneous crank speed over `n` samples.
fn rpm_range(s: &mut EngineSim, n: usize) -> (f64, f64) {
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for _ in 0..n {
        s.tick();
        lo = lo.min(s.rpm_instant());
        hi = hi.max(s.rpm_instant());
    }
    (lo, hi)
}

/// A source and an ear `distance` apart across the ground, m, at their heights, over ground that
/// reflects `reflection`.
#[derive(Clone, Copy)]
struct Geom {
    distance: f64,
    mic_height: f64,
    source_height: f64,
    reflection: f64,
}

/// A listener at `geom`, its one path settled there.
fn listener_at(geom: Geom) -> Listener {
    let mut l = Listener::new(FS);
    let ear = [geom.distance, geom.mic_height, 0.0];
    l.set_geometry(ear, &[[0.0, geom.source_height, 0.0]], 0.0, geom.reflection, true);
    l
}

/// The impulse response of a listener at `geom`, `n` samples long.
fn impulse_response(geom: Geom, n: usize) -> Vec<f32> {
    let mut l = listener_at(geom);
    (0..n).map(|i| l.process(0, if i == 0 { 1.0 } else { 0.0 }) as f32).collect()
}

// --- no two cycles are alike ---

/// Combustion scatter can be switched off for a deterministic engine.
#[test]
fn combustion_scatter_can_be_switched_off_for_a_deterministic_engine() {
    let mut s = sim(json!({ "combustionVariability": 0, "throatNoise": 0, "mechNoise": 0 }), 1);
    // The air through the inlet tract takes a couple of seconds to settle from rest, and until it has
    // each cycle draws a little differently from the last.
    s.render(FS_N);
    let works = indicated_work(&mut s, 8);
    assert!(cov(&works) < 0.002, "cov {}", cov(&works));
}

// --- the crank does not turn at a constant rate ---

/// Ripples within the cycle even at a commanded fixed speed.
#[test]
fn ripples_within_the_cycle_even_at_a_commanded_fixed_speed() {
    let mut s = sim(json!({}), 1);
    let (lo, hi) = rpm_range(&mut s, cycle_samples(3200.0) * 4);
    let irregularity = (hi - lo) / ((hi + lo) / 2.0);
    // A big single with a modest flywheel swings a few percent.
    assert!(irregularity > 0.004 && irregularity < 0.15, "irregularity {irregularity}");
}

/// Still holds the commanded mean speed.
#[test]
fn still_holds_the_commanded_mean_speed() {
    // The ripple integrator must carry no DC, or the rpm slider stops meaning anything and the
    // error becomes load-dependent.
    for throttle in [0.2, 0.6, 1.0] {
        let mut s = sim(json!({ "throttle": throttle }), 1);
        let mut sum = 0.0;
        let n = cycle_samples(3200.0) * 12;
        for _ in 0..n {
            s.tick();
            sum += s.rpm_instant();
        }
        let mean = sum / n as f64;
        assert!((mean - 3200.0).abs() / 3200.0 < 0.01, "throttle {throttle}: mean {mean}");
        // The steady readout should report the commanded mean, not the ripple.
        assert!((s.rpm() - 3200.0).abs() < 0.5, "throttle {throttle}: readout {}", s.rpm());
    }
}

/// A heavier flywheel smooths the ripple.
#[test]
fn a_heavier_flywheel_smooths_the_ripple() {
    let swing = |flywheel_inertia: f64| {
        let mut s = sim(json!({ "flywheelInertia": flywheel_inertia }), 1);
        let (lo, hi) = rpm_range(&mut s, cycle_samples(3200.0) * 4);
        hi - lo
    };
    assert!(swing(1.0) < swing(0.1));
}

// --- reciprocating inertia ---

/// Matches a numerical derivative of the piston motion.
#[test]
fn matches_a_numerical_derivative_of_the_piston_motion() {
    let spec = &common::presets().default_engine;
    let g = CrankGeometry::of(spec);
    let d_piston = |deg: f64| crank_at(&g, deg).d_position;
    let d2_piston = |deg: f64| crank_at(&g, deg).d2_position;
    let h = 1e-3;
    for deg in [15.0, 60.0, 110.0, 190.0, 265.0, 340.0] {
        let numeric_first =
            ((cylinder_volume(spec, deg + h) - cylinder_volume(spec, deg - h)) / (2.0 * h)) * (180.0 / PI);
        // dV/dtheta = -A * dx/dtheta, so this cross-checks the piston velocity.
        let area = (PI * spec.bore * spec.bore) / 4.0;
        assert!(
            (d_piston(deg) - -numeric_first / area).abs() < 0.5e-6,
            "first derivative at {deg}: {} vs {}",
            d_piston(deg),
            -numeric_first / area
        );

        let numeric_second = ((d_piston(deg + h) - d_piston(deg - h)) / (2.0 * h)) * (180.0 / PI);
        assert!(
            (d2_piston(deg) - numeric_second).abs() < 0.5e-5,
            "second derivative at {deg}: {} vs {numeric_second}",
            d2_piston(deg)
        );
    }
}

/// Does no net work over a cycle, so it cannot change the mean speed.
#[test]
fn does_no_net_work_over_a_cycle_so_it_cannot_change_the_mean_speed() {
    // It only stores and returns energy. If it integrated to something non-zero it would act as a
    // phantom torque and the rpm would depend on the piston mass.
    let spec = common::presets().default_engine.clone();
    let si = SpecInstance::new(spec.clone());
    let mut cyl = Cylinder::new(&spec, 0.0, 0x51f3a7 as f64);
    let omega = 335.0;
    let advance = |cyl: &mut Cylinder, deg: f64| {
        cyl.angle = deg;
        cyl.advance_with(&si, 1e-9, omega, 0.0, 0.0, gas::T_AMB, 900.0, 0.0, None);
        cyl.inertia_torque
    };
    let mut integral = 0.0;
    let step = 0.05;
    let mut deg = 0.0;
    while deg < 720.0 {
        integral += advance(&mut cyl, deg) * ((step * PI) / 180.0);
        deg += step;
    }
    // Normalise against the scale of the torque itself.
    let mut scale = 0.0f64;
    let mut deg = 0.0;
    while deg < 720.0 {
        scale = scale.max(advance(&mut cyl, deg).abs());
        deg += 10.0;
    }
    assert!(integral.abs() / (scale * 2.0 * PI) < 0.01, "net {integral} against scale {scale}");
}

// --- the listener is outdoors, not in a vacuum ---

/// The ground reflection combs the spectrum where geometry says it should.
#[test]
fn the_ground_reflection_combs_the_spectrum_where_geometry_says_it_should() {
    let geom = Geom { distance: 1.5, mic_height: 1.2, source_height: 0.35, reflection: 1.0 };

    // Impulse in, so the response is the two-path filter itself.
    let n = 8192;
    let ir = impulse_response(geom, n);
    let mag = magnitude_spectrum(&ir, n);

    let c = (gas::GAMMA_EXH * gas::R * gas::T_AMB).sqrt();
    let r_direct = geom.distance.hypot(geom.mic_height - geom.source_height);
    let r_ground = geom.distance.hypot(geom.mic_height + geom.source_height);
    let null_hz = c / (2.0 * (r_ground - r_direct));

    let bin_hz = FS / n as f64;
    let at = |hz: f64| mag[(hz / bin_hz).round() as usize];
    // Destructive at the predicted null, constructive at twice it.
    assert!(at(null_hz) < at(null_hz * 2.0) * 0.6, "null {} vs peak {}", at(null_hz), at(null_hz * 2.0));
}

/// Moving the listener changes the colouration.
#[test]
fn moving_the_listener_changes_the_colouration() {
    let response = |mic_height: f64| {
        let n = 4096;
        let geom = Geom { distance: 1.5, mic_height, source_height: 0.35, reflection: 0.8 };
        magnitude_spectrum(&impulse_response(geom, n), n)
    };
    let a = response(0.4);
    let b = response(1.8);
    let diff: f64 = (1..a.len()).map(|i| (a[i] - b[i]).abs()).sum();
    assert!(diff / a.len() as f64 > 0.01);
}

/// A hard surface reflects more than a soft one.
#[test]
fn a_hard_surface_reflects_more_than_a_soft_one() {
    let energy = |reflection: f64| {
        let mut l = listener_at(Geom { distance: 2.0, mic_height: 1.2, source_height: 0.35, reflection });
        let mut e = 0.0;
        for i in 0..4096 {
            let y = l.process(0, if i == 0 { 1.0 } else { 0.0 });
            e += y * y;
        }
        e
    };
    assert!(energy(0.9) > energy(0.1));
}

/// Distance still attenuates, and dulls as well as quietens.
#[test]
fn distance_still_attenuates_and_dulls_as_well_as_quietens() {
    let measure = |distance: f64| {
        let n = 4096;
        let geom = Geom { distance, mic_height: 1.2, source_height: 0.35, reflection: 0.7 };
        let mag = magnitude_spectrum(&impulse_response(geom, n), n);
        let bin_hz = FS / n as f64;
        let band = |lo: f64, hi: f64| {
            let mut e = 0.0;
            let mut i = (lo / bin_hz).floor() as usize;
            while (i as f64) < hi / bin_hz {
                e += mag[i] * mag[i];
                i += 1;
            }
            e
        };
        (band(100.0, 400.0), band(6000.0, 12000.0))
    };
    let (near_low, near_high) = measure(1.5);
    let (far_low, far_high) = measure(12.0);
    assert!(far_low < near_low);
    // Air absorption: the far signal loses proportionally more treble than bass.
    assert!(far_high / far_low < near_high / near_low);
}

// --- structural noise is driven by combustion, not by a schedule ---

/// A faster burn produces a steeper pressure rise.
#[test]
fn a_faster_burn_produces_a_steeper_pressure_rise() {
    // The mechanism, checked directly rather than through the mix.
    let steepness = |burn_duration: f64| {
        let mut s = sim(json!({ "burnDuration": burn_duration, "combustionVariability": 0 }), 1);
        let mut pk = 0.0f64;
        for _ in 0..cycle_samples(3200.0) * 3 {
            s.tick();
            pk = pk.max(s.cylinder().dpdt.abs());
        }
        pk
    };
    // Roughly 14 GPa/s at 20 deg falling to 1.6 at 90 deg.
    let (fast, mid, slow) = (steepness(20.0), steepness(45.0), steepness(90.0));
    assert!(fast > mid * 1.4, "20 deg {fast} vs 45 deg {mid}");
    assert!(mid > slow * 1.4, "45 deg {mid} vs 90 deg {slow}");
}

/// And therefore rings the casing harder.
#[test]
fn and_therefore_rings_the_casing_harder() {
    // Isolated by differencing mechNoise on against off, so the gas path — which is also affected
    // by burn duration, and far louder — cancels out. Measuring the total instead just measures
    // the exhaust.
    let structure = |burn_duration: f64| {
        let base = |mech: f64| json!({ "burnDuration": burn_duration, "combustionVariability": 0, "throatNoise": 0, "mechNoise": mech });
        let on = sim(base(1.0), 3).render(FS_N / 2);
        let off = sim(base(0.0), 3).render(FS_N / 2);
        let d: f64 = on.iter().zip(&off).map(|(&a, &b)| (a as f64 - b as f64).powi(2)).sum();
        (d / on.len() as f64).sqrt()
    };
    assert!(structure(20.0) > structure(90.0) * 2.0);
}

/// Mechanical noise sits well below an open exhaust.
#[test]
fn mechanical_noise_sits_well_below_an_open_exhaust() {
    let open = rms(&sim(json!({ "mechNoise": 0, "throatNoise": 0 }), 0).render(FS_N));
    let with_mech = rms(&sim(json!({ "mechNoise": 1, "throatNoise": 0 }), 0).render(FS_N));
    // Turning it to full must barely move an open pipe's level.
    assert!(20.0 * (with_mech / open).log10() < 2.0);
}

/// Stays finite with degenerate listener geometry.
#[test]
fn stays_finite_with_degenerate_listener_geometry() {
    let mut s = sim(
        json!({
            "exhaustHeight": 0,
            "groundReflection": 1,
            "recipMass": 5,
            "flywheelInertia": 0.02,
        }),
        1,
    );
    // The ear on the casing, below the ground.
    s.set_listener(Some([0.0, -5.0, 0.0]));
    let buf = s.render(FS_N);
    for &v in &buf {
        assert!(v.is_finite());
        assert!(v.abs() <= 1.0);
    }
}

/// The default single, every source of it at `at`, heard from `ear` over ground that reflects nothing.
fn heard_from(at: [f64; 3], ear: [f64; 3]) -> EngineSim {
    let mut cfg = common::default_config();
    cfg.engine.rpm = 3200.0;
    cfg.engine = common::with(&cfg.engine, json!({ "groundReflection": 0, "combustionVariability": 0 }));
    let mut s = EngineSim::new(FS, &cfg);
    let duct = s.pipe_solver().mouth_duct_id(0).unwrap().to_string();
    s.set_sources(SoundSources {
        mouths: vec![MouthPlace { duct, position: at }],
        intake: Some(at),
        second_intake: None,
        engine: Some(at),
        turbo: Some(at),
    });
    s.set_listener(Some(ear));
    s.render(FS_N * 2);
    s
}

/// Twice as far from the engine, the ear hears it half as loud.
#[test]
fn twice_as_far_from_the_engine_the_ear_hears_it_half_as_loud() {
    let at = [0.3, 0.2, 1.4];
    let near = rms(&heard_from(at, [at[0] + 1.0, at[1], at[2]]).render(FS_N));
    let far = rms(&heard_from(at, [at[0] + 2.0, at[1], at[2]]).render(FS_N));
    // Air absorption takes a little more of the treble over the longer path.
    assert!((near / far - 2.0).abs() < 0.15, "near {near} far {far}");
}

/// The ear can be moved without a click: the paths glide to their new lengths.
#[test]
fn the_ear_can_be_moved_without_a_click() {
    let at = [0.0, 0.0, 1.0];
    let mut s = heard_from(at, [1.0, 1.0, 2.0]);
    let steps = |buf: &[f32]| buf.windows(2).map(|w| (w[1] - w[0]).abs()).fold(0.0f32, f32::max);
    let steady = steps(&s.render(FS_N / 2));
    s.set_listener(Some([-2.0, 0.5, 3.5]));
    let moving = steps(&s.render(FS_N / 10));
    assert!(moving < steady * 1.5, "steps {moving} moving against {steady} steady");
}

/// A mouth the sources do not name is still heard, from behind the engine.
#[test]
fn a_mouth_the_sources_do_not_name_is_still_heard() {
    let mut cfg = common::default_config();
    cfg.sources = Some(SoundSources::default());
    let mut s = EngineSim::new(FS, &cfg);
    s.render(FS_N);
    assert!(rms(&s.render(FS_N / 2)) > 1e-3);
}

// --- the basics still hold ---

/// Still produces a stable, audible signal across presets.
#[test]
fn still_produces_a_stable_audible_signal_across_presets() {
    for p in 0..common::presets().pipe_presets.len() {
        let buf = sim(json!({}), p).render(FS_N / 2);
        let mut peak = 0.0f32;
        for &v in &buf {
            assert!(v.is_finite());
            peak = peak.max(v.abs());
        }
        assert!(peak > 1e-3, "preset {p} silent");
        assert!(peak < 1.0, "preset {p} pinned");
    }
}

/// An empty pipe with a port still works.
#[test]
fn an_empty_pipe_with_a_port_still_works() {
    let mut cfg = common::default_config();
    cfg.pipe = vec![make_segment(SegmentPartial { length: Some(0.05), d_in: Some(0.04), ..Default::default() })];
    let mut s = EngineSim::new(FS, &cfg);
    let buf = s.render(FS_N);
    assert!(buf.iter().all(|v| v.is_finite()));
}
