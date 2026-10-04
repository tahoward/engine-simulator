//! Twin-cylinder tests: the firing geometry, and the junction that merges two exhausts.
//!
//! The claims worth checking are the ones a listener would notice. An evenly-firing twin has
//! no half-order component at all, because both cylinders fire 360 degrees apart and the firing
//! frequency simply doubles. An unevenly-firing one does, and more strongly the more uneven it
//! is — that is the whole of the Harley thump. And a shared collector has to actually couple the
//! banks, or it is just two singles playing at once.

mod common;

use common::{FS, band_energy, hann, magnitude_spectrum};
use engine_sim::EngineSim;
use engine_sim::euler_pipe::{EulerPipe, EulerPipeOptions, ValveState};
use engine_sim::exhaust_graph::{DuctSink, DuctSource, ExhaustDuct, ExhaustGraph};
use engine_sim::exhaust_system::ExhaustSystem;
use engine_sim::spec::{PipeSegment, SegmentKind, SegmentPartial, firing_offset_deg, gas, make_segment};
use serde_json::{Value, json};

const N: usize = 65536;

fn pipe(length: f64, d_in: f64) -> PipeSegment {
    make_segment(SegmentPartial {
        kind: Some(SegmentKind::Pipe),
        length: Some(length),
        d_in: Some(d_in),
        ..Default::default()
    })
}

/// A twin, with the fields in `over` on top, run for two seconds.
fn twin(over: Value) -> EngineSim {
    let mut cfg = common::default_config();
    let base = json!({ "cylinders": 2, "exhaustLayout": "2into1", "vAngle": 45, "firingOffset": null, "rpm": 3000 });
    cfg.engine = common::with(&common::with(&cfg.engine, base), over);
    cfg.pipe = vec![pipe(0.35, 0.04)];
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS as usize * 2);
    sim
}

fn duct(id: &str, segments: Vec<PipeSegment>, from: DuctSource, to: DuctSink) -> ExhaustDuct {
    ExhaustDuct {
        id: id.to_string(),
        segments,
        from,
        to,
        heading_yaw: None,
        heading_pitch: None,
        heading_frame: None,
        continues: None,
        role: None,
    }
}

/// Two runners into one collector, as a graph.
///
/// Built by hand rather than through `compile_layout` so the test states the topology it is testing
/// instead of depending on what a layout name currently means.
fn twin_graph() -> ExhaustGraph {
    let merge = || DuctSink::Node { node: "merge".into() };
    ExhaustGraph {
        ducts: vec![
            duct("runner0", vec![pipe(0.6, 0.042)], DuctSource::Valve { cylinder: 0 }, merge()),
            duct("runner1", vec![pipe(0.6, 0.042)], DuctSource::Valve { cylinder: 1 }, merge()),
            duct("collector", vec![pipe(0.9, 0.055)], DuctSource::Node { node: "merge".into() }, DuctSink::Mouth),
        ],
        ..Default::default()
    }
}

/// Lossless, adiabatic and silent ducts on a fine grid: only the junction left to conserve or not.
fn bare_twin_system() -> ExhaustSystem {
    let opts = EulerPipeOptions {
        heat_transfer: Some(false),
        linear_damping: Some(0.0),
        darcy_friction: Some(0.0),
        radiate: Some(false),
        cell_size: Some(0.008),
        max_cells: Some(512),
        max_substeps: Some(64),
        ..Default::default()
    };
    ExhaustSystem::new(&twin_graph(), 2, FS, gas::T_AMB, &opts).expect("twin graph builds")
}

fn shut() -> ValveState {
    ValveState { throat_area: 0.0, cyl_pressure: gas::P_AMB, cyl_temp: gas::T_AMB, ..Default::default() }
}

/// `(a - b + 720) % 720`: how far `b` is behind `a` on the crank.
fn behind(a: f64, b: f64) -> f64 {
    (a - b + 720.0) % 720.0
}

mod firing_geometry {
    use super::*;

    /// a shared crankpin ties the firing interval to the V angle
    #[test]
    fn a_shared_crankpin_ties_the_firing_interval_to_the_v_angle() {
        let base = common::default_config().engine;
        let at = |v: f64| firing_offset_deg(&common::with(&base, json!({ "vAngle": v, "firingOffset": null })));
        // 360 + V, so the two intervals are 360+V and 360-V.
        assert_eq!(at(45.0), 405.0);
        assert_eq!(at(90.0), 450.0);
        assert_eq!(at(0.0), 360.0);
    }

    /// an override breaks that relationship, for layouts a shared pin cannot make
    #[test]
    fn an_override_breaks_that_relationship_for_layouts_a_shared_pin_cannot_make() {
        let base = common::default_config().engine;
        let at = |v: f64, o: f64| firing_offset_deg(&common::with(&base, json!({ "vAngle": v, "firingOffset": o })));
        assert_eq!(at(45.0, 270.0), 270.0);
        assert_eq!(at(90.0, 360.0), 360.0);
    }

    /// both banks run, a fixed number of degrees apart, and stay there
    #[test]
    fn both_banks_run_a_fixed_number_of_degrees_apart_and_stay_there() {
        let mut sim = twin(json!({ "vAngle": 45 }));
        // Mind the sign: cylinder 2 fires 405 degrees *after* cylinder 1, which means it is 405
        // degrees *behind* on the crank — it has that much further to go to reach its own firing
        // TDC. Asserting `b - a` instead would demand 315 and quietly accept a reversed firing
        // order, which is invisible in a twin and wrong in a V8.
        let gap = |sim: &EngineSim| behind(sim.cylinders()[0].angle, sim.cylinders()[1].angle);
        let first = gap(&sim);
        assert!((first - 405.0).abs() < 0.5, "gap {first}");
        // They share one crank, so the offset must not drift over thousands of cycles.
        sim.render(FS as usize * 3);
        assert!((gap(&sim) - first).abs() < 0.5, "gap {} after {first}", gap(&sim));
    }

    /// both cylinders actually fire
    #[test]
    fn both_cylinders_actually_fire() {
        let mut sim = twin(json!({}));
        let mut peaks = [0.0f64; 2];
        for _ in 0..FS as usize {
            sim.tick();
            for (b, peak) in peaks.iter_mut().enumerate() {
                *peak = peak.max(sim.cylinders()[b].pressure(sim.engine()));
            }
        }
        for p in peaks {
            assert!(p > 20e5, "peak {p}");
        }
        // Symmetric cylinders, so neither should dominate.
        assert!((peaks[0] - peaks[1]).abs() / peaks[0] < 0.5, "{peaks:?}");
    }
}

mod firing_interval_shapes_the_spectrum {
    use super::*;

    /// 25 Hz, one firing per cylinder per two revolutions.
    const F0: f64 = 3000.0 / 120.0;

    /// Energy at the half order against the full order the twin always has.
    fn ratio(mag: &[f64]) -> f64 {
        band_energy(mag, FS, N, F0, 3.0) / band_energy(mag, FS, N, 2.0 * F0, 3.0)
    }

    fn half_order_ratio(over: Value) -> f64 {
        ratio(&magnitude_spectrum(&hann(&twin(over).render(N)), N))
    }

    /// an evenly firing twin has essentially no half order
    #[test]
    fn an_evenly_firing_twin_has_essentially_no_half_order() {
        // 360/360 means the two firings are evenly spaced, so the fundamental is 2*f0 and the
        // half order has nothing to excite it. Measures about four orders of magnitude down.
        let r = half_order_ratio(json!({ "firingOffset": 360 }));
        assert!(r < 0.01, "ratio {r}");
    }

    /// an unevenly firing twin does, and more so the more uneven it is
    #[test]
    fn an_unevenly_firing_twin_does_and_more_so_the_more_uneven_it_is() {
        let v45 = half_order_ratio(json!({ "vAngle": 45, "firingOffset": null })); // 405/315
        let v90 = half_order_ratio(json!({ "vAngle": 90, "firingOffset": null })); // 450/270
        assert!(v45 > 0.005, "v45 {v45}");
        assert!(v90 > v45, "v90 {v90} v45 {v45}");
    }

    /// a single is not the same as a twin
    #[test]
    fn a_single_is_not_the_same_as_a_twin() {
        let mut single = common::default_config();
        single.engine = common::with(&single.engine, json!({ "cylinders": 1, "exhaustLayout": "single", "rpm": 3000 }));
        single.pipe = vec![pipe(0.35, 0.04)];
        let mut s = EngineSim::new(FS, &single);
        s.render(FS as usize * 2);
        let single_mag = magnitude_spectrum(&hann(&s.render(N)), N);
        let twin_mag = magnitude_spectrum(&hann(&twin(json!({ "firingOffset": 360 })).render(N)), N);

        // A single fires once per cycle, so it has a strong half order; an evenly firing twin
        // essentially none. Same rpm, same pipe.
        let (single_ratio, twin_ratio) = (ratio(&single_mag), ratio(&twin_mag));
        assert!(single_ratio > twin_ratio * 20.0, "single {single_ratio} twin {twin_ratio}");
    }
}

mod the_collector_couples_the_banks {
    use super::*;

    /// Bank 0's port pressure over 0.1 s, once warm.
    ///
    /// The heavy flywheel is what makes this measurement mean anything. There are *two* paths by
    /// which one cylinder can reach the other, and the crankshaft is the one that is easy to
    /// forget: torque ripple from bank 1 speeds and slows the shared crank, so bank 0's own
    /// schedule shifts. Pinning the rpm slider is not enough, because the ripple is applied on
    /// top of the mean speed. Only with the inertia turned up does the exhaust become the sole
    /// remaining path, which is the one this block is about.
    fn bank0_port(layout: &str, firing_offset: f64) -> Vec<f64> {
        port_trace(twin(json!({
            "exhaustLayout": layout,
            "firingOffset": firing_offset,
            "freeRunning": false,
            "flywheelInertia": 1e6,
        })))
    }

    fn port_trace(mut sim: EngineSim) -> Vec<f64> {
        (0..4800)
            .map(|_| {
                sim.tick();
                sim.pipe_solver().primaries()[0].port_pressure() - gas::P_AMB
            })
            .collect()
    }

    /// RMS difference between two traces, as a fraction of the first one's RMS.
    fn rel_diff(a: &[f64], b: &[f64]) -> f64 {
        let mut d = 0.0;
        let mut r = 0.0;
        for (x, y) in a.iter().zip(b) {
            d += (x - y).powi(2);
            r += x.powi(2);
        }
        (d / f64::max(r, 1e-30)).sqrt()
    }

    /// through a collector, bank 0 feels where bank 1 fires
    ///
    /// The real test of coupling. Bank 0's crank schedule is identical in both runs — only
    /// *bank 1's* phase moves. Any change bank 0 sees must have arrived through the junction.
    ///
    /// Comparing 2-into-1 against 2-into-2 directly would not show this: the merged path is also
    /// longer, so its port pressure differs for reasons that have nothing to do with the other
    /// cylinder. A mutant junction that couples each primary only to the collector fails here,
    /// whereas a test comparing fluctuation *magnitude* between the two layouts would pass it
    /// happily.
    #[test]
    fn through_a_collector_bank_0_feels_where_bank_1_fires() {
        // Measures ~1.27, i.e. the change is larger than the signal itself.
        let d = rel_diff(&bank0_port("2into1", 360.0), &bank0_port("2into1", 450.0));
        assert!(d > 0.3, "relative difference {d}");
    }

    /// with separate pipes the exhaust path is gone, leaving only the intake
    #[test]
    fn with_separate_pipes_the_exhaust_path_is_gone_leaving_only_the_intake() {
        // Measures ~1.8e-2, and that is the *intake* path rather than a leak.
        //
        // The intake plenum is finite, a real shared volume, and so is the tract it draws its air
        // through: move bank 1's firing and you move when it draws from and spits into the manifold
        // bank 0 breathes out of, and the waves that sends up the tract and back, so bank 0's
        // trapped mass changes. Engines do this — it is why a twin on one throttle body behaves
        // differently from one with two — so the coupling belongs here. What must stay true is
        // that it is small: over ten times under the 3e-1 that the collector produces, and tight
        // enough to catch a single turbulence generator shared between the cylinders' exhaust
        // valves, which adds about 4e-2 on top of it.
        let d = rel_diff(&bank0_port("2into2", 360.0), &bank0_port("2into2", 450.0));
        assert!(d < 3e-2, "relative difference {d}");
    }

    /// but the crankshaft is a path of its own
    #[test]
    fn but_the_crankshaft_is_a_path_of_its_own() {
        // Same separate pipes, ordinary flywheel: torque ripple from bank 1 pushes the shared
        // crank around, so bank 0 does feel it. A twin is never two independent singles.
        let light = |firing_offset: f64| {
            port_trace(twin(json!({ "exhaustLayout": "2into2", "firingOffset": firing_offset, "freeRunning": false })))
        };
        let d = rel_diff(&light(360.0), &light(450.0));
        assert!(d > 0.01, "relative difference {d}");
    }

    /// a pulse in one primary reaches the other
    #[test]
    fn a_pulse_in_one_primary_reaches_the_other() {
        let mut sys = bare_twin_system();
        let shut = [shut(), shut()];
        let rho0 = gas::P_AMB / (gas::R * gas::T_AMB);
        for i in 4..12 {
            sys.ducts[0].set_primitive(i, rho0 * 1.3, 0.0, gas::P_AMB * 1.3);
        }

        let peak = |d: &EulerPipe| (0..d.n).map(|i| (d.pressure_at(i) - gas::P_AMB).abs()).fold(0.0, f64::max);
        assert!(peak(&sys.primaries()[1]) < 1.0);
        for _ in 0..(FS * 0.004) as usize {
            sys.advance(1.0 / FS, &shut);
        }
        assert!(peak(&sys.primaries()[1]) > 500.0, "{}", peak(&sys.primaries()[1]));
        assert!(peak(sys.collector().unwrap()) > 500.0, "{}", peak(sys.collector().unwrap()));
    }

    /// nearly conserves mass and energy through the junction
    #[test]
    fn nearly_conserves_mass_and_energy_through_the_junction() {
        let mut sys = bare_twin_system();
        let shut = [shut(), shut()];
        for i in 4..12 {
            sys.ducts[0].set_primitive(i, 1.4, 0.0, gas::P_AMB * 1.5);
        }
        fn all(sys: &ExhaustSystem) -> [&EulerPipe; 3] {
            [&sys.primaries()[0], &sys.primaries()[1], sys.collector().unwrap()]
        }
        let mass = |sys: &ExhaustSystem| all(sys).iter().map(|d| d.total_mass()).sum::<f64>();
        let energy = |sys: &ExhaustSystem| all(sys).iter().map(|d| d.total_energy()).sum::<f64>();
        let m0 = mass(&sys);
        let e0 = energy(&sys);
        for _ in 0..(FS * 0.2) as usize {
            sys.advance(1.0 / FS, &shut);
        }

        // Looser than the 1e-9 a single duct manages, and deliberately so: the junction
        // linearises the wave returning into each duct, which is not exactly conservative. About
        // 0.01% over 0.2 s.
        assert!((mass(&sys) / m0 - 1.0).abs() < 1e-3, "mass {}", mass(&sys) / m0 - 1.0);
        assert!((energy(&sys) / e0 - 1.0).abs() < 1e-3, "energy {}", energy(&sys) / e0 - 1.0);
        assert_eq!(sys.recoveries(), 0);
    }
}

mod robustness {
    use super::*;

    /// every engine preset runs clean
    #[test]
    fn every_engine_preset_runs_clean() {
        for preset in &common::presets().engine_presets {
            let name = &preset.name;
            let mut sim = EngineSim::new(FS, &preset.config);
            sim.render(FS as usize);
            let buf = sim.render(FS as usize);
            let mut peak = 0.0f32;
            for v in &buf {
                assert!(v.is_finite(), "{name}");
                peak = peak.max(v.abs());
            }
            assert!(peak > 1e-3, "{name} silent");
            assert!(peak < 1.0, "{name} pinned");
            assert_eq!(sim.pipe_solver().recoveries(), 0, "{name}");
            assert_eq!(sim.cylinder().clamp_hits, 0, "{name}");
        }
    }

    /// switching layout and cylinder count mid-run stays finite
    #[test]
    fn switching_layout_and_cylinder_count_mid_run_stays_finite() {
        let mut sim = twin(json!({}));
        for change in [
            json!({ "exhaustLayout": "2into2" }),
            json!({ "cylinders": 1, "exhaustLayout": "single" }),
            json!({ "cylinders": 2, "exhaustLayout": "2into1" }),
            json!({ "vAngle": 90 }),
            json!({ "firingOffset": 270 }),
            json!({ "firingOffset": null }),
        ] {
            sim.set_engine_json(&change).unwrap();
            let buf = sim.render(FS as usize / 4);
            assert!(buf.iter().all(|v| v.is_finite()), "after {change}");
        }
    }

    /// re-phasing keeps the engine running rather than restarting it
    #[test]
    fn re_phasing_keeps_the_engine_running_rather_than_restarting_it() {
        // Changing the firing offset must not rebuild the cylinders: that would discard their gas
        // state and audibly restart the engine every time the slider moved.
        let mut sim = twin(json!({}));
        let before = sim.cylinders()[0].mass;
        sim.set_engine_json(&json!({ "firingOffset": 300 })).unwrap();
        assert_eq!(sim.cylinders()[0].mass, before);
        let gap = behind(sim.cylinders()[0].angle, sim.cylinders()[1].angle);
        assert!((gap - 300.0).abs() < 0.5, "gap {gap}");
    }

    /// reports one snapshot entry per bank
    #[test]
    fn reports_one_snapshot_entry_per_bank() {
        let mut sim = twin(json!({ "vAngle": 90 }));
        sim.render(512);
        let snap = sim.snapshot();
        assert_eq!(snap.banks.len(), 2);
        let gap = behind(snap.banks[0].crank_angle, snap.banks[1].crank_angle);
        assert!((gap - 450.0).abs() < 0.5, "gap {gap}");
        // The flat fields mirror bank 0, for the single-bank readouts.
        assert_eq!(snap.crank_angle, snap.banks[0].crank_angle);
        assert_eq!(snap.cyl_pressure, snap.banks[0].cyl_pressure);
    }
}

/// The Milwaukee-Eight 121 makes about the real engine's rated 189 N*m (139 lb-ft) at 3500 rpm and 115 hp
/// at 5020, and its phaser, advancing the one cam at low speed, lifts the torque below 3000.
#[test]
fn the_milwaukee_eight_makes_about_the_real_engines_torque_and_power() {
    let torque = |rpm: f64, over: Value| {
        let mut cfg = common::engine_preset("45° V-twin, Harley-Davidson Milwaukee-Eight 121").config.clone();
        let held = json!({ "freeRunning": false, "combustionVariability": 0, "throttle": 1, "rpm": rpm });
        cfg.engine = common::with(&common::with(&cfg.engine, held), over);
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
    let t3500 = torque(3500.0, json!({}));
    assert!((t3500 - 189.0).abs() < 0.1 * 189.0, "{t3500} N*m at 3500 rpm");
    let hp = torque(5020.0, json!({})) * 5020.0 * 2.0 * std::f64::consts::PI / 60.0 / 745.7;
    assert!((hp - 115.0).abs() < 0.1 * 115.0, "{hp} hp at 5020 rpm");
    let (mapped, fixed) = (torque(2500.0, json!({})), torque(2500.0, json!({ "vvtIntakeLow": 0 })));
    assert!(mapped > fixed + 1.5, "{mapped} N*m at 2500 rpm on the cam map, against {fixed} with the cam fixed");
}
