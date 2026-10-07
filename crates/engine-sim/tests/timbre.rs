//! Spectral balance tests.
//!
//! An engine can measure fine on every other test — firing frequency, resonances, passivity
//! — and still sound wrong. Valve-throat turbulence injected as *white* noise is the case in
//! point: the radiation derivative tilts it +6 dB/octave into a rising hiss that dominates
//! everything above 2 kHz, and nothing else notices. So the timbre needs its own guards.

use crate::common;

use common::{FS, hann, magnitude_spectrum};
use engine_sim::EngineSim;
use engine_sim::euler_pipe::{EulerPipe, EulerPipeOptions, HeadPort};
use engine_sim::spec::{PipeSegment, SegmentPartial, make_segment};
use serde_json::{Value, json};
use std::f64::consts::{PI, SQRT_2};

const N: usize = 65536;
const BINHZ: f64 = FS / N as f64;

/// The default engine at 3200 rpm with the fields in `over` replaced, on pipe preset `preset`:
/// one second to settle, then `N` samples.
fn render(over: Value, preset: usize) -> Vec<f32> {
    let mut cfg = common::default_config();
    cfg.engine = common::with(&common::with(&cfg.engine, json!({ "rpm": 3200 })), over);
    cfg.pipe = common::pipe_preset(preset);
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS as usize);
    sim.render(N)
}

/// Energy in the octave centred on `hz`.
fn octave(mag: &[f64], hz: f64) -> f64 {
    let lo = ((hz / SQRT_2 / BINHZ).floor() as usize).max(1);
    let hi = (((hz * SQRT_2) / BINHZ).ceil() as usize).min(mag.len() - 1);
    mag[lo..=hi].iter().map(|m| m * m).sum()
}

const OCTAVES: [f64; 10] = [31.5, 63.0, 125.0, 250.0, 500.0, 1000.0, 2000.0, 4000.0, 8000.0, 16000.0];

/// Energy in each of `OCTAVES`.
struct Bands([f64; OCTAVES.len()]);

impl Bands {
    fn get(&self, hz: f64) -> f64 {
        self.0[OCTAVES.iter().position(|&f| f == hz).expect("an octave centre")]
    }

    fn peak(&self) -> f64 {
        OCTAVES.iter().map(|&f| self.get(f)).fold(f64::NEG_INFINITY, f64::max)
    }
}

fn bands(buf: &[f32]) -> Bands {
    let mag = magnitude_spectrum(&hann(buf), N);
    Bands(OCTAVES.map(|f| octave(&mag, f)))
}

fn db(a: f64, b: f64) -> f64 {
    10.0 * (a / b).log10()
}

mod the_exhaust_note_is_not_dominated_by_broadband_hiss {
    use super::*;

    /// peaks in the low octaves, where a single-cylinder engine belongs
    #[test]
    fn peaks_in_the_low_octaves_where_a_single_cylinder_engine_belongs() {
        let b = bands(&render(json!({}), 1));
        let loudest = OCTAVES.iter().copied().reduce(|a, f| if b.get(f) > b.get(a) { f } else { a }).unwrap();
        // At 3200 rpm the firing frequency is 26.7 Hz, so the note's energy sits in the
        // 125-500 Hz octaves. A peak up at 2-4 kHz would mean noise had taken over.
        assert!(loudest >= 125.0, "loudest octave {loudest} Hz");
        assert!(loudest <= 500.0, "loudest octave {loudest} Hz");
    }

    /// rolls off steeply above 2 kHz rather than holding a flat hiss
    #[test]
    fn rolls_off_steeply_above_2_khz_rather_than_holding_a_flat_hiss() {
        let b = bands(&render(json!({}), 1));
        let peak = b.peak();
        // Measures about -14 dB at 4 kHz and -36 at 8 kHz; a white throat source sits well
        // above the 4 kHz limit, so this catches it.
        //
        // The 4 kHz limit leaves room for nonlinear steepening in the Euler solver, which
        // genuinely creates harmonics a linear model could not produce at any amplitude. What
        // matters is that that content is *harmonic* (brassiness) rather than broadband hiss,
        // and the 'band-limited, not white' test below checks that mechanism directly.
        assert!(db(b.get(4000.0), peak) < -12.0, "4 kHz at {} dB", db(b.get(4000.0), peak));
        assert!(db(b.get(8000.0), peak) < -28.0, "8 kHz at {} dB", db(b.get(8000.0), peak));
    }

    /// falls monotonically from 1 kHz upward
    #[test]
    fn falls_monotonically_from_1_khz_upward() {
        let b = bands(&render(json!({}), 1));
        for (lo, hi) in [(1000.0, 2000.0), (2000.0, 4000.0), (4000.0, 8000.0), (8000.0, 16000.0)] {
            assert!(b.get(hi) < b.get(lo), "{hi} Hz should be quieter than {lo} Hz");
        }
    }

    /// throat turbulence stays a texture, never the loudest thing in the room
    #[test]
    fn throat_turbulence_stays_a_texture_never_the_loudest_thing_in_the_room() {
        let off = bands(&render(json!({ "throatNoise": 0 }), 1));
        let full = bands(&render(json!({ "throatNoise": 1 }), 1));
        // Turning it to maximum may colour the upper mids, but it must not transform the
        // spectrum. White turbulence, tilted up by the radiation derivative, adds more than
        // 10 dB at 4 kHz at only *half* power.
        assert!(db(full.get(4000.0), off.get(4000.0)) < 8.0);
        assert!(db(full.get(8000.0), off.get(8000.0)) < 8.0);
        // And it must not shift the low end at all.
        assert!(db(full.get(250.0), off.get(250.0)).abs() < 1.5);
    }

    /// turbulence is band-limited, not white
    #[test]
    fn turbulence_is_band_limited_not_white() {
        // Isolate the noise by differencing the spectra with it off and at full power. If
        // the injected noise were white, the radiation derivative would make this
        // difference *grow* with frequency all the way to Nyquist.
        let off = bands(&render(json!({ "throatNoise": 0, "mechNoise": 0 }), 1));
        let on = bands(&render(json!({ "throatNoise": 1, "mechNoise": 0 }), 1));
        let added = |f: f64| (on.get(f) - off.get(f)).max(1e-30);
        // At or equal to, because above 8 kHz the noise adds nothing measurable: the two runs differ
        // there by a few percent either way, so both octaves can clamp to the floor together.
        assert!(added(8000.0) < added(2000.0));
        assert!(added(16000.0) <= added(8000.0));
    }

    /// holds across every preset and speed
    #[test]
    fn holds_across_every_preset_and_speed() {
        for (p, preset) in common::presets().pipe_presets.iter().enumerate() {
            // No preset is excepted, the expansion chamber included. Its treble is not a
            // quasi-1D shortcoming to be allowed for: excess energy there comes from fixed-rate
            // numerical artefacts such as batching the wall heat transfer every 16 samples, which
            // injects a periodic energy perturbation at exactly 48000/16 = 3000 Hz. That would
            // show in every preset and be loudest here only because this geometry has a high-Q
            // tailpipe mode near 3 kHz to amplify it.

            for rpm in [1500, 3200, 6500] {
                let b = bands(&render(json!({ "rpm": rpm }), p));
                let level = db(b.get(16000.0), b.peak());
                assert!(level < -26.0, "{} at {rpm} rpm has too much energy at 16 kHz: {level} dB", preset.name);
            }
        }
    }
}

mod the_exhaust_port_is_part_of_the_acoustic_system {
    use super::*;

    fn pipe(length: f64, d: f64) -> PipeSegment {
        make_segment(SegmentPartial { length: Some(length), d_in: Some(d), ..Default::default() })
    }

    /// lengthening the port lowers the resonance, because tuning starts at the valve
    #[test]
    fn lengthening_the_port_lowers_the_resonance_because_tuning_starts_at_the_valve() {
        let first = |port_length: f64| {
            let wg = EulerPipe::new(
                &[pipe(0.5, 0.042)],
                FS,
                293.0,
                &EulerPipeOptions {
                    port: Some(HeadPort { length: port_length, diameter: 0.034 }),
                    ..Default::default()
                },
            );
            // Quarter-wave of the whole duct, port included.
            343.0 / (4.0 * wg.total_length)
        };
        assert!(first(0.15) < first(0.01));
    }

    /// reports how much of the duct is port, so the display can skip it
    #[test]
    fn reports_how_much_of_the_duct_is_port_so_the_display_can_skip_it() {
        let wg = EulerPipe::new(
            &[pipe(0.5, 0.042)],
            FS,
            900.0,
            &EulerPipeOptions { port: Some(HeadPort { length: 0.055, diameter: 0.034 }), ..Default::default() },
        );
        assert!(wg.port_cells > 0);
        assert!(wg.port_cells < wg.n);

        let mut taps = [0.0f32; 128];
        wg.sample_pressure(&mut taps);
        assert!(taps.iter().all(|t| t.is_finite()));

        let bare = EulerPipe::new(&[pipe(0.5, 0.042)], FS, 900.0, &EulerPipeOptions::default());
        assert_eq!(bare.port_cells, 0);
        // The port really does add duct.
        assert!(wg.n > bare.n, "{} vs {}", wg.n, bare.n);
    }
}

mod plane_wave_validity_limit {
    use super::*;

    fn cutoff(dia: f64) -> f64 {
        let seg = make_segment(SegmentPartial { length: Some(0.6), d_in: Some(dia), ..Default::default() });
        EulerPipe::new(&[seg], FS, 900.0, &EulerPipeOptions::default()).plane_wave_cutoff_rad
    }

    /// a wide mouth cuts on sooner than a narrow one
    #[test]
    fn a_wide_mouth_cuts_on_sooner_than_a_narrow_one() {
        // omega = 1.84 c / a, so a bigger radius means a lower cut-on frequency.
        assert!(cutoff(0.12) < cutoff(0.03));
    }

    /// sits well above the fundamentals it must not touch
    #[test]
    fn sits_well_above_the_fundamentals_it_must_not_touch() {
        let hz = cutoff(0.1) / (2.0 * PI);
        assert!(hz > 1500.0, "{hz} Hz");
        assert!(hz < 12000.0, "{hz} Hz");
    }
}

mod no_fixed_rate_numerical_artefacts {
    use super::*;

    /// No stochastic sources at all, at 1500 rpm.
    fn quiet() -> Value {
        json!({ "rpm": 1500, "mechNoise": 0, "throatNoise": 0, "combustionVariability": 0 })
    }

    /// Energy in a narrow band around `hz`, relative to total, for a preset with no
    /// stochastic sources at all. Any tone that survives this has to come from the solver.
    fn tone_peak(preset: usize, hz: f64) -> f64 {
        let spec = magnitude_spectrum(&hann(&render(quiet(), preset)), N);
        let at = |f: f64| {
            let lo = ((f - 40.0) / BINHZ).floor() as usize;
            let hi = ((f + 40.0) / BINHZ).ceil() as usize;
            spec[lo..=hi].iter().map(|m| m * m).sum::<f64>()
        };
        // Compare the suspect band with its neighbours: a solver artefact is a narrow spike
        // sitting on top of whatever the engine is doing.
        at(hz) / (0.5 * (at(hz * 0.72) + at(hz * 1.38))).max(1e-30)
    }

    /// nothing rings at the heat-transfer batch rate
    #[test]
    fn nothing_rings_at_the_heat_transfer_batch_rate() {
        // Batching the wall heat transfer every N samples injects a periodic energy
        // perturbation at sampleRate/N — at N = 16 that is exactly 3000 Hz. It would appear in
        // every preset and dominate the expansion chamber, which has a high-Q tailpipe mode there.
        //
        // Deliberately checked on that preset, since it is the one that amplifies it most, as well as
        // the open header and the street muffler, with every stochastic source disabled so nothing
        // can mask it.
        for preset in [0, 2, 3] {
            let r = tone_peak(preset, FS / 16.0);
            assert!(r < 8.0, "preset {preset} has a spike at the heat batch rate: {r}");
        }
    }

    /// the expansion chamber is no louder in the treble than the others
    #[test]
    fn the_expansion_chamber_is_no_louder_in_the_treble_than_the_others() {
        // Its tailpipe mode amplifies any fixed-rate artefact, so it is where one would show first.
        let share = |preset: usize| {
            let b = bands(&render(quiet(), preset));
            let total: f64 = OCTAVES.iter().map(|&f| b.get(f)).sum();
            (b.get(2000.0) + b.get(4000.0) + b.get(8000.0) + b.get(16000.0)) / total
        };
        let chamber = share(2);
        assert!(chamber < 0.1, "chamber treble share {chamber}");
        assert!(chamber < share(1) + 0.08, "chamber treble share {chamber}");
    }
}
