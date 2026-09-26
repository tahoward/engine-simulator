//! The cross-wise modes of a chamber: the section eigenproblem against shapes with known answers, and
//! the coupled duct against what a wide, offset-pipe can should do to the sound.

mod common;

use common::FS;
use engine_sim::cross_modes::{PipeOpening, mode_cutoff_k, section_modes};
use engine_sim::euler_pipe::{EulerPipe, EulerPipeOptions, ValveState};
use engine_sim::spec::{ChamberSection, PipeSegment, Section, SegmentKind, SegmentPartial, gas, make_segment};
use std::f64::consts::PI;

fn opening(offset: f64, diameter: f64) -> [PipeOpening; 1] {
    [PipeOpening { offset, diameter }]
}

mod section_modes {
    use super::*;

    /// finds the Bessel roots of a circle
    #[test]
    fn finds_the_bessel_roots_of_a_circle() {
        let r = 0.1;
        let s = Section { section: ChamberSection::Round, width: 2.0 * r, height: 2.0 * r };
        let modes = section_modes(&s, 6.0 / r, &opening(0.05, 0.02));
        // Only modes even about the width axis: cos(m theta) for m = 1, 2, then the first radial mode.
        let want = [1.8412, 3.0542, 3.8317];
        assert!(modes.len() >= want.len(), "{} modes", modes.len());
        for (m, w) in modes.iter().zip(want) {
            let got = m.k * r;
            assert!((got - w).abs() / w < 0.005, "k R = {got}, want {w}");
        }
    }

    /// finds pi/W across a rounded rectangle, give or take the little the corners change
    #[test]
    fn finds_pi_w_across_a_rounded_rectangle_give_or_take_the_little_the_corners_change() {
        let s = Section { section: ChamberSection::Rect, width: 0.3, height: 0.12 };
        let modes = section_modes(&s, 40.0, &opening(0.1, 0.04));
        let first = modes[0].k;
        let want = PI / 0.3;
        assert!((first - want).abs() / want < 0.03, "k = {first}, want {want}");
    }

    /// puts an ellipse between the circles of its two axes
    #[test]
    fn puts_an_ellipse_between_the_circles_of_its_two_axes() {
        let s = Section { section: ChamberSection::Oval, width: 0.3, height: 0.15 };
        let k = section_modes(&s, 40.0, &opening(0.1, 0.04))[0].k;
        assert!(k > 1.8412 / 0.15, "k = {k}");
        assert!(k < 1.8412 / 0.075, "k = {k}");
    }

    /// leaves the antisymmetric modes to an offset pipe
    #[test]
    fn leaves_the_antisymmetric_modes_to_an_offset_pipe() {
        let s = Section { section: ChamberSection::Oval, width: 0.3, height: 0.15 };
        let centred = section_modes(&s, 40.0, &opening(0.0, 0.04)).swap_remove(0);
        let offset = section_modes(&s, 40.0, &opening(0.1, 0.04)).swap_remove(0);
        assert!(centred.at_pipes[0].abs() < 1e-6, "centred {}", centred.at_pipes[0]);
        assert!(offset.at_pipes[0].abs() > 0.5, "offset {}", offset.at_pipes[0]);
    }
}

mod a_duct_with_a_wide_offset_pipe_can {
    use super::*;

    fn pipe(length: f64, d: f64) -> PipeSegment {
        make_segment(SegmentPartial { length: Some(length), d_in: Some(d), ..Default::default() })
    }

    fn can(offset: f64) -> Vec<PipeSegment> {
        vec![
            pipe(0.4, 0.042),
            make_segment(SegmentPartial {
                kind: Some(SegmentKind::Chamber),
                length: Some(0.4),
                d_in: Some(0.042),
                d_out: Some(0.3),
                section: Some(ChamberSection::Oval),
                height: Some(0.12),
                offset_in: Some(offset),
                offset_out: Some(-offset),
                ..Default::default()
            }),
            pipe(0.4, 0.042),
        ]
    }

    fn options() -> EulerPipeOptions {
        EulerPipeOptions { cell_size: Some(0.035), single_step: Some(true), ..Default::default() }
    }

    /// keeps no modes for a round can with centred pipes
    #[test]
    fn keeps_no_modes_for_a_round_can_with_centred_pipes() {
        let p = EulerPipe::new(
            &[make_segment(SegmentPartial {
                kind: Some(SegmentKind::Chamber),
                length: Some(0.34),
                d_in: Some(0.042),
                d_out: Some(0.13),
                ..Default::default()
            })],
            FS,
            900.0,
            &options(),
        );
        assert!(p.cross_modes.is_none());
    }

    /// keeps modes for an offset oval, all of them inside the grid band
    #[test]
    fn keeps_modes_for_an_offset_oval_all_of_them_inside_the_grid_band() {
        let p = EulerPipe::new(&can(0.1), FS, 900.0, &options());
        let modes = p.cross_modes.as_ref().expect("an offset oval keeps cross-wise modes");
        let cap = (mode_cutoff_k(p.dx) * (gas::GAMMA_EXH * gas::R * 1400.0).sqrt()) / (2.0 * PI);
        for f in modes.frequencies() {
            assert!(f < cap, "mode at {f} Hz above the grid's {cap} Hz");
        }
    }

    /// Ring the duct with one valve pulse and compare the mouth's spectrum with the pipes offset and
    /// centred. Below the first cross-wise mode the two are the same muffler, since only the plane wave
    /// exists there. From it upwards the offset pipes drive the modes, which put peaks and notches in the
    /// transmission, so the spectra part company by many dB.
    #[test]
    fn changes_the_sound_from_the_first_cross_wise_mode_up_and_not_below_it() {
        let ring = |offset: f64| {
            let mut p =
                EulerPipe::new(&can(offset), FS, 900.0, &EulerPipeOptions { heat_transfer: Some(false), ..options() });
            let mut out = vec![0.0; 32768];
            for (k, o) in out.iter_mut().enumerate() {
                let pulse = k < 30;
                let valve = ValveState {
                    throat_area: if pulse { 3e-4 } else { 0.0 },
                    cyl_pressure: if pulse { 1.5e5 } else { gas::P_AMB },
                    cyl_temp: 900.0,
                    ..Default::default()
                };
                *o = p.advance(1.0 / FS, &valve).mouth_flow;
            }
            let f0 = p.cross_modes.as_ref().map_or(0.0, |m| m.frequencies().first().copied().unwrap_or(0.0));
            (out, f0)
        };
        let (offset, f0) = ring(0.1);
        let (centred, _) = ring(0.0);
        // c / 2W for a 300 mm can is 800-1000 Hz at these temperatures; the curved wall lifts it a little.
        assert!(f0 > 800.0, "f0 {f0}");
        assert!(f0 < 1300.0, "f0 {f0}");

        let mean_diff_db = |lo: f64, hi: f64| {
            let mut sum = 0.0;
            let mut n = 0;
            let mut f = lo;
            while f < hi {
                let a = band_energy(&offset, f, f + 25.0);
                let b = band_energy(&centred, f, f + 25.0);
                sum += (10.0 * (a / b).log10()).abs();
                n += 1;
                f += 25.0;
            }
            sum / n as f64
        };
        let below = mean_diff_db(300.0, f0 * 0.85);
        let above = mean_diff_db(f0 * 0.9, f0 * 1.35);
        // Measured at 1.2 dB below and 3.2 dB above. Below is not zero because a mode under its own
        // frequency still loads the pipe opening a little, as extra mass.
        assert!(above > 2.5, "above {above} dB");
        assert!(above > 2.0 * below, "above {above} dB, below {below} dB");
    }
}

/// Energy of `x` in the DFT bins from `lo` up to `hi` Hz, on a Hann window.
fn band_energy(x: &[f64], lo: f64, hi: f64) -> f64 {
    let n_len = x.len();
    let nf = n_len as f64;
    let mut e = 0.0;
    let k0 = ((lo * nf) / FS).ceil().max(1.0) as usize;
    let k1 = (((hi * nf) / FS).ceil() - 1.0).max(k0 as f64) as usize;
    for k in k0..=k1 {
        let mut re = 0.0;
        let mut im = 0.0;
        for (n, &v) in x.iter().enumerate() {
            let w = 0.5 - 0.5 * ((2.0 * PI * n as f64) / (nf - 1.0)).cos();
            let t = (2.0 * PI * k as f64 * n as f64) / nf;
            re += w * v * t.cos();
            im -= w * v * t.sin();
        }
        e += re * re + im * im;
    }
    e
}
