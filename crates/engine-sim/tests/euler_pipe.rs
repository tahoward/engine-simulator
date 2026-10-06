//! Tests for the quasi-1D Euler solver.
//!
//! Two families here, and both matter. The first is standard CFD verification: the Sod
//! shock tube against its exact solution, TVD behaviour (no overshoot), and conservation.
//! The second is the acoustics a linear waveguide guarantees *by construction* and this
//! solver has to earn numerically — resonating at c/4L, reflecting off area changes, and
//! staying stable. An approximate nonlinear method is only worth having over an exact
//! linear one if the linear behaviour survives.

mod common;

use common::{FS, find_peaks, hann, magnitude_spectrum};
use engine_sim::dsp::Noise;
use engine_sim::euler_pipe::{
    EulerPipe, EulerPipeOptions, GAMMA, HeadPort, SlopeLimiter, ValveState, darcy_factor, launch_radius,
    limit_area_ratio,
};
use engine_sim::math;
use engine_sim::spec::{PipeMaterial, PipeSegment, SegmentKind, SegmentPartial, gas, make_segment, speed_of_sound_exh};
use std::f64::consts::PI;

const FFT: usize = 32768;

/// A segment of `kind`, `length` and inlet diameter `d_in`, with its outlet diameter `d_out` or the default.
fn seg(kind: SegmentKind, length: f64, d_in: f64, d_out: Option<f64>) -> PipeSegment {
    make_segment(SegmentPartial {
        kind: Some(kind),
        length: Some(length),
        d_in: Some(d_in),
        d_out,
        ..Default::default()
    })
}

/// A plain pipe of `length` and diameter `d`.
fn pipe(length: f64, d: f64) -> PipeSegment {
    seg(SegmentKind::Pipe, length, d, None)
}

/// A closed-both-ends uniform duct, for pure gas-dynamics tests.
fn tube(cells: usize, length: f64, limiter: SlopeLimiter) -> EulerPipe {
    EulerPipe::new(
        &[pipe(length, 0.05)],
        FS,
        gas::T_AMB,
        &EulerPipeOptions {
            cell_size: Some(length / cells as f64),
            max_cells: Some(4096),
            radiate: Some(false),
            heat_transfer: Some(false),
            linear_damping: Some(0.0),
            darcy_friction: Some(0.0),
            limiter: Some(limiter),
            ..Default::default()
        },
    )
}

const SHUT: ValveState = ValveState {
    throat_area: 0.0,
    cyl_pressure: gas::P_AMB,
    cyl_temp: gas::T_AMB,
    cyl_gamma: gas::GAMMA_EXH,
    extra_mass_flow: 0.0,
};

/// A valve with the exhaust gas's gamma and no extra flow.
fn valve(throat_area: f64, cyl_pressure: f64, cyl_temp: f64) -> ValveState {
    ValveState { throat_area, cyl_pressure, cyl_temp, ..Default::default() }
}

/// Number of integers `k` from 0 with `k < x`.
fn upto(x: f64) -> usize {
    x.ceil() as usize
}

mod shock_capturing {
    use super::*;

    /// Sod's problem, the standard verification case. Exact solution computed here by
    /// Newton iteration on the star pressure.
    fn sod_exact(x: f64, t: f64, g: f64) -> f64 {
        let (r_l, p_l, r_r, p_r) = (1.0, 1.0, 0.125, 0.1);
        let c_l = ((g * p_l) / r_l).sqrt();
        let c_r = ((g * p_r) / r_r).sqrt();
        let f = |p: f64, r_k: f64, p_k: f64, c_k: f64| {
            if p > p_k {
                let a = 2.0 / ((g + 1.0) * r_k);
                let b = (p_k * (g - 1.0)) / (g + 1.0);
                return (p - p_k) * (a / (p + b)).sqrt();
            }
            ((2.0 * c_k) / (g - 1.0)) * ((p / p_k).powf((g - 1.0) / (2.0 * g)) - 1.0)
        };
        let fd = |p: f64, r_k: f64, p_k: f64, c_k: f64| {
            if p > p_k {
                let a = 2.0 / ((g + 1.0) * r_k);
                let b = (p_k * (g - 1.0)) / (g + 1.0);
                let s = (a / (p + b)).sqrt();
                return s * (1.0 - (p - p_k) / (2.0 * (p + b)));
            }
            (1.0 / (r_k * c_k)) * (p / p_k).powf(-(g + 1.0) / (2.0 * g))
        };
        let mut p = 0.5 * (p_l + p_r);
        for _ in 0..80 {
            let next = (p
                - (f(p, r_l, p_l, c_l) + f(p, r_r, p_r, c_r)) / (fd(p, r_l, p_l, c_l) + fd(p, r_r, p_r, c_r)))
            .max(1e-10);
            if (next - p).abs() < 1e-15 {
                p = next;
                break;
            }
            p = next;
        }
        let u_star = 0.5 * (f(p, r_r, p_r, c_r) - f(p, r_l, p_l, c_l));
        let s = if t > 0.0 { (x - 0.5) / t } else { 0.0 };
        if s <= u_star {
            let c_star = c_l * (p / p_l).powf((g - 1.0) / (2.0 * g));
            if s < -c_l {
                return r_l;
            }
            if s > u_star - c_star {
                return r_l * (p / p_l).powf(1.0 / g);
            }
            // Inside the left rarefaction fan. The (g-1)/2 factor on s is essential — without
            // it the fan head lands at 1.72*cL instead of cL.
            let c = (2.0 / (g + 1.0)) * (c_l - ((g - 1.0) / 2.0) * s);
            return r_l * (c / c_l).powf(2.0 / (g - 1.0));
        }
        let s_r = c_r * (((g + 1.0) / (2.0 * g)) * (p / p_r) + (g - 1.0) / (2.0 * g)).sqrt();
        if s >= s_r {
            return r_r;
        }
        r_r * ((p / p_r + (g - 1.0) / (g + 1.0)) / (((g - 1.0) / (g + 1.0)) * (p / p_r) + 1.0))
    }

    struct SodResult {
        l1: f64,
        overshoot: f64,
        undershoot: f64,
    }

    /// Run Sod in solver units. The solver fixes gamma at the exhaust value, so the exact
    /// solution is evaluated with the same gamma rather than the textbook 1.4.
    fn sod(cells: usize, limiter: SlopeLimiter) -> SodResult {
        // Scale the classic initial data up to real pressures so nothing hits the floors.
        const SCALE: f64 = 1e5;
        let mut p = tube(cells, 1.0, limiter);
        for i in 0..p.n {
            let x = (i as f64 + 0.5) * p.dx;
            if x < 0.5 {
                p.set_primitive(i, 1.0, 0.0, SCALE);
            } else {
                p.set_primitive(i, 0.125, 0.0, 0.1 * SCALE);
            }
        }
        // Time is scaled with the sound speed: c ~ sqrt(p/rho), so t_scaled = t/sqrt(SCALE).
        let t_end = 0.2 / math::sqrt(SCALE);
        let mut t = 0.0;
        while t < t_end {
            let h = math::min(2e-5, t_end - t);
            p.advance(h, &SHUT);
            t += h;
        }
        let mut l1 = 0.0;
        let mut max_rho = 0.0f64;
        let mut min_rho = f64::INFINITY;
        for i in 0..p.n {
            let x = (i as f64 + 0.5) * p.dx;
            l1 += (p.density_at(i) - sod_exact(x, 0.2, gas::GAMMA_EXH)).abs() * p.dx;
            max_rho = max_rho.max(p.density_at(i));
            min_rho = min_rho.min(p.density_at(i));
        }
        SodResult { l1, overshoot: max_rho - 1.0, undershoot: 0.125 - min_rho }
    }

    /// matches the exact Sod solution and converges
    #[test]
    fn matches_the_exact_sod_solution_and_converges() {
        let coarse = sod(100, SlopeLimiter::Mc);
        let fine = sod(400, SlopeLimiter::Mc);
        assert!(coarse.l1 < 0.03, "coarse L1 {}", coarse.l1);
        assert!(fine.l1 < coarse.l1, "fine L1 {} vs coarse {}", fine.l1, coarse.l1);
        // L1 convergence on a discontinuous problem is capped near first order because the
        // limiter drops to first order at the shock and the contact.
        let order = (coarse.l1 / fine.l1).ln() / 4f64.ln();
        assert!(order > 0.5, "order {order}");
    }

    /// is TVD: no limiter overshoots the initial data
    #[test]
    fn is_tvd_no_limiter_overshoots_the_initial_data() {
        // This is the whole point of gradient limiting. An unlimited second-order scheme
        // rings at the shock, and those oscillations are audible as well as unphysical.
        for limiter in [SlopeLimiter::Minmod, SlopeLimiter::Mc, SlopeLimiter::VanLeer] {
            let r = sod(200, limiter);
            assert!(r.overshoot < 1e-6, "{limiter:?} overshoot {}", r.overshoot);
            assert!(r.undershoot < 1e-6, "{limiter:?} undershoot {}", r.undershoot);
        }
    }

    /// mc is less diffusive than minmod, as the limiter theory says
    #[test]
    fn mc_is_less_diffusive_than_minmod_as_the_limiter_theory_says() {
        let mc = sod(200, SlopeLimiter::Mc).l1;
        let minmod = sod(200, SlopeLimiter::Minmod).l1;
        assert!(mc < minmod, "mc {mc} minmod {minmod}");
    }
}

mod nonlinear_steepening_the_reason_for_the_whole_exercise {
    use super::*;

    const LAMBDA: f64 = 0.4;

    /// Steepness of a travelling wave after propagating `travel` metres, as a multiple of
    /// the steepness the same wave would have if it stayed sinusoidal.
    ///
    /// A long duct filled with many wavelengths, measured only in the middle half so that
    /// nothing reflected from either end can reach the measurement window. Measuring a
    /// short burst in a closed tube instead does not work: reflections off the walls
    /// generate harmonics of their own, amplitude-independently, which swamps the effect
    /// being tested.
    fn steepness_ratio(amplitude_bar: f64, travel: f64) -> f64 {
        let length = 4.0;
        let mut p = EulerPipe::new(
            &[pipe(length, 0.05)],
            FS,
            gas::T_AMB,
            &EulerPipeOptions {
                cell_size: Some(LAMBDA / 100.0),
                max_cells: Some(4096),
                radiate: Some(false),
                heat_transfer: Some(false),
                linear_damping: Some(0.0),
                darcy_friction: Some(0.0),
                max_substeps: Some(4096),
                ..Default::default()
            },
        );
        let rho0 = gas::P_AMB / (gas::R * gas::T_AMB);
        let c0 = speed_of_sound_exh(gas::T_AMB);
        let amp = amplitude_bar * 1e5;
        for i in 0..p.n {
            let x = (i as f64 + 0.5) * p.dx;
            let dp = amp * math::sin((2.0 * PI * x) / LAMBDA);
            // A rightward-travelling simple wave.
            p.set_primitive(i, rho0 + dp / (c0 * c0), dp / (rho0 * c0), gas::P_AMB + dp);
        }

        let t_end = travel / c0;
        let mut t = 0.0;
        while t < t_end {
            let h = math::min(2e-5, t_end - t);
            p.advance(h, &SHUT);
            t += h;
        }

        // Steepest gradient and peak amplitude, both from the middle half only.
        let lo = (p.n as f64 * 0.25).floor() as usize;
        let hi = (p.n as f64 * 0.75).floor() as usize;
        let mut max_grad = 0.0f64;
        let mut peak = 0.0f64;
        for i in lo..hi {
            max_grad = max_grad.max((p.pressure_at(i + 1) - p.pressure_at(i)).abs() / p.dx);
            peak = peak.max((p.pressure_at(i) - gas::P_AMB).abs());
        }
        // A pure sine of this amplitude and wavelength has max gradient 2*pi*A/lambda.
        max_grad / ((2.0 * PI * peak) / LAMBDA)
    }

    /// a large-amplitude wave steepens; a small one stays sinusoidal
    #[test]
    fn a_large_amplitude_wave_steepens_a_small_one_stays_sinusoidal() {
        let quiet = steepness_ratio(0.002, 0.8);
        let loud = steepness_ratio(0.8, 0.8);
        // A linear waveguide gives exactly 1.0 here at every amplitude, which is why it
        // cannot make an open pipe sound brassy.
        assert!(quiet < 1.15, "quiet {quiet}");
        assert!(loud > 1.5, "loud {loud}");
    }

    /// steepening grows with amplitude
    #[test]
    fn steepening_grows_with_amplitude() {
        let a = steepness_ratio(0.1, 0.8);
        let b = steepness_ratio(0.8, 0.8);
        assert!(b > a, "{b} vs {a}");
    }

    /// steepening grows with distance travelled
    #[test]
    fn steepening_grows_with_distance_travelled() {
        let far = steepness_ratio(0.5, 1.0);
        let near = steepness_ratio(0.5, 0.15);
        assert!(far > near, "{far} vs {near}");
    }
}

mod conservation {
    use super::*;

    /// a sealed duct conserves mass and energy
    #[test]
    fn a_sealed_duct_conserves_mass_and_energy() {
        let mut p = tube(120, 1.0, SlopeLimiter::Mc);
        // Put a pressure bump in the middle so waves slosh around.
        for i in 40..60 {
            p.set_primitive(i, 1.4, 0.0, 1.6 * gas::P_AMB);
        }
        let m0 = p.total_mass();
        let e0 = p.total_energy();
        for _ in 0..400 {
            p.advance(1.0 / FS, &SHUT);
        }
        let dm = (p.total_mass() - m0).abs() / m0;
        let de = (p.total_energy() - e0).abs() / e0;
        assert!(dm < 1e-9, "mass drift {dm}");
        assert!(de < 1e-9, "energy drift {de}");
    }

    /// Acoustic energy, i.e. the deviation from a quiescent duct.
    ///
    /// Total energy is the wrong thing to watch for stability. It is dominated by internal
    /// energy — order 1e5 J/m^3 — so an acoustic field can grow by three orders of
    /// magnitude while total energy stays conserved to 1e-9. A conservation test alone
    /// happily passes a duct that is screaming.
    fn acoustic_energy(p: &EulerPipe) -> f64 {
        let mut e = 0.0;
        for i in 0..p.n {
            let dp = p.pressure_at(i) - gas::P_AMB;
            let u = p.velocity_at(i);
            e += (dp * dp + p.density_at(i) * 1e5 * u * u) * p.area_of(i);
        }
        e
    }

    /// a cavity between two area changes does not pump itself
    #[test]
    fn a_cavity_between_two_area_changes_does_not_pump_itself() {
        // This guards two coupled requirements of the Hancock predictor. The area source has
        // to be in the predictor as well as the corrector, *and* the predictor's fluxes have
        // to be area-weighted so it stays well balanced at rest. Omitting the source lets the
        // acoustic energy here grow by orders of magnitude; adding it without area-weighting
        // the fluxes instead invents momentum at rest and makes the transient tens of dB too
        // loud. A single expansion or contraction hides both — only a cavity traps the error.
        for segments in [
            vec![pipe(0.35, 0.04), pipe(0.3, 0.14), pipe(0.35, 0.04)],
            vec![pipe(0.5, 0.04), seg(SegmentKind::Chamber, 0.3, 0.04, Some(0.14)), pipe(0.2, 0.04)],
        ] {
            let mut p = EulerPipe::new(
                &segments,
                FS,
                gas::T_AMB,
                &EulerPipeOptions {
                    cell_size: Some(0.008),
                    max_cells: Some(512),
                    heat_transfer: Some(false),
                    // Production damping. The scheme retains a small second-order inconsistency in a
                    // near-lossless sealed cavity, whose growth rate measures about 1.4 /s — two
                    // orders of magnitude below this, so it stays comfortably suppressed.
                    linear_damping: Some(150.0),
                    darcy_friction: Some(0.0),
                    radiate: Some(false),
                    max_substeps: Some(64),
                    ..Default::default()
                },
            );
            let mid = p.n / 2;
            let rho = p.density_at(0);
            p.set_primitive(mid, rho, 0.0, gas::P_AMB * 1.05);

            let mut early = 0.0;
            let mark = (FS * 0.03).floor() as usize;
            for k in 0..upto(FS * 0.35) {
                p.advance(1.0 / FS, &SHUT);
                if k == mark {
                    early = acoustic_energy(&p);
                }
            }
            assert!(early > 0.0);
            let growth = acoustic_energy(&p) / early;
            assert!(growth < 1.1, "acoustic energy grew {growth}x");
        }
    }

    /// is well balanced: a duct at rest stays at rest whatever its shape
    #[test]
    fn is_well_balanced_a_duct_at_rest_stays_at_rest_whatever_its_shape() {
        // If the flux difference and the p dA source do not cancel exactly at rest, varying
        // area spontaneously generates flow.
        let mut p = EulerPipe::new(
            &[
                pipe(0.3, 0.035),
                seg(SegmentKind::Cone, 0.3, 0.035, Some(0.12)),
                seg(SegmentKind::Chamber, 0.3, 0.12, Some(0.2)),
                seg(SegmentKind::Cone, 0.2, 0.12, Some(0.03)),
            ],
            FS,
            900.0,
            &EulerPipeOptions {
                heat_transfer: Some(false),
                radiate: Some(false),
                max_substeps: Some(64),
                ..Default::default()
            },
        );
        for _ in 0..2000 {
            p.advance(1.0 / FS, &SHUT);
        }
        let mut max_u = 0.0f64;
        for i in 0..p.n {
            max_u = max_u.max(p.velocity_at(i).abs());
        }
        assert!(max_u < 1e-6, "max |u| {max_u}");
    }

    /// a sealed duct does not gain energy, so it cannot run away
    #[test]
    fn a_sealed_duct_does_not_gain_energy_so_it_cannot_run_away() {
        let mut p = tube(120, 1.0, SlopeLimiter::Mc);
        for i in 0..p.n {
            let x = (i as f64 + 0.5) * p.dx;
            p.set_primitive(i, 1.2, 60.0 * math::sin(12.0 * x), gas::P_AMB * (1.0 + 0.3 * math::sin(9.0 * x)));
        }
        let e0 = p.total_energy();
        for _ in 0..3000 {
            p.advance(1.0 / FS, &SHUT);
        }
        assert!(p.total_energy() < e0 * 1.0001, "energy {} from {e0}", p.total_energy());
        for i in 0..p.n {
            assert!(p.pressure_at(i).is_finite());
            assert!(p.pressure_at(i) > 0.0);
        }
    }
}

mod the_linear_acoustics_a_waveguide_gets_for_free {
    use super::*;

    /// Impulse response at the mouth of a closed-open duct driven by a flow pulse.
    fn impulse_response(p: &mut EulerPipe, n: usize, kick: f64) -> Vec<f32> {
        let rho0 = p.density_at(0);
        // Inject a short velocity pulse into the first cell.
        p.set_primitive(0, rho0, kick, gas::P_AMB);
        (0..n).map(|_| p.advance(1.0 / FS, &SHUT).mouth_flow as f32).collect()
    }

    /// Build a test duct with light damping so its resonances are sharp.
    fn duct(segments: &[PipeSegment], port_temp: f64) -> EulerPipe {
        EulerPipe::new(
            segments,
            FS,
            port_temp,
            &EulerPipeOptions {
                cell_size: Some(0.008),
                max_cells: Some(512),
                // Heat transfer off so the temperature field stays as initialised.
                heat_transfer: Some(false),
                linear_damping: Some(6.0),
                darcy_friction: Some(0.0),
                max_substeps: Some(64),
                ..Default::default()
            },
        )
    }

    /// The peak nearest `want`.
    fn nearest(peaks: &[common::Peak], want: f64) -> common::Peak {
        peaks
            .iter()
            .copied()
            .reduce(|best, q| if (q.hz - want).abs() < (best.hz - want).abs() { q } else { best })
            .unwrap()
    }

    /// Measured fundamental, located by searching a window around the solver's own
    /// quarter-wave prediction.
    ///
    /// Taking "the strongest peak in a wide range" is fragile: which mode dominates depends
    /// on the geometry, so it can silently compare mode 1 in one case against mode 3 in
    /// another.
    fn fundamental_of(segments: &[PipeSegment], port_temp: f64) -> f64 {
        let mut p = duct(segments, port_temp);
        let predicted = p.quarter_wave_hz();
        let mag = magnitude_spectrum(&impulse_response(&mut p, FFT, 2.0), FFT);
        let peaks = find_peaks(&mag, FS, FFT, predicted * 0.55, predicted * 1.6, 0.1);
        assert!(!peaks.is_empty(), "no peak near {predicted:.0} Hz");
        nearest(&peaks, predicted).hz
    }

    fn fundamental(length: f64) -> f64 {
        fundamental_of(&[pipe(length, 0.045)], gas::T_AMB)
    }

    /// resonates at odd multiples of c/4L
    #[test]
    fn resonates_at_odd_multiples_of_c_4l() {
        for length in [0.6, 1.0] {
            let mut p = duct(&[pipe(length, 0.05)], gas::T_AMB);
            let ir = impulse_response(&mut p, FFT, 2.0);
            let mag = magnitude_spectrum(&ir, FFT);

            // Cross-check the solver's own prediction against the closed form first: the pipe plus
            // its open-end correction, 0.6133 of the mouth radius.
            let f1 = speed_of_sound_exh(gas::T_AMB) / (4.0 * (p.total_length + 0.6133 * 0.025));
            let err = (p.quarter_wave_hz() - f1).abs() / f1;
            assert!(err < 0.02, "L={length}: predicted {} vs closed form {f1}", p.quarter_wave_hz());
            let peaks = find_peaks(&mag, FS, FFT, f1 * 0.5, f1 * 4.5, 0.05);
            assert!(peaks.len() >= 2, "L={length}: {} peaks", peaks.len());

            for mode in [1.0, 3.0] {
                let want = mode * f1;
                let near = nearest(&peaks, want);
                // Looser than a delay line would need: a finite-volume scheme has numerical
                // dispersion, where a delay line has none.
                assert!(
                    (near.hz - want).abs() / want < 0.1,
                    "L={length} mode {mode}: wanted ~{want:.0} Hz, got {:.0} Hz",
                    near.hz
                );
            }
        }
    }

    /// halving the length roughly doubles the fundamental
    #[test]
    fn halving_the_length_roughly_doubles_the_fundamental() {
        let ratio = fundamental(0.5) / fundamental(1.0);
        assert!(ratio > 1.75, "ratio {ratio}");
        assert!(ratio < 2.25, "ratio {ratio}");
    }

    /// a chamber lowers the tuning, so area changes still reflect
    #[test]
    fn a_chamber_lowers_the_tuning_so_area_changes_still_reflect() {
        let straight = fundamental_of(&[pipe(1.0, 0.04)], gas::T_AMB);
        let chambered = fundamental_of(
            &[pipe(0.5, 0.04), seg(SegmentKind::Chamber, 0.3, 0.04, Some(0.14)), pipe(0.2, 0.04)],
            gas::T_AMB,
        );
        // The volume acts as a compliance, so the system tunes below a plain pipe of the
        // same overall length.
        assert!(chambered < straight * 0.95, "chambered {chambered} vs straight {straight}");
    }

    /// hot gas raises the resonance
    #[test]
    fn hot_gas_raises_the_resonance() {
        let segments = [pipe(1.0, 0.045)];
        // Sound travels faster in hot gas, so the same physical pipe tunes higher — which
        // is why an exhaust note shifts as the engine warms up.
        let hot = fundamental_of(&segments, 950.0);
        let cold = fundamental_of(&segments, gas::T_AMB);
        assert!(hot > cold * 1.2, "hot {hot} vs cold {cold}");
    }
}

mod geometry_and_robustness {
    use super::*;

    const PORT: HeadPort = HeadPort { length: 0.055, diameter: 0.034 };

    fn close_to(a: f64, b: f64, digits: i32) -> bool {
        (a - b).abs() < 10f64.powi(-digits) / 2.0
    }

    /// takes the plane-wave limit from where higher modes are launched, not always the mouth
    #[test]
    fn takes_the_plane_wave_limit_from_where_higher_modes_are_launched_not_always_the_mouth() {
        let header = pipe(0.5, 0.048);
        // Straight to the end: the mouth.
        assert!(close_to(launch_radius(&[header.clone()]), 0.024, 9));
        // A gradual megaphone: its throat, however wide the mouth.
        assert!(close_to(launch_radius(&[header.clone(), seg(SegmentKind::Cone, 1.2, 0.048, Some(0.2))]), 0.024, 9));
        // A step up to a wide tail: the tail, which the step launches modes into.
        assert!(close_to(launch_radius(&[header.clone(), pipe(0.6, 0.12)]), 0.06, 9));
        // A cone too steep to be a horn is a step.
        assert!(close_to(launch_radius(&[header.clone(), seg(SegmentKind::Cone, 0.05, 0.048, Some(0.12))]), 0.06, 9));
        // After a muffler, only the run from the tailpipe on counts.
        let r = launch_radius(&[
            header,
            seg(SegmentKind::Chamber, 0.34, 0.042, Some(0.13)),
            pipe(0.2, 0.04),
            seg(SegmentKind::Cone, 0.4, 0.04, Some(0.1)),
        ]);
        assert!(close_to(r, 0.02, 9), "{r}");
    }

    /// fixes cell length, so a short pipe costs less than a long one
    #[test]
    fn fixes_cell_length_so_a_short_pipe_costs_less_than_a_long_one() {
        let short = EulerPipe::new(&[pipe(0.4, 0.04)], FS, 900.0, &EulerPipeOptions::default());
        let long = EulerPipe::new(&[pipe(1.4, 0.04)], FS, 900.0, &EulerPipeOptions::default());
        assert!(long.n > short.n * 2, "{} vs {}", long.n, short.n);
        // Cell length stays put, which is what keeps the CFL timestep from collapsing when
        // the user shortens the pipe.
        assert!((long.dx - short.dx).abs() / short.dx < 0.25);
    }

    /// reports the port portion so the display can skip it
    #[test]
    fn reports_the_port_portion_so_the_display_can_skip_it() {
        let p = EulerPipe::new(
            &[pipe(0.5, 0.042)],
            FS,
            900.0,
            &EulerPipeOptions { port: Some(PORT), ..Default::default() },
        );
        assert!(p.port_cells > 0);
        assert!(p.port_cells < p.n);
        let bare = EulerPipe::new(&[pipe(0.5, 0.042)], FS, 900.0, &EulerPipeOptions::default());
        assert_eq!(bare.port_cells, 0);
        assert!(p.total_length > bare.total_length);
    }

    /// limits area steps without losing a chamber its volume
    #[test]
    fn limits_area_steps_without_losing_a_chamber_its_volume() {
        let mut face = vec![1.0, 1.0, 1.0, 1.0, 9.6, 9.6, 9.6, 9.6, 9.6, 1.0, 1.0, 1.0, 1.0, 0.1, 0.1, 0.1];
        let drawn = face.clone();
        limit_area_ratio(&mut face, 1.6);
        for f in 0..face.len() - 1 {
            let ratio = (face[f] / face[f + 1]).max(face[f + 1] / face[f]);
            assert!(ratio <= 1.6 + 1e-9, "face {f}: ratio {ratio}");
        }
        let vol = |a: &[f64]| {
            let mut v = 0.0;
            for f in 0..a.len() {
                v += (if f == 0 || f == a.len() - 1 { 0.5 } else { 1.0 }) * a[f];
            }
            v
        };
        assert!(close_to(vol(&face) / vol(&drawn), 1.0, 6));
        // The step straddles the drawn edge: the pipe beside it widens, the body narrows.
        assert!(face[3] > 1.0);
        assert!(face[4] < 9.6);
    }

    /// keeps a drawn muffler can close to its drawn volume and diameter
    #[test]
    fn keeps_a_drawn_muffler_can_close_to_its_drawn_volume_and_diameter() {
        let l = 0.34;
        let p = EulerPipe::new(
            &[pipe(0.5, 0.042), seg(SegmentKind::Chamber, l, 0.042, Some(0.13)), pipe(0.5, 0.042)],
            FS,
            900.0,
            &EulerPipeOptions { cell_size: Some(0.035), single_step: Some(true), ..Default::default() },
        );
        let mut vol = 0.0;
        let mut peak = 0.0f64;
        for i in 0..p.n {
            let x = (i as f64 + 0.5) * p.dx;
            if x < 0.25 || x > 0.75 + l {
                continue;
            }
            vol += (p.area_of(i) - (PI / 4.0) * 0.042f64.powi(2)) * p.dx;
            peak = peak.max(((4.0 * p.area_of(i)) / PI).sqrt());
        }
        let drawn = (PI / 4.0) * (0.13f64.powi(2) - 0.042f64.powi(2)) * l * 0.84;
        assert!(vol / drawn > 0.93, "volume ratio {}", vol / drawn);
        assert!(vol / drawn < 1.07, "volume ratio {}", vol / drawn);
        assert!(peak > 0.12, "peak diameter {peak}");
    }

    /// survives degenerate geometry and a violent valve
    #[test]
    fn survives_degenerate_geometry_and_a_violent_valve() {
        for segments in [vec![], vec![pipe(0.01, 0.008)], vec![seg(SegmentKind::Chamber, 2.0, 0.02, Some(0.4))]] {
            let mut p = EulerPipe::new(
                &segments,
                FS,
                1100.0,
                &EulerPipeOptions { port: Some(HeadPort { length: 0.05, diameter: 0.034 }), ..Default::default() },
            );
            for k in 0..4000 {
                let open = k % 200 < 60;
                let r = p.advance(
                    1.0 / FS,
                    &valve(if open { 7e-4 } else { 0.0 }, if open { 7e6 } else { gas::P_AMB }, 1800.0),
                );
                assert!(r.mouth_flow.is_finite());
                assert!(r.port_pressure.is_finite());
                assert!(r.port_pressure > 0.0);
            }
        }
    }

    /// keeps the substep count inside its budget
    #[test]
    fn keeps_the_substep_count_inside_its_budget() {
        let mut p = EulerPipe::new(
            &[pipe(1.3, 0.045)],
            FS,
            950.0,
            &EulerPipeOptions { port: Some(PORT), ..Default::default() },
        );
        let mut worst = 0;
        let mut total = 0;
        let iters = 4000;
        for k in 0..iters {
            let open = k % 300 < 90;
            let r =
                p.advance(1.0 / FS, &valve(if open { 7e-4 } else { 0.0 }, if open { 6e6 } else { gas::P_AMB }, 1600.0));
            worst = worst.max(r.substeps);
            total += r.substeps;
        }
        // Sized for roughly 2 substeps per audio sample, and even the worst sample takes no more than 8.
        assert!((total as f64 / iters as f64) < 4.0, "mean {}", total as f64 / iters as f64);
        assert!(worst <= 8, "worst {worst}");
    }

    /// A wide-open valve with little pressure across it, as through the exhaust stroke, passes a
    /// steady flow: one taken at the port pressure before it would overshoot, flowing back the next
    /// sample and out again the one after, and pump the duct.
    #[test]
    fn a_wide_open_valve_across_a_small_pressure_difference_flows_steadily() {
        for over in [200.0, 2000.0, 20000.0] {
            let mut p = EulerPipe::new(
                &[pipe(0.85, 0.044)],
                FS,
                950.0,
                &EulerPipeOptions { port: Some(PORT), ..Default::default() },
            );
            let open = valve(1.1e-3, gas::P_AMB + over, 900.0);
            let mut flows = Vec::new();
            for _ in 0..4800 {
                p.advance(1.0 / FS, &open);
                flows.push(p.valve_source_flow());
            }
            let settled = &flows[4000..];
            let mean = settled.iter().sum::<f64>() / settled.len() as f64;
            let ripple = settled.windows(2).map(|w| (w[1] - w[0]).abs()).fold(0.0, f64::max);
            assert!(settled.iter().all(|&f| f > 0.0), "{over} Pa over: the flow reverses");
            assert!(ripple < 0.01 * mean, "{over} Pa over: steps of {ripple} kg/s on a flow of {mean}");
        }
    }
}

mod wall_temperature_is_solved_not_assumed {
    use super::*;

    fn duct(over: EulerPipeOptions) -> EulerPipe {
        let opts = EulerPipeOptions { port: Some(HeadPort { length: 0.055, diameter: 0.034 }), ..Default::default() };
        EulerPipe::new(&[pipe(1.2, 0.042)], FS, 950.0, &opts.overlaid(&over))
    }

    /// Hold the valve open onto a hot, pressurised cylinder, as a running engine would.
    fn hot() -> ValveState {
        valve(4e-4, 2.2e5, 1300.0)
    }

    fn run(p: &mut EulerPipe, seconds: f64) {
        let v = hot();
        for _ in 0..upto(FS * seconds) {
            p.advance(1.0 / FS, &v);
        }
    }

    /// warms up from cold over tens of seconds
    #[test]
    fn warms_up_from_cold_over_tens_of_seconds() {
        let mut p = duct(EulerPipeOptions { initial_wall_temp: Some(gas::T_AMB), ..Default::default() });
        assert!((p.mean_wall_temp() - gas::T_AMB).abs() < 0.5);
        run(&mut p, 2.0);
        let t2 = p.mean_wall_temp();
        run(&mut p, 18.0);
        let t20 = p.mean_wall_temp();
        // Monotonic, and slow: typical 1.2 mm tubing has a time constant of tens of seconds,
        // so two seconds must not get anywhere near equilibrium.
        assert!(t2 > gas::T_AMB + 5.0, "t2 {t2}");
        assert!(t20 > t2 + 50.0, "t20 {t20} t2 {t2}");
        assert!(t2 < 500.0, "t2 {t2}");
    }

    /// runs cooler downstream, which a fixed wall temperature could not do
    #[test]
    fn runs_cooler_downstream_which_a_fixed_wall_temperature_could_not_do() {
        let mut p = duct(EulerPipeOptions::default());
        run(&mut p, 20.0);
        let mut wall = [0.0f32; 128];
        let mut gas_t = [0.0f32; 128];
        p.sample_wall_temperature(&mut wall);
        p.sample_temperature(&mut gas_t);
        // Both the wall and the gas must fall along the duct.
        assert!(wall[0] > wall[127] + 40.0, "wall {} -> {}", wall[0], wall[127]);
        assert!(gas_t[0] > gas_t[127], "gas {} -> {}", gas_t[0], gas_t[127]);
        // And the wall must sit between the gas and ambient.
        assert!(wall[64] < gas_t[64]);
        assert!(wall[64] as f64 > gas::T_AMB);
    }

    /// airflow cools it, and cooler gas lowers the tuning
    #[test]
    fn airflow_cools_it_and_cooler_gas_lowers_the_tuning() {
        let settle = |air_speed: f64| {
            let mut p = duct(EulerPipeOptions { air_speed: Some(air_speed), ..Default::default() });
            run(&mut p, 20.0);
            (p.mean_wall_temp(), p.quarter_wave_hz())
        };
        let still = settle(0.0);
        let moving = settle(30.0);
        assert!(moving.0 < still.0 - 30.0, "wall {} vs {}", moving.0, still.0);
        // Sound travels slower in cooler gas, so the same pipe tunes lower.
        assert!(moving.1 < still.1, "{} Hz vs {} Hz", moving.1, still.1);
    }

    /// a thicker wall warms more slowly, because thermal mass scales with it
    #[test]
    fn a_thicker_wall_warms_more_slowly_because_thermal_mass_scales_with_it() {
        let after30 = |wall_thickness: f64| {
            let mut p = duct(EulerPipeOptions {
                wall_thickness: Some(wall_thickness),
                initial_wall_temp: Some(gas::T_AMB),
                ..Default::default()
            });
            run(&mut p, 15.0);
            p.mean_wall_temp()
        };
        let thin = after30(0.0006);
        let thick = after30(0.003);
        assert!(thin > thick + 30.0, "thin {thin} thick {thick}");
    }

    /// a rebuild inherits the wall state instead of discarding it
    #[test]
    fn a_rebuild_inherits_the_wall_state_instead_of_discarding_it() {
        let mut p = duct(EulerPipeOptions { initial_wall_temp: Some(gas::T_AMB), ..Default::default() });
        run(&mut p, 15.0);
        let warm = p.mean_wall_temp();
        assert!(warm > gas::T_AMB + 40.0, "warm {warm}");

        // A geometry edit must not throw away a thermal state that takes half a minute to
        // rebuild, or the tuning would jump every time the user drags a handle.
        let rebuilt = EulerPipe::new(
            &[pipe(1.4, 0.042)],
            FS,
            950.0,
            &EulerPipeOptions {
                port: Some(HeadPort { length: 0.055, diameter: 0.034 }),
                inherit_wall: Some(p.export_wall()),
                ..Default::default()
            },
        );
        assert!((rebuilt.mean_wall_temp() - warm).abs() < 5.0, "rebuilt {} vs {warm}", rebuilt.mean_wall_temp());
    }

    /// a light titanium wall warms faster than a cast-iron one of the same thickness
    #[test]
    fn a_light_titanium_wall_warms_faster_than_a_cast_iron_one_of_the_same_thickness() {
        let after = |material: PipeMaterial| {
            let mut p = duct(EulerPipeOptions {
                material: Some(material.wall()),
                initial_wall_temp: Some(gas::T_AMB),
                ..Default::default()
            });
            run(&mut p, 10.0);
            p.mean_wall_temp()
        };
        let titanium = after(PipeMaterial::Titanium);
        let iron = after(PipeMaterial::CastIron);
        assert!(titanium > iron + 20.0, "titanium {titanium} cast iron {iron}");
    }
}

mod friction_acts_on_what_it_physically_should {
    use super::*;

    /// the friction factor is the Moody chart's
    #[test]
    fn the_friction_factor_is_the_moody_charts() {
        // Colebrook's values, which Haaland's form tracks to within 2%.
        for (re, rel, colebrook) in [(1e5, 0.0, 0.0180), (1e5, 1e-3, 0.0222), (1e5, 5e-3, 0.0313), (1e6, 1e-4, 0.0134)]
        {
            let f = darcy_factor(re, rel);
            assert!((f - colebrook).abs() / colebrook < 0.03, "Re {re} e/D {rel}: {f} vs {colebrook}");
        }
    }

    /// a rough bore drags more on a steady flow than a smooth one
    #[test]
    fn a_rough_bore_drags_more_on_a_steady_flow_than_a_smooth_one() {
        let steady = |material: PipeMaterial| {
            let mut p = EulerPipe::new(
                &[pipe(1.5, 0.035)],
                FS,
                900.0,
                &EulerPipeOptions { material: Some(material.wall()), ..Default::default() },
            );
            let v = valve(6e-4, 2.5e5, 1100.0);
            for _ in 0..upto(FS * 0.5) {
                p.advance(1.0 / FS, &v);
            }
            p.velocity_at(p.n / 2)
        };
        let smooth = steady(PipeMaterial::Titanium);
        let rough = steady(PipeMaterial::CastIron);
        assert!(rough < smooth * 0.98, "cast iron {rough} m/s vs titanium {smooth} m/s");
    }

    /// a steady mean flow is barely touched by the acoustic damping term
    #[test]
    fn a_steady_mean_flow_is_barely_touched_by_the_acoustic_damping_term() {
        // The linear boundary-layer term is acoustic damping and must skip the mean flow.
        // Applied to the total velocity it would brake the mean hard — at low speed it would
        // outweigh Darcy roughly forty to one.
        let steady = |linear_damping: f64| {
            let mut p = EulerPipe::new(
                &[pipe(1.0, 0.04)],
                FS,
                900.0,
                &EulerPipeOptions {
                    heat_transfer: Some(false),
                    linear_damping: Some(linear_damping),
                    darcy_friction: Some(0.03),
                    ..Default::default()
                },
            );
            let v = valve(6e-4, 2.5e5, 1100.0);
            for _ in 0..upto(FS * 0.5) {
                p.advance(1.0 / FS, &v);
            }
            p.velocity_at(p.n / 2)
        };
        let none = steady(0.0);
        let full = steady(150.0);
        assert!(none > 50.0, "undamped mean flow {none}");
        // Within 15%: the mean is exempt, so only the quadratic terms should bite.
        assert!(full > none * 0.85, "damped {full} vs undamped {none}");
    }

    /// but acoustic waves are still damped, at the calibrated rate
    #[test]
    fn but_acoustic_waves_are_still_damped_at_the_calibrated_rate() {
        let decay_per_pass = |linear_damping: f64| {
            let length = 1.0;
            let mut p = EulerPipe::new(
                &[pipe(length, 0.04)],
                FS,
                600.0,
                &EulerPipeOptions {
                    heat_transfer: Some(false),
                    radiate: Some(false),
                    linear_damping: Some(linear_damping),
                    darcy_friction: Some(0.0),
                    ..Default::default()
                },
            );
            let rho0 = gas::P_AMB / (gas::R * 600.0);
            let c = speed_of_sound_exh(600.0);
            for i in 0..p.n {
                let dp = 200.0 * math::sin((2.0 * PI * (i as f64 + 0.5) * p.dx) / 0.5);
                p.set_primitive(i, rho0 + dp / (c * c), 0.0, gas::P_AMB + dp);
            }
            let amp = |p: &EulerPipe| (0..p.n).map(|i| (p.pressure_at(i) - gas::P_AMB).abs()).fold(0.0, f64::max);
            let a0 = amp(&p);
            let seconds = 0.05;
            for _ in 0..upto(FS * seconds) {
                p.advance(1.0 / FS, &SHUT);
            }
            (20.0 * (amp(&p) / a0).log10()) / ((c * seconds) / length)
        };
        // Calibrated to about 1.5 dB per pass of a 1 m duct; numerical dissipation alone is
        // roughly 0.25, so the physical term has to dominate.
        let physical = decay_per_pass(150.0);
        let numerical_only = decay_per_pass(0.0);
        assert!(physical < -1.0, "physical {physical} dB/pass");
        assert!(physical < numerical_only * 3.0, "physical {physical} vs numerical {numerical_only}");
    }
}

mod the_open_end_reflects_less_at_high_frequency_as_a_real_one_does {
    use super::*;

    /// Energy decay time of a single closed-open mode, ms, with all wall losses disabled so
    /// only the boundary can remove energy.
    fn mode_decay_ms(order: u32, dia: f64, cfl: Option<f64>) -> f64 {
        let mut p = EulerPipe::new(
            &[pipe(1.0, dia)],
            FS,
            gas::T_AMB,
            &EulerPipeOptions {
                cell_size: Some(0.003),
                max_cells: Some(4096),
                heat_transfer: Some(false),
                linear_damping: Some(0.0),
                darcy_friction: Some(0.0),
                max_substeps: Some(64),
                cfl,
                ..Default::default()
            },
        );
        let c = speed_of_sound_exh(gas::T_AMB);
        let rho0 = gas::P_AMB / (gas::R * gas::T_AMB);
        // Closed-open mode shape: p ~ cos(kx) with k = (2m-1)pi/2L.
        let k = ((2.0 * order as f64 - 1.0) * PI) / (2.0 * p.total_length);
        for i in 0..p.n {
            let dp = 50.0 * math::cos(k * (i as f64 + 0.5) * p.dx);
            p.set_primitive(i, rho0 + dp / (c * c), 0.0, gas::P_AMB + dp);
        }
        let energy = |p: &EulerPipe| {
            let mut e = 0.0;
            for i in 0..p.n {
                let dp = p.pressure_at(i) - gas::P_AMB;
                e += dp * dp + (rho0 * c).powi(2) * p.velocity_at(i).powi(2);
            }
            e
        };
        let e0 = energy(&p);
        for s in 0..upto(FS * 0.5) {
            p.advance(1.0 / FS, &SHUT);
            if energy(&p) < e0 / std::f64::consts::E {
                return ((s + 1) as f64 / FS) * 1000.0;
            }
        }
        500.0
    }

    /// high modes die away far faster than low ones
    #[test]
    fn high_modes_die_away_far_faster_than_low_ones() {
        // The whole point: a low mode barely radiates and rings for a long time, while a high
        // mode leaves through the mouth almost immediately. Measures 188 ms at 84 Hz against
        // 8 ms at 1.9 kHz.
        let low = mode_decay_ms(1, 0.05, None);
        let mid = mode_decay_ms(5, 0.05, None);
        let high = mode_decay_ms(12, 0.05, None);
        assert!(mid < low * 0.5, "mid {mid} low {low}");
        assert!(high < mid * 0.5, "high {high} mid {mid}");
        assert!(low > 50.0, "low {low}");
    }

    /// decay matches the reflection coefficient it should have
    #[test]
    fn decay_matches_the_reflection_coefficient_it_should_have() {
        // Energy falls by |R|^2 each round trip, so the decay time pins |R| down. Compared
        // against the Levine-Schwinger result for an unflanged pipe, which the mouth's radiation
        // impedance tracks closely.
        let c = speed_of_sound_exh(gas::T_AMB);
        let a = 0.025;
        let round_trip = 2.0 / c;
        for (order, ls_ref) in [(1, 0.99), (5, 0.94), (12, 0.77)] {
            let f = ((2.0 * order as f64 - 1.0) * c) / 4.0;
            let ka = (2.0 * PI * f * a) / c;
            let tau = mode_decay_ms(order, 0.05, None) / 1000.0;
            // tau = roundTrip / (-2 ln|R|)
            let measured = (-round_trip / (2.0 * tau)).exp();
            assert!((measured - ls_ref).abs() < 0.1, "ka={ka:.2}: |R| measured {measured:.3}, expected near {ls_ref}");
        }
    }

    /// is independent of how many substeps the solver takes
    #[test]
    fn is_independent_of_how_many_substeps_the_solver_takes() {
        // The reflection filter runs once per CFL substep, so its coefficient has to come from
        // the substep duration. Deriving it from the audio sample period instead would double
        // the corner frequency and make high-frequency standing waves linger about three times
        // too long.
        let two_substeps = mode_decay_ms(10, 0.05, Some(0.85));
        let many_substeps = mode_decay_ms(10, 0.05, Some(0.2));
        assert!(many_substeps > two_substeps * 0.7, "{many_substeps} vs {two_substeps}");
        assert!(many_substeps < two_substeps * 1.4, "{many_substeps} vs {two_substeps}");
    }

    /// driven with noise, a pipe rings at its first few modes and barely above
    #[test]
    fn driven_with_noise_a_pipe_rings_at_its_first_few_modes_and_barely_above() {
        // The comb a too-perfect open end gives: every mode of the pipe standing out of the
        // noise driving it, all the way up, where a real tube's only ring at the bottom.
        const N: usize = 8192;
        let mut p = EulerPipe::new(
            &[pipe(1.0, 0.05)],
            FS,
            900.0,
            &EulerPipeOptions { heat_transfer: Some(false), single_step: Some(true), ..Default::default() },
        );
        let mut noise = Noise::new(1234.0);
        let mut drive = |p: &mut EulerPipe| {
            let v = ValveState { extra_mass_flow: 0.002 * noise.next(), ..Default::default() };
            p.advance(1.0 / FS, &v).mouth_flow as f32
        };
        for _ in 0..upto(FS * 0.1) {
            drive(&mut p);
        }
        let mut power = vec![0.0; N / 2];
        for _ in 0..24 {
            let block: Vec<f32> = (0..N).map(|_| drive(&mut p)).collect();
            for (a, m) in power.iter_mut().zip(magnitude_spectrum(&hann(&block), N)) {
                *a += m * m;
            }
        }
        let bin = FS / N as f64;
        let level = |lo: f64, hi: f64, pick: fn(f64, f64) -> f64, from: f64| {
            ((lo / bin) as usize..=(hi / bin) as usize).map(|k| 10.0 * power[k].max(1e-30).log10()).fold(from, pick)
        };
        // How far each mode stands above the dip after it, dB.
        let f1 = p.quarter_wave_hz();
        let depth = |mode: f64| {
            let f = (2.0 * mode - 1.0) * f1;
            level(f - 0.5 * f1, f + 0.5 * f1, f64::max, f64::MIN)
                - level(f + 0.5 * f1, f + 1.5 * f1, f64::min, f64::MAX)
        };
        let fundamental = depth(1.0);
        let high: Vec<f64> =
            (1..40).map(f64::from).filter(|&m| (2.0 * m - 1.0) * f1 > 1500.0).take(12).map(depth).collect();
        let high_mean = high.iter().sum::<f64>() / high.len() as f64;
        assert!(fundamental > 12.0, "fundamental stands {fundamental:.1} dB out");
        assert!(high_mean < 8.0, "modes above 1.5 kHz stand {high_mean:.1} dB out on average");
    }

    /// a wider mouth radiates high frequencies away sooner
    #[test]
    fn a_wider_mouth_radiates_high_frequencies_away_sooner() {
        // Reflection falls with ka, so a bigger radius reflects less at a given frequency.
        let wide = mode_decay_ms(8, 0.09, None);
        let narrow = mode_decay_ms(8, 0.03, None);
        assert!(wide < narrow, "wide {wide} narrow {narrow}");
    }
}

/// The solver's cell loops are vectorised two cells at a time with a scalar tail. These are the checks that stay meaningful on it: violent states that reach every branch of
/// the Riemann solver and the limiters, and cell counts of both parities so the tails are exercised.
mod vectorised_loops {
    use super::*;

    /// Deterministic PRNG, so a failure is reproducible from the seed alone.
    fn rng(seed: u32) -> impl FnMut() -> f64 {
        let mut s = seed;
        move || {
            s = s.wrapping_mul(1664525).wrapping_add(1013904223);
            s as f64 / 4294967296.0
        }
    }

    fn check_physical(p: &EulerPipe, label: &str) {
        for i in 0..p.n {
            let (rho, u, pr) = (p.density_at(i), p.velocity_at(i), p.pressure_at(i));
            assert!(rho > 0.0 && pr > 0.0 && u.is_finite(), "{label}: cell {i} has rho {rho}, u {u}, p {pr}");
        }
    }

    /// Deliberately violent states, to reach the code paths a well-behaved duct never does.
    ///
    /// The vectorised Riemann solver computes all four of the scalar `hllc`'s branches and
    /// selects, so the supersonic ones only get tested if something is actually supersonic.
    /// Mach numbers out to +/-2.5 guarantee faces where `s_l >= 0` and where `s_r <= 0`, and the
    /// pressure and density jumps drive the limiter into its zero-slope guard. Every limiter has
    /// to come through with a physical state and without needing a recovery.
    #[test]
    fn survives_violent_supersonic_states_with_every_limiter() {
        for limiter in [SlopeLimiter::Mc, SlopeLimiter::Minmod, SlopeLimiter::VanLeer] {
            let mut p = EulerPipe::new(
                &common::pipe_preset(0),
                FS,
                950.0,
                &EulerPipeOptions { limiter: Some(limiter), ..Default::default() },
            );
            let mut next = rng(0xc0ffee);
            let dt = 1.0 / FS / 4.0;
            let (mut forward, mut backward) = (false, false);
            for trial in 0..12 {
                for i in 0..p.n {
                    let rho = 0.1 + 4.0 * next();
                    let u = (next() * 2.0 - 1.0) * 900.0;
                    let pr = 2e4 + next() * 6e5;
                    let c = (GAMMA * pr / rho).sqrt();
                    forward |= u >= c;
                    backward |= u <= -c;
                    p.set_primitive(i, rho, u, pr);
                }
                p.advance(dt, &SHUT);
                check_physical(&p, &format!("{limiter:?} trial {trial}"));
            }
            // The supersonic branches are only exercised if both directions showed up.
            assert!(forward && backward, "{limiter:?}: no supersonic cells in both directions");
            assert_eq!(p.recoveries, 0, "{limiter:?} needed a recovery");
        }
    }

    /// Ducts of lengths that give odd and even cell counts, sealed, with random flow in them.
    fn sealed(length: f64) -> EulerPipe {
        EulerPipe::new(
            &[pipe(length, 0.042)],
            FS,
            950.0,
            &EulerPipeOptions {
                radiate: Some(false),
                heat_transfer: Some(false),
                linear_damping: Some(0.0),
                darcy_friction: Some(0.0),
                ..Default::default()
            },
        )
    }

    const LENGTHS: [f64; 4] = [0.31, 0.33, 0.35, 0.37];

    /// Odd cell counts, where the vector loops hand a leftover element to their scalar tails. A tail
    /// that skipped or double-counted its cell would break conservation in a sealed duct.
    #[test]
    fn conserves_mass_and_energy_for_both_parities_of_cell_count() {
        let mut parities = [false; 2];
        for length in LENGTHS {
            let mut p = sealed(length);
            parities[p.n % 2] = true;
            let mut next = rng(0xbeef + p.n as u32);
            for i in 0..p.n {
                p.set_primitive(i, 0.4 + next(), (next() * 2.0 - 1.0) * 400.0, 8e4 + next() * 3e5);
            }
            let (m0, e0) = (p.total_mass(), p.total_energy());
            for _ in 0..8 {
                p.advance(1.0 / FS / 2.0, &SHUT);
            }
            let dm = (p.total_mass() - m0).abs() / m0;
            let de = (p.total_energy() - e0).abs() / e0;
            assert!(dm < 1e-12, "n = {}: mass drift {dm}", p.n);
            assert!(de < 1e-12, "n = {}: energy drift {de}", p.n);
            check_physical(&p, &format!("n = {}", p.n));
        }
        // The tails are only actually exercised if both parities showed up.
        assert!(parities[0] && parities[1]);
    }

    /// A sealed uniform duct has no preferred direction, so mirror-image initial data must stay a
    /// mirror image. The vector loops pair cells from the inlet end, so an odd count puts the
    /// scalar tail at one end only: any disagreement between the two paths shows as asymmetry.
    #[test]
    fn keeps_a_mirror_symmetric_state_symmetric_for_both_parities_of_cell_count() {
        let mut parities = [false; 2];
        for length in LENGTHS {
            let mut p = sealed(length);
            let n = p.n;
            parities[n % 2] = true;
            let mut next = rng(0xbeef + n as u32);
            for i in 0..n.div_ceil(2) {
                let (rho, u, pr) = (0.4 + next(), (next() * 2.0 - 1.0) * 400.0, 8e4 + next() * 3e5);
                let j = n - 1 - i;
                // The middle cell of an odd duct is its own mirror image, so it cannot move.
                let u = if i == j { 0.0 } else { u };
                p.set_primitive(i, rho, u, pr);
                p.set_primitive(j, rho, -u, pr);
            }
            for _ in 0..40 {
                p.advance(1.0 / FS / 2.0, &SHUT);
            }
            for i in 0..n {
                let j = n - 1 - i;
                let dp = (p.pressure_at(i) - p.pressure_at(j)).abs() / p.pressure_at(i);
                let du = (p.velocity_at(i) + p.velocity_at(j)).abs() / 400.0;
                assert!(dp < 1e-12 && du < 1e-12, "n = {n}: cells {i} and {j} differ by {dp} in p, {du} in u");
            }
        }
        assert!(parities[0] && parities[1]);
    }
}

// --- heat released in the gas ---

/// Heat released in the leading cells raises their pressure, and all of it is in the gas's energy.
#[test]
fn heat_in_the_leading_cells_raises_their_pressure_and_is_accounted_for() {
    let mut p = tube(40, 1.0, SlopeLimiter::Mc);
    let cells = 4;
    let volume = p.leading_volume(cells);
    let (e0, p0) = (p.total_energy(), p.pressure_at(0));
    let taken = p.add_heat(1.0, cells, volume);
    assert!((taken - 1.0).abs() < 1e-12, "took {taken} J of 1 J");
    assert!(((p.total_energy() - e0) / taken - 1.0).abs() < 1e-9);
    assert!(p.pressure_at(0) > p0 && p.pressure_at(cells) == p0, "only the leading cells are heated");
}

/// A cell takes at most a quarter of its internal energy at a time, and is never heated past the
/// ceiling however much is released in it: the heat it refuses is handed back.
#[test]
fn a_cell_is_never_heated_past_the_ceiling() {
    let mut p = tube(40, 1.0, SlopeLimiter::Mc);
    let volume = p.leading_volume(2);
    let e0 = p.total_energy();
    let taken = p.add_heat(1e3, 2, volume);
    assert!(taken < 1e3, "took all {taken} J");
    assert!(((p.total_energy() - e0) / (0.25 * e0 / 20.0) - 1.0).abs() < 1e-9, "a quarter of each cell's");
    for _ in 0..40 {
        p.add_heat(1e3, 2, volume);
    }
    assert!(p.temperature_at(0) <= 2600.0 + 1e-6, "{} K", p.temperature_at(0));
    assert!(p.temperature_at(0) > 2500.0, "{} K", p.temperature_at(0));
}
