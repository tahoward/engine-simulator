//! The roar of the jet a tailpipe blows into the air.
//!
//! It must follow Lighthill's law, so a faster jet is far louder and a pipe drawing air in is silent;
//! and it must stay broadband noise round its Strouhal peak, not a hiss climbing to the top octave.

mod common;

use common::{FS, hann, magnitude_spectrum};
use engine_sim::radiation::MouthJet;

const AREA: f64 = 0.0005;
const TEMP: f64 = 800.0;

/// RMS of the jet's sound, Pa, for a steady exit speed `u`, m/s, settled.
fn rms_at(u: f64) -> f64 {
    let mut jet = MouthJet::new(99.0);
    let out: Vec<f64> = (0..48000).map(|_| jet.process(u * AREA, AREA, TEMP, 1.0, FS)).skip(4800).collect();
    (out.iter().map(|x| x * x).sum::<f64>() / out.len() as f64).sqrt()
}

/// doubling the speed makes it far louder, near the eighth power
#[test]
fn doubling_the_speed_makes_it_far_louder_near_the_eighth_power() {
    // 24 dB for the eighth power, less the hot jet's density correction, which favours the slower.
    let rise = 20.0 * (rms_at(200.0) / rms_at(100.0)).log10();
    assert!(rise > 18.0 && rise < 28.0, "doubling the speed added {rise:.1} dB");
}

/// a pipe drawing air in makes no jet
#[test]
fn a_pipe_drawing_air_in_makes_no_jet() {
    assert_eq!(rms_at(-150.0), 0.0);
}

/// its spectrum peaks near the Strouhal frequency and falls away above
#[test]
fn its_spectrum_peaks_near_the_strouhal_frequency_and_falls_away_above() {
    const N: usize = 16384;
    let u = 150.0;
    let d = (4.0 * AREA / std::f64::consts::PI).sqrt();
    let strouhal = 0.2 * u / d;
    let mut jet = MouthJet::new(7.0);
    for _ in 0..4800 {
        jet.process(u * AREA, AREA, TEMP, 1.0, FS);
    }
    let block: Vec<f32> = (0..N).map(|_| jet.process(u * AREA, AREA, TEMP, 1.0, FS) as f32).collect();
    let mag = magnitude_spectrum(&hann(&block), N);
    let bin = FS / N as f64;
    let band = |hz: f64| {
        let (lo, hi) = ((hz / 1.41 / bin) as usize, (hz * 1.41 / bin) as usize);
        mag[lo..=hi].iter().map(|m| m * m).sum::<f64>() / (hi - lo + 1) as f64
    };
    let at_peak = band(strouhal);
    // Gentler below the peak than above it, as a measured jet's spectrum is.
    assert!(band(strouhal / 8.0) < at_peak * 0.25, "three octaves below");
    assert!(band(strouhal * 8.0) < at_peak * 0.02, "three octaves above");
}
