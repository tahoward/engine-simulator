//! Floating-point helpers with JavaScript's semantics, and the transcendental functions the
//! simulation uses.
//!
//! Every transcendental is a pure software implementation, so a native build and the Wasm build
//! compute the same bits and the web app and the desktop app sound the same: `pow` and `exp` from
//! `crate::pow`, the rest from the `libm` crate. `f64`'s own `sin`, `exp` and so on call the
//! platform's C library, which differs from one system to the next.
//!
//! `max`, `min` and `round` follow `Math.max`, `Math.min` and `Math.round` exactly, NaN and signed
//! zero included, because the solver's recovery path depends on a NaN propagating rather than being
//! swallowed by a comparison.

pub use core::f64::consts::{PI, SQRT_2};

#[inline(always)]
pub fn sin(x: f64) -> f64 {
    libm::sin(x)
}

#[inline(always)]
pub fn cos(x: f64) -> f64 {
    libm::cos(x)
}

/// `sin(x)` and `cos(x)` together, sharing the argument reduction. Bit for bit the same as the two
/// separately.
#[inline(always)]
pub fn sincos(x: f64) -> (f64, f64) {
    libm::sincos(x)
}

#[inline(always)]
pub fn exp(x: f64) -> f64 {
    crate::pow::exp(x)
}

#[inline(always)]
pub fn log(x: f64) -> f64 {
    libm::log(x)
}

#[inline(always)]
pub fn pow(x: f64, y: f64) -> f64 {
    crate::pow::pow(x, y)
}

#[inline(always)]
pub fn tanh(x: f64) -> f64 {
    libm::tanh(x)
}

#[inline(always)]
pub fn atan(x: f64) -> f64 {
    libm::atan(x)
}

#[inline(always)]
pub fn atan2(y: f64, x: f64) -> f64 {
    libm::atan2(y, x)
}

#[inline(always)]
pub fn asin(x: f64) -> f64 {
    libm::asin(x)
}

#[inline(always)]
pub fn acos(x: f64) -> f64 {
    libm::acos(x)
}

#[inline(always)]
pub fn cbrt(x: f64) -> f64 {
    libm::cbrt(x)
}

/// Correctly rounded on every target, so no library is needed.
#[inline(always)]
pub fn sqrt(x: f64) -> f64 {
    x.sqrt()
}

/// `Math.max(a, b)`: NaN if either is, and `+0` over `-0`.
#[inline(always)]
pub fn max(a: f64, b: f64) -> f64 {
    if a > b {
        a
    } else if b > a {
        b
    } else if a == b {
        if a == 0.0 && a.is_sign_negative() { b } else { a }
    } else {
        f64::NAN
    }
}

/// `Math.min(a, b)`: NaN if either is, and `-0` under `+0`.
#[inline(always)]
pub fn min(a: f64, b: f64) -> f64 {
    if a < b {
        a
    } else if b < a {
        b
    } else if a == b {
        if a == 0.0 && a.is_sign_positive() { b } else { a }
    } else {
        f64::NAN
    }
}

/// `x < lo ? lo : x > hi ? hi : x`, which passes NaN through.
#[inline(always)]
pub fn clamp(x: f64, lo: f64, hi: f64) -> f64 {
    if x < lo {
        lo
    } else if x > hi {
        hi
    } else {
        x
    }
}

/// `Math.round`: halves round toward +Infinity, as V8 does it.
#[inline]
pub fn round(x: f64) -> f64 {
    if !x.is_finite() {
        return x;
    }
    let r = x.ceil();
    if r - 0.5 > x { r - 1.0 } else { r }
}

/// `Math.sign`.
#[inline]
pub fn sign(x: f64) -> f64 {
    if x > 0.0 {
        1.0
    } else if x < 0.0 {
        -1.0
    } else {
        x
    }
}

/// ToUint32, as `x >>> 0` applies it to a number: truncated, then taken modulo 2^32.
pub fn to_uint32(x: f64) -> u32 {
    if !x.is_finite() {
        return 0;
    }
    let t = x.trunc();
    t.rem_euclid(4_294_967_296.0) as u32
}

/// `Math.hypot`, bit for bit as V8 computes it: scaled by the largest magnitude, with a compensated
/// sum of the squares.
pub fn hypot(values: &[f64]) -> f64 {
    let mut max_abs = 0.0f64;
    let mut one_nan = false;
    for &v in values {
        let a = v.abs();
        if a == f64::INFINITY {
            return f64::INFINITY;
        }
        if a.is_nan() {
            one_nan = true;
        } else if a > max_abs {
            max_abs = a;
        }
    }
    if one_nan {
        return f64::NAN;
    }
    if max_abs == 0.0 {
        return 0.0;
    }
    let mut sum = 0.0f64;
    let mut compensation = 0.0f64;
    for &v in values {
        let n = v.abs() / max_abs;
        let summand = n * n - compensation;
        let preliminary = sum + summand;
        compensation = (preliminary - sum) - summand;
        sum = preliminary;
    }
    sum.sqrt() * max_abs
}

/// `Math.hypot(a, b)`, written out for the per-sample path. The same arithmetic as `hypot`.
#[inline]
pub fn hypot2(a: f64, b: f64) -> f64 {
    let x = a.abs();
    let y = b.abs();
    if x == f64::INFINITY || y == f64::INFINITY {
        return f64::INFINITY;
    }
    let m = if x > y { x } else { y };
    if m == 0.0 {
        return 0.0;
    }
    if m.is_nan() {
        return f64::NAN;
    }
    let nx = x / m;
    let ny = y / m;
    let mut sum = 0.0;
    let mut comp = 0.0;
    let mut summand = nx * nx - comp;
    let mut prelim = sum + summand;
    comp = prelim - sum - summand;
    sum = prelim;
    summand = ny * ny - comp;
    prelim = sum + summand;
    sum = prelim;
    sum.sqrt() * m
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_matches_javascript() {
        assert_eq!(round(0.5), 1.0);
        assert_eq!(round(-0.5), -0.0);
        assert_eq!(round(2.5), 3.0);
        assert_eq!(round(-2.5), -2.0);
        assert_eq!(round(0.49999999999999994), 0.0);
        assert_eq!(round(1.4), 1.0);
    }

    #[test]
    fn max_and_min_propagate_nan() {
        assert!(max(f64::NAN, 1.0).is_nan());
        assert!(max(1.0, f64::NAN).is_nan());
        assert!(min(f64::NAN, 1.0).is_nan());
        assert!(max(-0.0, 0.0).is_sign_positive());
        assert!(min(0.0, -0.0).is_sign_negative());
    }

    #[test]
    fn to_uint32_wraps() {
        assert_eq!(to_uint32(4_294_967_296.0 + 5.0), 5);
        assert_eq!(to_uint32(-1.0), 4_294_967_295);
        assert_eq!(to_uint32(3.9), 3);
    }

    #[test]
    fn hypot_forms_agree() {
        for &(a, b) in &[(3.0, 4.0), (1e-300, 1e-300), (0.2, -7.5), (0.0, 0.0)] {
            assert_eq!(hypot(&[a, b]).to_bits(), hypot2(a, b).to_bits());
        }
    }
}
