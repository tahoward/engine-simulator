//! `pow` and `exp`: Arm's optimized-routines algorithms, as musl ships them.
//!
//! Table-driven, and about twice as fast as the fdlibm `pow` the `libm` crate has, for a worst-case
//! error of 0.54 ULP against fdlibm's 0.8 or so. The simulation takes a few hundred powers every
//! audio sample, in the cylinders' heat transfer, the orifices, the open ends and the junctions.
//!
//! Written without fused multiply-add, which musl uses where the hardware has it: the results must
//! be the same bits on every target, and the Wasm build has no FMA. Expression order follows the C,
//! which evaluates left to right.
//!
//! Copyright (c) 2018, Arm Limited. SPDX-License-Identifier: MIT.

use crate::math_tables::{
    EXP_POLY, EXP_TAB, INV_LN2_N, LN2HI, LN2LO, NEG_LN2_HI_N, NEG_LN2_LO_N, POW_POLY, POW_TAB, SHIFT,
};

const POW_LOG_TABLE_BITS: u32 = 7;
const EXP_TABLE_BITS: u32 = 7;
const N_EXP: u64 = 1 << EXP_TABLE_BITS;
const OFF: u64 = 0x3fe6955500000000;
const SIGN_BIAS: u64 = 0x800 << EXP_TABLE_BITS;

#[inline(always)]
fn top12(x: f64) -> u32 {
    (x.to_bits() >> 52) as u32
}

/// `log(x)` as `y + tail`, where `ix` is the bits of a positive normal `x`.
#[inline(always)]
fn log_inline(ix: u64) -> (f64, f64) {
    let tmp = ix.wrapping_sub(OFF);
    let i = ((tmp >> (52 - POW_LOG_TABLE_BITS)) % (1 << POW_LOG_TABLE_BITS)) as usize;
    let k = (tmp as i64) >> 52;
    let iz = ix.wrapping_sub(tmp & (0xfffu64 << 52));
    let z = f64::from_bits(iz);
    let kd = k as f64;

    let (invc, logc, logctail) = POW_TAB[i];

    // Split z such that rhi, rlo and rhi*rhi are exact and |rlo| <= |r|.
    let zhi = f64::from_bits(iz.wrapping_add(1u64 << 31) & (u64::MAX << 32));
    let zlo = z - zhi;
    let rhi = zhi * invc - 1.0;
    let rlo = zlo * invc;
    let r = rhi + rlo;

    let t1 = kd * LN2HI + logc;
    let t2 = t1 + r;
    let lo1 = kd * LN2LO + logctail;
    let lo2 = t1 - t2 + r;

    let a = &POW_POLY;
    let ar = a[0] * r;
    let ar2 = r * ar;
    let ar3 = r * ar2;
    let arhi = a[0] * rhi;
    let arhi2 = rhi * arhi;
    let hi = t2 + arhi2;
    let lo3 = rlo * (ar + arhi);
    let lo4 = t2 - hi + arhi2;
    let p = ar3 * (a[1] + r * a[2] + ar2 * (a[3] + r * a[4] + ar2 * (a[5] + r * a[6])));
    let lo = lo1 + lo2 + lo3 + lo4 + p;
    let y = hi + lo;
    let tail = hi - y + lo;
    (y, tail)
}

#[inline(never)]
fn oflow(sign: u64) -> f64 {
    let x = if sign != 0 { -f64::from_bits(0x7000000000000000) } else { f64::from_bits(0x7000000000000000) };
    x * f64::from_bits(0x7000000000000000)
}

#[inline(never)]
fn uflow(sign: u64) -> f64 {
    let x = if sign != 0 { -f64::from_bits(0x1000000000000000) } else { f64::from_bits(0x1000000000000000) };
    x * f64::from_bits(0x1000000000000000)
}

/// `scale * (1 + tmp)` without intermediate rounding, where it may overflow or underflow.
#[inline(never)]
fn specialcase(tmp: f64, sbits: u64, ki: u64) -> f64 {
    if ki & 0x80000000 == 0 {
        // k > 0: the exponent of scale might have overflowed by <= 460.
        let scale = f64::from_bits(sbits.wrapping_sub(1009u64 << 52));
        let y = f64::from_bits(0x7f00000000000000) * (scale + scale * tmp);
        return y;
    }
    // k < 0: care in the subnormal range.
    let sbits = sbits.wrapping_add(1022u64 << 52);
    let scale = f64::from_bits(sbits);
    let mut y = scale + scale * tmp;
    if y.abs() < 1.0 {
        let one = if y < 0.0 { -1.0 } else { 1.0 };
        let lo = scale - y + scale * tmp;
        let hi = one + y;
        let lo = one - hi + y + lo;
        y = (hi + lo) - one;
        if y == 0.0 {
            y = f64::from_bits(sbits & 0x8000000000000000);
        }
    }
    f64::from_bits(0x0010000000000000) * y
}

/// `sign * exp(x + xtail)`, where `|xtail| < 2^-8/N` and `|xtail| <= |x|`.
#[inline(always)]
fn exp_inline(x: f64, xtail: f64, sign_bias: u64) -> f64 {
    let mut abstop = top12(x) & 0x7ff;
    let tiny = 0x3c9u32; // top12(0x1p-54)
    if abstop.wrapping_sub(tiny) >= 0x408 - tiny {
        if abstop.wrapping_sub(tiny) >= 0x80000000 {
            let one = 1.0 + x;
            return if sign_bias != 0 { -one } else { one };
        }
        if abstop >= 0x409 {
            return if x.to_bits() >> 63 != 0 { uflow(sign_bias) } else { oflow(sign_bias) };
        }
        abstop = 0;
    }
    let z = INV_LN2_N * x;
    let mut kd = z + SHIFT;
    let ki = kd.to_bits();
    kd -= SHIFT;
    let mut r = x + kd * NEG_LN2_HI_N + kd * NEG_LN2_LO_N;
    r += xtail;
    let idx = (2 * (ki % N_EXP)) as usize;
    let top = ki.wrapping_add(sign_bias) << (52 - EXP_TABLE_BITS);
    let tail = f64::from_bits(EXP_TAB[idx]);
    let sbits = EXP_TAB[idx + 1].wrapping_add(top);
    let r2 = r * r;
    let c = &EXP_POLY;
    let tmp = tail + r + r2 * (c[0] + r * c[1]) + r2 * r2 * (c[2] + r * c[3]);
    if abstop == 0 {
        return specialcase(tmp, sbits, ki);
    }
    let scale = f64::from_bits(sbits);
    scale + scale * tmp
}

/// 0 if not an integer, 1 if odd, 2 if even, for the bits of a non-zero finite value.
#[inline]
fn checkint(iy: u64) -> u32 {
    let e = (iy >> 52 & 0x7ff) as i32;
    if e < 0x3ff {
        return 0;
    }
    if e > 0x3ff + 52 {
        return 2;
    }
    if iy & ((1u64 << (0x3ff + 52 - e)) - 1) != 0 {
        return 0;
    }
    if iy & (1u64 << (0x3ff + 52 - e)) != 0 {
        return 1;
    }
    2
}

/// Whether `i` is the bits of 0, infinity or NaN.
#[inline(always)]
fn zeroinfnan(i: u64) -> bool {
    i.wrapping_mul(2).wrapping_sub(1) >= 2 * f64::INFINITY.to_bits() - 1
}

pub fn pow(x: f64, y: f64) -> f64 {
    let mut sign_bias = 0u64;
    let mut ix = x.to_bits();
    let iy = y.to_bits();
    let mut topx = top12(x);
    let topy = top12(y);
    if topx.wrapping_sub(0x001) >= 0x7ff - 0x001 || (topy & 0x7ff).wrapping_sub(0x3be) >= 0x43e - 0x3be {
        // Special cases: x below 2^-1022, infinite or NaN, or |y| below 2^-65, above 2^63, or NaN.
        if zeroinfnan(iy) {
            if iy.wrapping_mul(2) == 0 {
                return 1.0;
            }
            if ix == 1.0f64.to_bits() {
                return 1.0;
            }
            if ix.wrapping_mul(2) > 2 * f64::INFINITY.to_bits() || iy.wrapping_mul(2) > 2 * f64::INFINITY.to_bits() {
                return x + y;
            }
            if ix.wrapping_mul(2) == 2 * 1.0f64.to_bits() {
                return 1.0;
            }
            if (ix.wrapping_mul(2) < 2 * 1.0f64.to_bits()) == (iy >> 63 == 0) {
                return 0.0;
            }
            return y * y;
        }
        if zeroinfnan(ix) {
            let mut x2 = x * x;
            if ix >> 63 != 0 && checkint(iy) == 1 {
                x2 = -x2;
            }
            return if iy >> 63 != 0 { 1.0 / x2 } else { x2 };
        }
        // Here x and y are non-zero finite.
        if ix >> 63 != 0 {
            let yint = checkint(iy);
            if yint == 0 {
                return f64::NAN;
            }
            if yint == 1 {
                sign_bias = SIGN_BIAS;
            }
            ix &= 0x7fffffffffffffff;
            topx &= 0x7ff;
        }
        if (topy & 0x7ff).wrapping_sub(0x3be) >= 0x43e - 0x3be {
            if ix == 1.0f64.to_bits() {
                return 1.0;
            }
            if (topy & 0x7ff) < 0x3be {
                // |y| < 2^-65: x^y ~= 1 + y log(x).
                return if ix > 1.0f64.to_bits() { 1.0 + y } else { 1.0 - y };
            }
            return if (ix > 1.0f64.to_bits()) == (topy < 0x800) { oflow(0) } else { uflow(0) };
        }
        if topx == 0 {
            // Normalize subnormal x so exponent becomes negative.
            ix = (f64::from_bits(ix) * f64::from_bits(0x4330000000000000)).to_bits();
            ix &= 0x7fffffffffffffff;
            ix = ix.wrapping_sub(52u64 << 52);
        }
    }

    let (hi, lo) = log_inline(ix);
    let yhi = f64::from_bits(iy & (u64::MAX << 27));
    let ylo = y - yhi;
    let lhi = f64::from_bits(hi.to_bits() & (u64::MAX << 27));
    let llo = hi - lhi + lo;
    let ehi = yhi * lhi;
    let elo = ylo * lhi + y * llo;
    exp_inline(ehi, elo, sign_bias)
}

/// `x`'s logarithm, as `pow` takes it, for raising `x` to several powers with one logarithm: `powf` of
/// it gives the bits `pow` does.
#[derive(Clone, Copy, Debug)]
pub struct PowBase {
    x: f64,
    /// The logarithm split for the product with the power, or `normal` false where `x` is zero,
    /// negative, subnormal, infinite or NaN, which `pow` handles case by case.
    lhi: f64,
    llo: f64,
    normal: bool,
}

#[inline(always)]
pub fn pow_base(x: f64) -> PowBase {
    if top12(x).wrapping_sub(0x001) >= 0x7ff - 0x001 {
        return PowBase { x, lhi: 0.0, llo: 0.0, normal: false };
    }
    let (hi, lo) = log_inline(x.to_bits());
    let lhi = f64::from_bits(hi.to_bits() & (u64::MAX << 27));
    PowBase { x, lhi, llo: hi - lhi + lo, normal: true }
}

impl PowBase {
    /// `pow(x, y)`, to the bit.
    #[inline(always)]
    pub fn powf(&self, y: f64) -> f64 {
        if !self.normal || (top12(y) & 0x7ff).wrapping_sub(0x3be) >= 0x43e - 0x3be {
            return pow(self.x, y);
        }
        let yhi = f64::from_bits(y.to_bits() & (u64::MAX << 27));
        let ylo = y - yhi;
        let ehi = yhi * self.lhi;
        let elo = ylo * self.lhi + y * self.llo;
        exp_inline(ehi, elo, 0)
    }
}

pub fn exp(x: f64) -> f64 {
    let mut abstop = top12(x) & 0x7ff;
    let tiny = 0x3c9u32; // top12(0x1p-54)
    if abstop.wrapping_sub(tiny) >= 0x408 - tiny {
        if abstop.wrapping_sub(tiny) >= 0x80000000 {
            // Avoid spurious underflow for tiny x. 0 is a common input.
            return 1.0 + x;
        }
        if abstop >= 0x409 {
            if x.to_bits() == f64::NEG_INFINITY.to_bits() {
                return 0.0;
            }
            if abstop >= 0x7ff {
                return 1.0 + x;
            }
            return if x.to_bits() >> 63 != 0 { uflow(0) } else { oflow(0) };
        }
        abstop = 0;
    }
    let z = INV_LN2_N * x;
    let mut kd = z + SHIFT;
    let ki = kd.to_bits();
    kd -= SHIFT;
    let r = x + kd * NEG_LN2_HI_N + kd * NEG_LN2_LO_N;
    let idx = (2 * (ki % N_EXP)) as usize;
    let top = ki << (52 - EXP_TABLE_BITS);
    let tail = f64::from_bits(EXP_TAB[idx]);
    let sbits = EXP_TAB[idx + 1].wrapping_add(top);
    let r2 = r * r;
    let c = &EXP_POLY;
    let tmp = tail + r + r2 * (c[0] + r * c[1]) + r2 * r2 * (c[2] + r * c[3]);
    if abstop == 0 {
        return specialcase(tmp, sbits, ki);
    }
    let scale = f64::from_bits(sbits);
    scale + scale * tmp
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ulps(a: f64, b: f64) -> u64 {
        if a == b {
            return 0;
        }
        (a.to_bits() as i64 - b.to_bits() as i64).unsigned_abs()
    }

    /// Within an ULP of the fdlibm implementation everywhere the simulation takes a power.
    #[test]
    fn pow_agrees_with_fdlibm() {
        let mut s: u64 = 0x9e3779b97f4a7c15;
        let mut next = || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 11) as f64 / (1u64 << 53) as f64
        };
        let mut worst = 0;
        for _ in 0..2_000_000 {
            let x = 10f64.powf(next() * 16.0 - 8.0);
            let y = next() * 8.0 - 4.0;
            worst = worst.max(ulps(pow(x, y), libm::pow(x, y)));
        }
        assert!(worst <= 1, "worst {worst} ulp");
    }

    #[test]
    fn exp_agrees_with_fdlibm() {
        let mut worst = 0;
        for k in 0..2_000_000 {
            let x = -700.0 + 1400.0 * (k as f64 / 2_000_000.0);
            worst = worst.max(ulps(exp(x), libm::exp(x)));
        }
        assert!(worst <= 1, "worst {worst} ulp");
    }

    /// A shared logarithm raises to the bits `pow` gives, special cases included.
    #[test]
    fn a_shared_logarithm_gives_pow_to_the_bit() {
        let mut s: u64 = 0x2545f4914f6cdd1d;
        let mut next = || {
            s ^= s << 13;
            s ^= s >> 7;
            s ^= s << 17;
            (s >> 11) as f64 / (1u64 << 53) as f64
        };
        for _ in 0..1_000_000 {
            let x = 10f64.powf(next() * 16.0 - 8.0);
            let y = next() * 8.0 - 4.0;
            assert_eq!(pow_base(x).powf(y).to_bits(), pow(x, y).to_bits(), "{x}^{y}");
        }
        for (x, y) in [
            (0.0, 2.0),
            (-2.0, 3.0),
            (-2.0, 0.5),
            (f64::INFINITY, -1.0),
            (f64::NAN, 2.0),
            (2.0, 0.0),
            (1e-310, 0.5),
            (3.0, 1e-30),
            (3.0, 1e30),
        ] {
            let (a, b) = (pow_base(x).powf(y), pow(x, y));
            assert!(a.to_bits() == b.to_bits() || (a.is_nan() && b.is_nan()), "{x}^{y}: {a} against {b}");
        }
    }

    #[test]
    fn special_cases() {
        assert_eq!(pow(2.0, 0.0), 1.0);
        assert_eq!(pow(0.0, 2.0), 0.0);
        assert!(pow(-2.0, 0.5).is_nan());
        assert_eq!(pow(-2.0, 3.0), -8.0);
        assert_eq!(pow(f64::INFINITY, -1.0), 0.0);
        assert!(pow(f64::NAN, 2.0).is_nan());
        assert_eq!(exp(0.0), 1.0);
        assert_eq!(exp(f64::NEG_INFINITY), 0.0);
        assert_eq!(exp(1000.0), f64::INFINITY);
        assert_eq!(exp(-1000.0), 0.0);
    }
}
