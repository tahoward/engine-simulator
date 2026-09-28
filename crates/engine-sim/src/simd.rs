//! Two `f64` lanes, for the gas solver's cell loops.
//!
//! Each lane computes exactly what the scalar code computes for one cell: IEEE addition,
//! subtraction, multiplication, division and square root are correctly rounded element by element,
//! so vectorising across cells changes no bits. Branches become selects, with both sides computed
//! and the one the scalar code would take kept; a discarded lane may hold Inf or NaN, which is
//! harmless because nothing traps and a select copies bits.
//!
//! Two kinds of clamp appear in the solver and they differ on NaN, so both are provided:
//! `max_js` is `Math.max`, which propagates NaN, and `select(x.gt(k), x, k)` is `x > k ? x : k`,
//! which discards it. The recovery path depends on a broken cell staying NaN.
//!
//! `load` and `store` take the two elements from `i` without a bounds check, so they are `unsafe`: the
//! loops that use them are bounded by `i + 1 < n` on slices at least `n` long.
//!
//! NEON on aarch64 and SIMD128 on wasm32 have `Math.max` semantics natively (NaN in, NaN out; +0
//! above -0). SSE2 does not, and gets it from compares and masks.

#[cfg(target_arch = "aarch64")]
mod imp {
    use core::arch::aarch64::*;

    #[derive(Clone, Copy)]
    pub struct F2(pub float64x2_t);
    #[derive(Clone, Copy)]
    pub struct M2(pub uint64x2_t);

    #[allow(clippy::should_implement_trait)]
    impl F2 {
        #[inline(always)]
        pub fn splat(x: f64) -> F2 {
            unsafe { F2(vdupq_n_f64(x)) }
        }
        /// # Safety
        /// `i + 2 <= s.len()`.
        #[inline(always)]
        pub unsafe fn load(s: &[f64], i: usize) -> F2 {
            debug_assert!(i + 2 <= s.len());
            unsafe { F2(vld1q_f64(s.as_ptr().add(i))) }
        }
        /// # Safety
        /// `i + 2 <= s.len()`.
        #[inline(always)]
        pub unsafe fn store(self, s: &mut [f64], i: usize) {
            debug_assert!(i + 2 <= s.len());
            unsafe { vst1q_f64(s.as_mut_ptr().add(i), self.0) }
        }
        #[inline(always)]
        pub fn add(self, o: F2) -> F2 {
            unsafe { F2(vaddq_f64(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn sub(self, o: F2) -> F2 {
            unsafe { F2(vsubq_f64(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn mul(self, o: F2) -> F2 {
            unsafe { F2(vmulq_f64(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn div(self, o: F2) -> F2 {
            unsafe { F2(vdivq_f64(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn sqrt(self) -> F2 {
            unsafe { F2(vsqrtq_f64(self.0)) }
        }
        #[inline(always)]
        pub fn abs(self) -> F2 {
            unsafe { F2(vabsq_f64(self.0)) }
        }
        #[inline(always)]
        pub fn neg(self) -> F2 {
            unsafe { F2(vnegq_f64(self.0)) }
        }
        #[inline(always)]
        pub fn max_js(self, o: F2) -> F2 {
            unsafe { F2(vmaxq_f64(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn min_js(self, o: F2) -> F2 {
            unsafe { F2(vminq_f64(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn lt(self, o: F2) -> M2 {
            unsafe { M2(vcltq_f64(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn le(self, o: F2) -> M2 {
            unsafe { M2(vcleq_f64(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn gt(self, o: F2) -> M2 {
            unsafe { M2(vcgtq_f64(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn ge(self, o: F2) -> M2 {
            unsafe { M2(vcgeq_f64(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn lane0(self) -> f64 {
            unsafe { vgetq_lane_f64(self.0, 0) }
        }
        #[inline(always)]
        pub fn lane1(self) -> f64 {
            unsafe { vgetq_lane_f64(self.0, 1) }
        }
    }

    /// `m ? a : b`, lane by lane.
    #[inline(always)]
    pub fn select(m: M2, a: F2, b: F2) -> F2 {
        unsafe { F2(vbslq_f64(m.0, a.0, b.0)) }
    }
}

#[cfg(all(target_arch = "wasm32", target_feature = "simd128"))]
mod imp {
    use core::arch::wasm32::*;

    #[derive(Clone, Copy)]
    pub struct F2(pub v128);
    #[derive(Clone, Copy)]
    pub struct M2(pub v128);

    #[allow(clippy::should_implement_trait)]
    impl F2 {
        #[inline(always)]
        pub fn splat(x: f64) -> F2 {
            F2(f64x2_splat(x))
        }
        /// # Safety
        /// `i + 2 <= s.len()`.
        #[inline(always)]
        pub unsafe fn load(s: &[f64], i: usize) -> F2 {
            debug_assert!(i + 2 <= s.len());
            unsafe { F2(v128_load(s.as_ptr().add(i) as *const v128)) }
        }
        /// # Safety
        /// `i + 2 <= s.len()`.
        #[inline(always)]
        pub unsafe fn store(self, s: &mut [f64], i: usize) {
            debug_assert!(i + 2 <= s.len());
            unsafe { v128_store(s.as_mut_ptr().add(i) as *mut v128, self.0) }
        }
        #[inline(always)]
        pub fn add(self, o: F2) -> F2 {
            F2(f64x2_add(self.0, o.0))
        }
        #[inline(always)]
        pub fn sub(self, o: F2) -> F2 {
            F2(f64x2_sub(self.0, o.0))
        }
        #[inline(always)]
        pub fn mul(self, o: F2) -> F2 {
            F2(f64x2_mul(self.0, o.0))
        }
        #[inline(always)]
        pub fn div(self, o: F2) -> F2 {
            F2(f64x2_div(self.0, o.0))
        }
        #[inline(always)]
        pub fn sqrt(self) -> F2 {
            F2(f64x2_sqrt(self.0))
        }
        #[inline(always)]
        pub fn abs(self) -> F2 {
            F2(f64x2_abs(self.0))
        }
        #[inline(always)]
        pub fn neg(self) -> F2 {
            F2(f64x2_neg(self.0))
        }
        #[inline(always)]
        pub fn max_js(self, o: F2) -> F2 {
            F2(f64x2_max(self.0, o.0))
        }
        #[inline(always)]
        pub fn min_js(self, o: F2) -> F2 {
            F2(f64x2_min(self.0, o.0))
        }
        #[inline(always)]
        pub fn lt(self, o: F2) -> M2 {
            M2(f64x2_lt(self.0, o.0))
        }
        #[inline(always)]
        pub fn le(self, o: F2) -> M2 {
            M2(f64x2_le(self.0, o.0))
        }
        #[inline(always)]
        pub fn gt(self, o: F2) -> M2 {
            M2(f64x2_gt(self.0, o.0))
        }
        #[inline(always)]
        pub fn ge(self, o: F2) -> M2 {
            M2(f64x2_ge(self.0, o.0))
        }
        #[inline(always)]
        pub fn lane0(self) -> f64 {
            f64x2_extract_lane::<0>(self.0)
        }
        #[inline(always)]
        pub fn lane1(self) -> f64 {
            f64x2_extract_lane::<1>(self.0)
        }
    }

    /// `m ? a : b`, lane by lane.
    #[inline(always)]
    pub fn select(m: M2, a: F2, b: F2) -> F2 {
        F2(v128_bitselect(a.0, b.0, m.0))
    }
}

#[cfg(target_arch = "x86_64")]
mod imp {
    use core::arch::x86_64::*;

    #[derive(Clone, Copy)]
    pub struct F2(pub __m128d);
    #[derive(Clone, Copy)]
    pub struct M2(pub __m128d);

    #[allow(clippy::should_implement_trait)]
    impl F2 {
        #[inline(always)]
        pub fn splat(x: f64) -> F2 {
            unsafe { F2(_mm_set1_pd(x)) }
        }
        /// # Safety
        /// `i + 2 <= s.len()`.
        #[inline(always)]
        pub unsafe fn load(s: &[f64], i: usize) -> F2 {
            debug_assert!(i + 2 <= s.len());
            unsafe { F2(_mm_loadu_pd(s.as_ptr().add(i))) }
        }
        /// # Safety
        /// `i + 2 <= s.len()`.
        #[inline(always)]
        pub unsafe fn store(self, s: &mut [f64], i: usize) {
            debug_assert!(i + 2 <= s.len());
            unsafe { _mm_storeu_pd(s.as_mut_ptr().add(i), self.0) }
        }
        #[inline(always)]
        pub fn add(self, o: F2) -> F2 {
            unsafe { F2(_mm_add_pd(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn sub(self, o: F2) -> F2 {
            unsafe { F2(_mm_sub_pd(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn mul(self, o: F2) -> F2 {
            unsafe { F2(_mm_mul_pd(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn div(self, o: F2) -> F2 {
            unsafe { F2(_mm_div_pd(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn sqrt(self) -> F2 {
            unsafe { F2(_mm_sqrt_pd(self.0)) }
        }
        #[inline(always)]
        pub fn abs(self) -> F2 {
            unsafe { F2(_mm_andnot_pd(_mm_set1_pd(-0.0), self.0)) }
        }
        #[inline(always)]
        pub fn neg(self) -> F2 {
            unsafe { F2(_mm_xor_pd(_mm_set1_pd(-0.0), self.0)) }
        }
        /// `Math.max`: `maxpd` returns its second operand on NaN or on equal values, so NaN is
        /// forced where either is one and equal values are ANDed, which puts +0 above -0.
        #[inline(always)]
        pub fn max_js(self, o: F2) -> F2 {
            unsafe {
                let m = _mm_max_pd(self.0, o.0);
                let eq = _mm_cmpeq_pd(self.0, o.0);
                let m = _mm_or_pd(_mm_and_pd(eq, _mm_and_pd(self.0, o.0)), _mm_andnot_pd(eq, m));
                let nan = _mm_cmpunord_pd(self.0, o.0);
                F2(_mm_or_pd(m, nan))
            }
        }
        /// `Math.min`, as `max_js` with equal values ORed, which puts -0 below +0.
        #[inline(always)]
        pub fn min_js(self, o: F2) -> F2 {
            unsafe {
                let m = _mm_min_pd(self.0, o.0);
                let eq = _mm_cmpeq_pd(self.0, o.0);
                let m = _mm_or_pd(_mm_and_pd(eq, _mm_or_pd(self.0, o.0)), _mm_andnot_pd(eq, m));
                let nan = _mm_cmpunord_pd(self.0, o.0);
                F2(_mm_or_pd(m, nan))
            }
        }
        #[inline(always)]
        pub fn lt(self, o: F2) -> M2 {
            unsafe { M2(_mm_cmplt_pd(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn le(self, o: F2) -> M2 {
            unsafe { M2(_mm_cmple_pd(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn gt(self, o: F2) -> M2 {
            unsafe { M2(_mm_cmpgt_pd(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn ge(self, o: F2) -> M2 {
            unsafe { M2(_mm_cmpge_pd(self.0, o.0)) }
        }
        #[inline(always)]
        pub fn lane0(self) -> f64 {
            unsafe { _mm_cvtsd_f64(self.0) }
        }
        #[inline(always)]
        pub fn lane1(self) -> f64 {
            unsafe { _mm_cvtsd_f64(_mm_unpackhi_pd(self.0, self.0)) }
        }
    }

    /// `m ? a : b`, lane by lane.
    #[inline(always)]
    pub fn select(m: M2, a: F2, b: F2) -> F2 {
        unsafe { F2(_mm_or_pd(_mm_and_pd(m.0, a.0), _mm_andnot_pd(m.0, b.0))) }
    }
}

#[cfg(not(any(
    target_arch = "aarch64",
    target_arch = "x86_64",
    all(target_arch = "wasm32", target_feature = "simd128")
)))]
mod imp {
    use crate::math;

    #[derive(Clone, Copy)]
    pub struct F2(pub [f64; 2]);
    #[derive(Clone, Copy)]
    pub struct M2(pub [bool; 2]);

    macro_rules! lanes {
        ($a:expr, $b:expr, $f:expr) => {
            F2([$f($a.0[0], $b.0[0]), $f($a.0[1], $b.0[1])])
        };
    }

    #[allow(clippy::should_implement_trait)]
    impl F2 {
        #[inline(always)]
        pub fn splat(x: f64) -> F2 {
            F2([x, x])
        }
        /// # Safety
        /// `i + 2 <= s.len()`.
        #[inline(always)]
        pub unsafe fn load(s: &[f64], i: usize) -> F2 {
            F2([s[i], s[i + 1]])
        }
        /// # Safety
        /// `i + 2 <= s.len()`.
        #[inline(always)]
        pub unsafe fn store(self, s: &mut [f64], i: usize) {
            s[i] = self.0[0];
            s[i + 1] = self.0[1];
        }
        #[inline(always)]
        pub fn add(self, o: F2) -> F2 {
            lanes!(self, o, |a: f64, b: f64| a + b)
        }
        #[inline(always)]
        pub fn sub(self, o: F2) -> F2 {
            lanes!(self, o, |a: f64, b: f64| a - b)
        }
        #[inline(always)]
        pub fn mul(self, o: F2) -> F2 {
            lanes!(self, o, |a: f64, b: f64| a * b)
        }
        #[inline(always)]
        pub fn div(self, o: F2) -> F2 {
            lanes!(self, o, |a: f64, b: f64| a / b)
        }
        #[inline(always)]
        pub fn sqrt(self) -> F2 {
            F2([self.0[0].sqrt(), self.0[1].sqrt()])
        }
        #[inline(always)]
        pub fn abs(self) -> F2 {
            F2([self.0[0].abs(), self.0[1].abs()])
        }
        #[inline(always)]
        pub fn neg(self) -> F2 {
            F2([-self.0[0], -self.0[1]])
        }
        #[inline(always)]
        pub fn max_js(self, o: F2) -> F2 {
            lanes!(self, o, math::max)
        }
        #[inline(always)]
        pub fn min_js(self, o: F2) -> F2 {
            lanes!(self, o, math::min)
        }
        #[inline(always)]
        pub fn lt(self, o: F2) -> M2 {
            M2([self.0[0] < o.0[0], self.0[1] < o.0[1]])
        }
        #[inline(always)]
        pub fn le(self, o: F2) -> M2 {
            M2([self.0[0] <= o.0[0], self.0[1] <= o.0[1]])
        }
        #[inline(always)]
        pub fn gt(self, o: F2) -> M2 {
            M2([self.0[0] > o.0[0], self.0[1] > o.0[1]])
        }
        #[inline(always)]
        pub fn ge(self, o: F2) -> M2 {
            M2([self.0[0] >= o.0[0], self.0[1] >= o.0[1]])
        }
        #[inline(always)]
        pub fn lane0(self) -> f64 {
            self.0[0]
        }
        #[inline(always)]
        pub fn lane1(self) -> f64 {
            self.0[1]
        }
    }

    /// `m ? a : b`, lane by lane.
    #[inline(always)]
    pub fn select(m: M2, a: F2, b: F2) -> F2 {
        F2([if m.0[0] { a.0[0] } else { b.0[0] }, if m.0[1] { a.0[1] } else { b.0[1] }])
    }
}

pub use imp::{F2, M2, select};

impl core::ops::Add for F2 {
    type Output = F2;
    #[inline(always)]
    fn add(self, o: F2) -> F2 {
        F2::add(self, o)
    }
}

impl core::ops::Sub for F2 {
    type Output = F2;
    #[inline(always)]
    fn sub(self, o: F2) -> F2 {
        F2::sub(self, o)
    }
}

impl core::ops::Mul for F2 {
    type Output = F2;
    #[inline(always)]
    fn mul(self, o: F2) -> F2 {
        F2::mul(self, o)
    }
}

impl core::ops::Div for F2 {
    type Output = F2;
    #[inline(always)]
    fn div(self, o: F2) -> F2 {
        F2::div(self, o)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math;

    #[test]
    fn max_and_min_match_javascript() {
        let cases = [
            (1.0, 2.0),
            (2.0, 1.0),
            (-0.0, 0.0),
            (0.0, -0.0),
            (-0.0, -0.0),
            (f64::NAN, 1.0),
            (1.0, f64::NAN),
            (f64::INFINITY, -1.0),
        ];
        for (a, b) in cases {
            let v = F2::splat(a).max_js(F2::splat(b)).lane0();
            let w = math::max(a, b);
            assert!(v.to_bits() == w.to_bits() || (v.is_nan() && w.is_nan()), "max({a}, {b}) = {v}, want {w}");
            let v = F2::splat(a).min_js(F2::splat(b)).lane1();
            let w = math::min(a, b);
            assert!(v.to_bits() == w.to_bits() || (v.is_nan() && w.is_nan()), "min({a}, {b}) = {v}, want {w}");
        }
    }

    #[test]
    fn select_takes_the_first_where_the_mask_is_set() {
        let (a, b) = unsafe { (F2::load(&[1.0, 2.0], 0), F2::load(&[3.0, 1.0], 0)) };
        let m = a.lt(b);
        let s = select(m, a, b);
        assert_eq!((s.lane0(), s.lane1()), (1.0, 1.0));
    }
}
