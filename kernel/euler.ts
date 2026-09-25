/**
 * Wasm SIMD kernel for the two hot loops of the quasi-1D Euler solver.
 *
 * **This is AssemblyScript, not TypeScript.** It is deliberately outside `src/` so the
 * project's `tsc --noEmit` never sees `v128` or `usize`. Build it with
 * `npm run build:kernel`, which regenerates `src/audio/worklet/kernelWasm.ts`.
 *
 * Why this file exists: a CPU profile of the TypeScript path on the V-twin preset (56.1% of
 * one core) puts 72.4%
 * of total runtime in `reconstruct` (37.7%), `update` (19.3%) and `hllc` (15.4%) — three
 * straight-line loops over `Float64Array`. The working set is ~17 kB, so it is already
 * L1-resident and therefore ALU-bound, not memory-bound. The only lever left is wider
 * arithmetic, and V8 does not autovectorize. Hence explicit `f64x2`.
 *
 * Two rules govern every line here:
 *
 * 1. **Expression order is copied from the TypeScript, associativity included.** `0.5 * r *
 *    u * u` is `((0.5 * r) * u) * u`, not `0.5 * (r * (u * u))`. Float addition is not
 *    associative, so a "tidier" grouping changes the last bits and the differential test
 *    (`test/kernel.test.ts`) fails. Loop-invariant scalar products like `0.5 * dt` are
 *    hoisted, which is exact because they do not involve a per-cell value.
 *
 * 2. **NaN semantics are matched, not approximated.** Two different clamps appear in the
 *    solver and they behave differently on NaN:
 *      - `Math.max(x, k)` propagates NaN  -> `f64x2.max`, which also propagates (verified).
 *      - `x > k ? x : k` *discards* NaN (the compare is false, so `k` wins)
 *        -> `v128.bitselect(x, k, f64x2.gt(x, k))`, which also discards it.
 *    Substituting `max` for the ternary would silently rescue states the TS path treats as
 *    broken, and `recoveries` / `resetToQuiescent` are load-bearing on the audio thread.
 *
 * The scheme itself is documented in `src/audio/worklet/eulerPipe.ts`; nothing about the
 * physics changes here.
 */

// ---------------------------------------------------------------------------
// Memory layout
// ---------------------------------------------------------------------------

/**
 * Cells-plus-faces this kernel is compiled for.
 *
 * Fixed rather than sized per duct so the field offsets are compile-time constants. The
 * TypeScript side checks `n + 2 <= CAP` and falls back to its own loops if a caller asks
 * for a longer duct than this, so raising `maxCells` past it degrades rather than breaks.
 */
const CAP: i32 = 256;

/** Number of f64 fields in the block. Must equal `KERNEL_FIELDS.length` on the TS side. */
const NARR: i32 = 24;

/** Bytes per field. */
const STRIDE8: usize = <usize>CAP * 8;

/**
 * One static, 16-byte-aligned block holding every field.
 *
 * `memory.data` lets the compiler place it, so there is no guessing about where AS put its
 * own statics — assuming a fixed low offset instead is exactly the kind of thing that works
 * until a compiler upgrade moves it.
 */
const BASE: usize = memory.data(NARR * CAP * 8, 16);

export function baseOffset(): usize {
  return BASE;
}
export function capacity(): i32 {
  return CAP;
}
export function stride(): i32 {
  return CAP;
}

/**
 * The step's scalars, passed through memory rather than as arguments.
 *
 * Every duct has its own instance of this module, so each call site in the solver reaches several
 * different export functions. V8 only lowers a call into Wasm to a direct one when the target is a
 * single known function; otherwise it takes the generic path, which boxes every `f64` argument and
 * result into a fresh heap object: five per duct per substep, were these scalars passed as arguments.
 * Integers cross unboxed either way.
 */
const IO: usize = memory.data(4 * 8, 8);
const IO_DT: usize = 0;
const IO_KLIN: usize = 8;
const IO_DARCY: usize = 16;
const IO_MAX_SPEED: usize = 24;

export function ioOffset(): usize {
  return IO;
}

/** `reconstruct`, with `dt` read from the I/O block and the largest `|u| + c` written back to it. */
export function reconstructIo(n: i32, limiter: i32): void {
  store<f64>(IO + IO_MAX_SPEED, reconstruct(n, load<f64>(IO + IO_DT), limiter));
}

/** `update`, with `dt`, `kLin` and `darcy` read from the I/O block. */
export function updateIo(n: i32): void {
  update(n, load<f64>(IO + IO_DT), load<f64>(IO + IO_KLIN), load<f64>(IO + IO_DARCY));
}

// Field indices. The TypeScript loader builds its views in this order.
const I_RHO: i32 = 0;
const I_MOM: i32 = 1;
const I_EN: i32 = 2;
const I_PR: i32 = 3;
const I_PU: i32 = 4;
const I_PP: i32 = 5;
const I_SR: i32 = 6;
const I_SU: i32 = 7;
const I_SP: i32 = 8;
const I_LR: i32 = 9;
const I_LU: i32 = 10;
const I_LP: i32 = 11;
const I_RR: i32 = 12;
const I_RU: i32 = 13;
const I_RP: i32 = 14;
const I_F0: i32 = 15;
const I_F1: i32 = 16;
const I_F2: i32 = 17;
const I_FP: i32 = 18;
const I_AREAFACE: i32 = 19;
const I_INVVOL: i32 = 20;
const I_INVDIA: i32 = 21;
const I_CONTRACTIONK: i32 = 22;
const I_UMEAN: i32 = 23;

// @ts-ignore: AssemblyScript decorator
@inline
function fld(k: i32): usize {
  return BASE + <usize>k * STRIDE8;
}

// ---------------------------------------------------------------------------
// Constants, supplied by the caller
// ---------------------------------------------------------------------------

/**
 * Set from `GAS.gammaExh` and friends rather than duplicated here.
 *
 * Hardcoding gamma in two languages is a divergence waiting to happen: the TS side would be
 * changed, this file would not, and the differential test would be the only thing standing
 * between that and a solver whose two halves disagree about the working fluid. The derived
 * quantities are recomputed from the same expressions the TS module uses, so they are
 * bit-identical rather than merely close.
 */
let GAMMA: f64 = 1.33;
let GM1: f64 = 0.33;
let INV_GM1: f64 = 1.0 / 0.33;
let MIN_INTERNAL: f64 = 1e-3 / 0.33;
let MEAN_FLOW_RATE: f64 = 1.25;

export function setConstants(gamma: f64, meanFlowRate: f64): void {
  GAMMA = gamma;
  GM1 = gamma - 1;
  INV_GM1 = 1 / GM1;
  MIN_INTERNAL = 1e-3 / GM1;
  MEAN_FLOW_RATE = meanFlowRate;
}

/** Limiter codes, matching `Limiter` in eulerPipe.ts. */
const LIM_MC: i32 = 0;
const LIM_MINMOD: i32 = 1;
const LIM_VANLEER: i32 = 2;

// ---------------------------------------------------------------------------
// Limiters
// ---------------------------------------------------------------------------
//
// Each is the vector transcription of the scalar function of the same name. The guard
// `a * b <= 0` is false when the product is NaN, in both the scalar and the vector form, so
// a NaN slope falls through to the arithmetic branch in both — as it must, or a poisoned
// cell would quietly limit to zero here instead of being caught downstream.

// @ts-ignore
@inline
function vMinmod(a: v128, b: v128, zero: v128): v128 {
  const flat = f64x2.le(f64x2.mul(a, b), zero);
  const pick = v128.bitselect(a, b, f64x2.lt(f64x2.abs(a), f64x2.abs(b)));
  return v128.bitselect(zero, pick, flat);
}

// @ts-ignore
@inline
function vVanLeer(a: v128, b: v128, zero: v128, two: v128): v128 {
  const prod = f64x2.mul(a, b);
  const flat = f64x2.le(prod, zero);
  const pick = f64x2.div(f64x2.mul(two, prod), f64x2.add(a, b));
  return v128.bitselect(zero, pick, flat);
}

// @ts-ignore
@inline
function vMc(a: v128, b: v128, zero: v128, half: v128, two: v128): v128 {
  const flat = f64x2.le(f64x2.mul(a, b), zero);
  const c = f64x2.mul(half, f64x2.add(a, b));
  const ac = f64x2.abs(c);
  const m = f64x2.min(
    ac,
    f64x2.min(f64x2.mul(two, f64x2.abs(a)), f64x2.mul(two, f64x2.abs(b))),
  );
  // `c < 0 ? -m : m`. `f64x2.neg` rather than `0 - m`, so a zero slope keeps the sign the
  // scalar version would have produced.
  const signed = v128.bitselect(f64x2.neg(m), m, f64x2.lt(c, zero));
  return v128.bitselect(zero, signed, flat);
}

// @ts-ignore
@inline
function sMinmod(a: f64, b: f64): f64 {
  if (a * b <= 0) return 0;
  return Math.abs(a) < Math.abs(b) ? a : b;
}

// @ts-ignore
@inline
function sVanLeer(a: f64, b: f64): f64 {
  if (a * b <= 0) return 0;
  return (2 * a * b) / (a + b);
}

// @ts-ignore
@inline
function sMc(a: f64, b: f64): f64 {
  if (a * b <= 0) return 0;
  const c = 0.5 * (a + b);
  const ac = Math.abs(c);
  const m = Math.min(ac, Math.min(2 * Math.abs(a), 2 * Math.abs(b)));
  return c < 0 ? -m : m;
}

// ---------------------------------------------------------------------------
// Riemann solver
// ---------------------------------------------------------------------------

/**
 * HLLC at one face, scalar. A transcription of `hllcSolve` in eulerPipe.ts, used for the odd
 * face left over by the vector loop.
 */
function hllcScalar(
  rL: f64,
  uL: f64,
  pL: f64,
  rR: f64,
  uR: f64,
  pR: f64,
  idx: i32,
): void {
  const f0 = fld(I_F0);
  const f1 = fld(I_F1);
  const f2 = fld(I_F2);
  const fp = fld(I_FP);
  const o = <usize>idx * 8;

  const invRL = 1 / rL;
  const invRR = 1 / rR;
  const cL = Math.sqrt(GAMMA * pL * invRL);
  const cR = Math.sqrt(GAMMA * pR * invRR);
  const eL = pL * INV_GM1 + 0.5 * rL * uL * uL;
  const eR = pR * INV_GM1 + 0.5 * rR * uR * uR;

  const sL = Math.min(uL - cL, uR - cR);
  const sR = Math.max(uL + cL, uR + cR);

  if (sL >= 0) {
    store<f64>(f0 + o, rL * uL);
    store<f64>(f1 + o, rL * uL * uL + pL);
    store<f64>(f2 + o, (eL + pL) * uL);
    store<f64>(fp + o, pL);
    return;
  }
  if (sR <= 0) {
    store<f64>(f0 + o, rR * uR);
    store<f64>(f1 + o, rR * uR * uR + pR);
    store<f64>(f2 + o, (eR + pR) * uR);
    store<f64>(fp + o, pR);
    return;
  }

  const mL = rL * (sL - uL);
  const mR = rR * (sR - uR);
  const denom = mL - mR;
  const sStar = Math.abs(denom) < 1e-12 ? 0 : (pR - pL + mL * uL - mR * uR) / denom;

  store<f64>(fp + o, Math.max(pL + mL * (sStar - uL), 1e-3));

  if (sStar >= 0) {
    const f = mL / (sL - sStar);
    store<f64>(f0 + o, rL * uL + sL * (f - rL));
    store<f64>(f1 + o, rL * uL * uL + pL + sL * (f * sStar - rL * uL));
    store<f64>(
      f2 + o,
      (eL + pL) * uL + sL * (f * (eL * invRL + (sStar - uL) * (sStar + pL / mL)) - eL),
    );
  } else {
    const f = mR / (sR - sStar);
    store<f64>(f0 + o, rR * uR + sR * (f - rR));
    store<f64>(f1 + o, rR * uR * uR + pR + sR * (f * sStar - rR * uR));
    store<f64>(
      f2 + o,
      (eR + pR) * uR + sR * (f * (eR * invRR + (sStar - uR) * (sStar + pR / mR)) - eR),
    );
  }
}

/**
 * HLLC at two faces at once, branchless.
 *
 * The scalar version returns early on a supersonic face and picks one of two star branches
 * otherwise. Lanes cannot diverge, so all four outcomes are computed and selected. That
 * costs roughly 1.7x the scalar flops to fill two lanes, which is why the profile's 15.4%
 * in `hllc` is the part of this work SIMD helps least — around 1.15x rather than 2x.
 *
 * The discarded lanes may contain Inf or NaN from a division that the scalar path never
 * performs. That is safe: wasm float ops do not trap, and `bitselect` chooses bit patterns,
 * so nothing leaks out of a lane that was not selected. The two divisors that matter cannot
 * be zero in a *selected* lane anyway — `sL <= uL - cL < uL` makes `mL` strictly negative
 * and `sR >= uR + cR > uR` makes `mR` strictly positive, which is the same property the
 * scalar code already relies on.
 */
// @ts-ignore
@inline
function vHllc(
  rL: v128,
  uL: v128,
  pL: v128,
  rR: v128,
  uR: v128,
  pR: v128,
  idx: i32,
): void {
  const zero = f64x2.splat(0);
  const half = f64x2.splat(0.5);
  const one = f64x2.splat(1);
  const gamma = f64x2.splat(GAMMA);
  const invGm1 = f64x2.splat(INV_GM1);

  const invRL = f64x2.div(one, rL);
  const invRR = f64x2.div(one, rR);
  const cL = f64x2.sqrt(f64x2.mul(f64x2.mul(gamma, pL), invRL));
  const cR = f64x2.sqrt(f64x2.mul(f64x2.mul(gamma, pR), invRR));
  // eL = pL * INV_GM1 + 0.5 * rL * uL * uL
  const eL = f64x2.add(
    f64x2.mul(pL, invGm1),
    f64x2.mul(f64x2.mul(f64x2.mul(half, rL), uL), uL),
  );
  const eR = f64x2.add(
    f64x2.mul(pR, invGm1),
    f64x2.mul(f64x2.mul(f64x2.mul(half, rR), uR), uR),
  );

  const sL = f64x2.min(f64x2.sub(uL, cL), f64x2.sub(uR, cR));
  const sR = f64x2.max(f64x2.add(uL, cL), f64x2.add(uR, cR));

  // Plain upwind fluxes, which double as the supersonic answers.
  const rLuL = f64x2.mul(rL, uL);
  const rRuR = f64x2.mul(rR, uR);
  const FL0 = rLuL;
  const FL1 = f64x2.add(f64x2.mul(rLuL, uL), pL);
  const FL2 = f64x2.mul(f64x2.add(eL, pL), uL);
  const FR0 = rRuR;
  const FR1 = f64x2.add(f64x2.mul(rRuR, uR), pR);
  const FR2 = f64x2.mul(f64x2.add(eR, pR), uR);

  const mL = f64x2.mul(rL, f64x2.sub(sL, uL));
  const mR = f64x2.mul(rR, f64x2.sub(sR, uR));
  const denom = f64x2.sub(mL, mR);
  const quot = f64x2.div(
    f64x2.sub(f64x2.add(f64x2.sub(pR, pL), f64x2.mul(mL, uL)), f64x2.mul(mR, uR)),
    denom,
  );
  // `Math.abs(denom) < 1e-12 ? 0 : quot`
  const sStar = v128.bitselect(
    zero,
    quot,
    f64x2.lt(f64x2.abs(denom), f64x2.splat(1e-12)),
  );

  const pStar = f64x2.max(
    f64x2.add(pL, f64x2.mul(mL, f64x2.sub(sStar, uL))),
    f64x2.splat(1e-3),
  );

  // Left star state.
  const fL = f64x2.div(mL, f64x2.sub(sL, sStar));
  const SL0 = f64x2.add(FL0, f64x2.mul(sL, f64x2.sub(fL, rL)));
  const SL1 = f64x2.add(
    f64x2.add(f64x2.mul(rLuL, uL), pL),
    f64x2.mul(sL, f64x2.sub(f64x2.mul(fL, sStar), rLuL)),
  );
  const SL2 = f64x2.add(
    FL2,
    f64x2.mul(
      sL,
      f64x2.sub(
        f64x2.mul(
          fL,
          f64x2.add(
            f64x2.mul(eL, invRL),
            f64x2.mul(f64x2.sub(sStar, uL), f64x2.add(sStar, f64x2.div(pL, mL))),
          ),
        ),
        eL,
      ),
    ),
  );

  // Right star state.
  const fR = f64x2.div(mR, f64x2.sub(sR, sStar));
  const SR0 = f64x2.add(FR0, f64x2.mul(sR, f64x2.sub(fR, rR)));
  const SR1 = f64x2.add(
    f64x2.add(f64x2.mul(rRuR, uR), pR),
    f64x2.mul(sR, f64x2.sub(f64x2.mul(fR, sStar), rRuR)),
  );
  const SR2 = f64x2.add(
    FR2,
    f64x2.mul(
      sR,
      f64x2.sub(
        f64x2.mul(
          fR,
          f64x2.add(
            f64x2.mul(eR, invRR),
            f64x2.mul(f64x2.sub(sStar, uR), f64x2.add(sStar, f64x2.div(pR, mR))),
          ),
        ),
        eR,
      ),
    ),
  );

  // Select, in the order the scalar branches are tested: supersonic left, then supersonic
  // right, then the star region.
  const starMask = f64x2.ge(sStar, zero);
  let o0 = v128.bitselect(SL0, SR0, starMask);
  let o1 = v128.bitselect(SL1, SR1, starMask);
  let o2 = v128.bitselect(SL2, SR2, starMask);
  let op = pStar;

  const rightMask = f64x2.le(sR, zero);
  o0 = v128.bitselect(FR0, o0, rightMask);
  o1 = v128.bitselect(FR1, o1, rightMask);
  o2 = v128.bitselect(FR2, o2, rightMask);
  op = v128.bitselect(pR, op, rightMask);

  const leftMask = f64x2.ge(sL, zero);
  o0 = v128.bitselect(FL0, o0, leftMask);
  o1 = v128.bitselect(FL1, o1, leftMask);
  o2 = v128.bitselect(FL2, o2, leftMask);
  op = v128.bitselect(pL, op, leftMask);

  const o = <usize>idx * 8;
  v128.store(fld(I_F0) + o, o0);
  v128.store(fld(I_F1) + o, o1);
  v128.store(fld(I_F2) + o, o2);
  v128.store(fld(I_FP) + o, op);
}

// ---------------------------------------------------------------------------
// reconstruct
// ---------------------------------------------------------------------------

/**
 * Primitives, limited slopes, Hancock half-step and every interior face flux.
 *
 * Returns the largest `|u| + c` seen, which the caller uses to size the next substep. The
 * two boundary faces are deliberately left alone, exactly as in the TS version, so a
 * junction can still read `endState` and impose a pressure between reconstruction and the
 * conservative update.
 */
export function reconstruct(n: i32, dt: f64, limiter: i32): f64 {
  const rho = fld(I_RHO);
  const mom = fld(I_MOM);
  const en = fld(I_EN);
  const pr = fld(I_PR);
  const pu = fld(I_PU);
  const pp = fld(I_PP);
  const srA = fld(I_SR);
  const suA = fld(I_SU);
  const spA = fld(I_SP);
  const lr = fld(I_LR);
  const lu = fld(I_LU);
  const lp = fld(I_LP);
  const rr = fld(I_RR);
  const ru = fld(I_RU);
  const rp = fld(I_RP);
  const areaFace = fld(I_AREAFACE);
  const invVol = fld(I_INVVOL);

  const zero = f64x2.splat(0);
  const half = f64x2.splat(0.5);
  const one = f64x2.splat(1);
  const two = f64x2.splat(2);
  const gammaV = f64x2.splat(GAMMA);
  const gm1V = f64x2.splat(GM1);
  const invGm1V = f64x2.splat(INV_GM1);
  const pFloor = f64x2.splat(1e-3);
  const rFloor = f64x2.splat(1e-7);

  // --- primitives, and the wave speed for the next substep ---
  let accV = zero;
  let i = 0;
  for (; i + 1 < n; i += 2) {
    const o = <usize>i * 8;
    const r = v128.load(rho + o);
    const invR = f64x2.div(one, r);
    const u = f64x2.mul(v128.load(mom + o), invR);
    // p = Math.max((GAMMA - 1) * (en[i] - 0.5 * r * u * u), 1e-3)
    const p = f64x2.max(
      f64x2.mul(
        gm1V,
        f64x2.sub(
          v128.load(en + o),
          f64x2.mul(f64x2.mul(f64x2.mul(half, r), u), u),
        ),
      ),
      pFloor,
    );
    v128.store(pr + o, r);
    v128.store(pu + o, u);
    v128.store(pp + o, p);
    const s = f64x2.add(
      f64x2.abs(u),
      f64x2.sqrt(f64x2.mul(f64x2.mul(gammaV, p), invR)),
    );
    // `s > acc ? s : acc`, which skips NaN just as the scalar comparison does. `f64x2.max`
    // would propagate it and turn one poisoned cell into a non-finite wave speed.
    accV = v128.bitselect(s, accV, f64x2.gt(s, accV));
  }
  let maxSpeed = f64x2.extract_lane(accV, 0);
  const lane1 = f64x2.extract_lane(accV, 1);
  if (lane1 > maxSpeed) maxSpeed = lane1;
  for (; i < n; i++) {
    const o = <usize>i * 8;
    const r = load<f64>(rho + o);
    const invR = 1 / r;
    const u = load<f64>(mom + o) * invR;
    const p = Math.max(GM1 * (load<f64>(en + o) - 0.5 * r * u * u), 1e-3);
    store<f64>(pr + o, r);
    store<f64>(pu + o, u);
    store<f64>(pp + o, p);
    const s = Math.abs(u) + Math.sqrt(GAMMA * p * invR);
    if (s > maxSpeed) maxSpeed = s;
  }

  // --- limited slopes on primitives ---
  // End cells peeled off, as in the TS version, so the hot body carries no boundary test.
  store<f64>(srA, 0);
  store<f64>(suA, 0);
  store<f64>(spA, 0);
  const lastO = <usize>(n - 1) * 8;
  store<f64>(srA + lastO, 0);
  store<f64>(suA + lastO, 0);
  store<f64>(spA + lastO, 0);

  const slopeEnd = n - 1;
  i = 1;
  if (limiter == LIM_MINMOD) {
    for (; i + 1 < slopeEnd; i += 2) {
      const o = <usize>i * 8;
      const cR = v128.load(pr + o);
      const cU = v128.load(pu + o);
      const cP = v128.load(pp + o);
      v128.store(
        srA + o,
        vMinmod(f64x2.sub(cR, v128.load(pr + o - 8)), f64x2.sub(v128.load(pr + o + 8), cR), zero),
      );
      v128.store(
        suA + o,
        vMinmod(f64x2.sub(cU, v128.load(pu + o - 8)), f64x2.sub(v128.load(pu + o + 8), cU), zero),
      );
      v128.store(
        spA + o,
        vMinmod(f64x2.sub(cP, v128.load(pp + o - 8)), f64x2.sub(v128.load(pp + o + 8), cP), zero),
      );
    }
    for (; i < slopeEnd; i++) {
      const o = <usize>i * 8;
      store<f64>(srA + o, sMinmod(load<f64>(pr + o) - load<f64>(pr + o - 8), load<f64>(pr + o + 8) - load<f64>(pr + o)));
      store<f64>(suA + o, sMinmod(load<f64>(pu + o) - load<f64>(pu + o - 8), load<f64>(pu + o + 8) - load<f64>(pu + o)));
      store<f64>(spA + o, sMinmod(load<f64>(pp + o) - load<f64>(pp + o - 8), load<f64>(pp + o + 8) - load<f64>(pp + o)));
    }
  } else if (limiter == LIM_VANLEER) {
    for (; i + 1 < slopeEnd; i += 2) {
      const o = <usize>i * 8;
      const cR = v128.load(pr + o);
      const cU = v128.load(pu + o);
      const cP = v128.load(pp + o);
      v128.store(
        srA + o,
        vVanLeer(f64x2.sub(cR, v128.load(pr + o - 8)), f64x2.sub(v128.load(pr + o + 8), cR), zero, two),
      );
      v128.store(
        suA + o,
        vVanLeer(f64x2.sub(cU, v128.load(pu + o - 8)), f64x2.sub(v128.load(pu + o + 8), cU), zero, two),
      );
      v128.store(
        spA + o,
        vVanLeer(f64x2.sub(cP, v128.load(pp + o - 8)), f64x2.sub(v128.load(pp + o + 8), cP), zero, two),
      );
    }
    for (; i < slopeEnd; i++) {
      const o = <usize>i * 8;
      store<f64>(srA + o, sVanLeer(load<f64>(pr + o) - load<f64>(pr + o - 8), load<f64>(pr + o + 8) - load<f64>(pr + o)));
      store<f64>(suA + o, sVanLeer(load<f64>(pu + o) - load<f64>(pu + o - 8), load<f64>(pu + o + 8) - load<f64>(pu + o)));
      store<f64>(spA + o, sVanLeer(load<f64>(pp + o) - load<f64>(pp + o - 8), load<f64>(pp + o + 8) - load<f64>(pp + o)));
    }
  } else {
    for (; i + 1 < slopeEnd; i += 2) {
      const o = <usize>i * 8;
      const cR = v128.load(pr + o);
      const cU = v128.load(pu + o);
      const cP = v128.load(pp + o);
      v128.store(
        srA + o,
        vMc(f64x2.sub(cR, v128.load(pr + o - 8)), f64x2.sub(v128.load(pr + o + 8), cR), zero, half, two),
      );
      v128.store(
        suA + o,
        vMc(f64x2.sub(cU, v128.load(pu + o - 8)), f64x2.sub(v128.load(pu + o + 8), cU), zero, half, two),
      );
      v128.store(
        spA + o,
        vMc(f64x2.sub(cP, v128.load(pp + o - 8)), f64x2.sub(v128.load(pp + o + 8), cP), zero, half, two),
      );
    }
    for (; i < slopeEnd; i++) {
      const o = <usize>i * 8;
      store<f64>(srA + o, sMc(load<f64>(pr + o) - load<f64>(pr + o - 8), load<f64>(pr + o + 8) - load<f64>(pr + o)));
      store<f64>(suA + o, sMc(load<f64>(pu + o) - load<f64>(pu + o - 8), load<f64>(pu + o + 8) - load<f64>(pu + o)));
      store<f64>(spA + o, sMc(load<f64>(pp + o) - load<f64>(pp + o - 8), load<f64>(pp + o + 8) - load<f64>(pp + o)));
    }
  }

  // --- half-step evolution (Hancock) ---
  // `0.5 * dt` is loop-invariant and hoisted; the product with `invVol[i]` stays per-cell,
  // which is what the TS left-associative `0.5 * dt * invVol[i]` computes.
  const halfDt = 0.5 * dt;
  const halfDtV = f64x2.splat(halfDt);
  i = 0;
  for (; i + 1 < n; i += 2) {
    const o = <usize>i * 8;
    const sr = v128.load(srA + o);
    const su = v128.load(suA + o);
    const sp = v128.load(spA + o);
    const cR = v128.load(pr + o);
    const cU = v128.load(pu + o);
    const cP = v128.load(pp + o);

    const rL = f64x2.sub(cR, f64x2.mul(half, sr));
    const uL = f64x2.sub(cU, f64x2.mul(half, su));
    const pLv = f64x2.sub(cP, f64x2.mul(half, sp));
    const rR = f64x2.add(cR, f64x2.mul(half, sr));
    const uR = f64x2.add(cU, f64x2.mul(half, su));
    const pRv = f64x2.add(cP, f64x2.mul(half, sp));

    const UL1 = f64x2.mul(rL, uL);
    const UL2 = f64x2.add(
      f64x2.mul(pLv, invGm1V),
      f64x2.mul(f64x2.mul(f64x2.mul(half, rL), uL), uL),
    );
    const UR1 = f64x2.mul(rR, uR);
    const UR2 = f64x2.add(
      f64x2.mul(pRv, invGm1V),
      f64x2.mul(f64x2.mul(f64x2.mul(half, rR), uR), uR),
    );

    const aLh = v128.load(areaFace + o);
    const aRh = v128.load(areaFace + o + 8);
    const hk = f64x2.mul(halfDtV, v128.load(invVol + o));

    const d0 = f64x2.mul(hk, f64x2.sub(f64x2.mul(aLh, UL1), f64x2.mul(aRh, UR1)));
    const d1 = f64x2.add(
      f64x2.mul(
        hk,
        f64x2.sub(
          f64x2.mul(aLh, f64x2.add(f64x2.mul(UL1, uL), pLv)),
          f64x2.mul(aRh, f64x2.add(f64x2.mul(UR1, uR), pRv)),
        ),
      ),
      f64x2.mul(f64x2.mul(hk, cP), f64x2.sub(aRh, aLh)),
    );
    const d2 = f64x2.mul(
      hk,
      f64x2.sub(
        f64x2.mul(f64x2.mul(aLh, f64x2.add(UL2, pLv)), uL),
        f64x2.mul(f64x2.mul(aRh, f64x2.add(UR2, pRv)), uR),
      ),
    );

    // Left face state. `a0 > 1e-7 ? a0 : 1e-7` discards NaN, so bitselect rather than max.
    let a0 = f64x2.add(rL, d0);
    let a1 = f64x2.add(UL1, d1);
    let a2 = f64x2.add(UL2, d2);
    let dens = v128.bitselect(a0, rFloor, f64x2.gt(a0, rFloor));
    let invDens = f64x2.div(one, dens);
    v128.store(lr + o, dens);
    v128.store(lu + o, f64x2.mul(a1, invDens));
    v128.store(
      lp + o,
      f64x2.max(
        f64x2.mul(
          gm1V,
          f64x2.sub(a2, f64x2.mul(f64x2.mul(f64x2.mul(half, a1), a1), invDens)),
        ),
        pFloor,
      ),
    );

    a0 = f64x2.add(rR, d0);
    a1 = f64x2.add(UR1, d1);
    a2 = f64x2.add(UR2, d2);
    dens = v128.bitselect(a0, rFloor, f64x2.gt(a0, rFloor));
    invDens = f64x2.div(one, dens);
    v128.store(rr + o, dens);
    v128.store(ru + o, f64x2.mul(a1, invDens));
    v128.store(
      rp + o,
      f64x2.max(
        f64x2.mul(
          gm1V,
          f64x2.sub(a2, f64x2.mul(f64x2.mul(f64x2.mul(half, a1), a1), invDens)),
        ),
        pFloor,
      ),
    );
  }
  for (; i < n; i++) {
    const o = <usize>i * 8;
    const sr = load<f64>(srA + o);
    const su = load<f64>(suA + o);
    const sp = load<f64>(spA + o);
    const cR = load<f64>(pr + o);
    const cU = load<f64>(pu + o);
    const cP = load<f64>(pp + o);

    const rL = cR - 0.5 * sr;
    const uL = cU - 0.5 * su;
    const pLs = cP - 0.5 * sp;
    const rR = cR + 0.5 * sr;
    const uR = cU + 0.5 * su;
    const pRs = cP + 0.5 * sp;

    const UL1 = rL * uL;
    const UL2 = pLs * INV_GM1 + 0.5 * rL * uL * uL;
    const UR1 = rR * uR;
    const UR2 = pRs * INV_GM1 + 0.5 * rR * uR * uR;

    const aLh = load<f64>(areaFace + o);
    const aRh = load<f64>(areaFace + o + 8);
    const hk = halfDt * load<f64>(invVol + o);

    const d0 = hk * (aLh * UL1 - aRh * UR1);
    const d1 = hk * (aLh * (UL1 * uL + pLs) - aRh * (UR1 * uR + pRs)) + hk * cP * (aRh - aLh);
    const d2 = hk * (aLh * (UL2 + pLs) * uL - aRh * (UR2 + pRs) * uR);

    let a0 = rL + d0;
    let a1 = UL1 + d1;
    let a2 = UL2 + d2;
    let dens = a0 > 1e-7 ? a0 : 1e-7;
    let invDens = 1 / dens;
    store<f64>(lr + o, dens);
    store<f64>(lu + o, a1 * invDens);
    store<f64>(lp + o, Math.max(GM1 * (a2 - 0.5 * a1 * a1 * invDens), 1e-3));

    a0 = rR + d0;
    a1 = UR1 + d1;
    a2 = UR2 + d2;
    dens = a0 > 1e-7 ? a0 : 1e-7;
    invDens = 1 / dens;
    store<f64>(rr + o, dens);
    store<f64>(ru + o, a1 * invDens);
    store<f64>(rp + o, Math.max(GM1 * (a2 - 0.5 * a1 * a1 * invDens), 1e-3));
  }

  // --- interior faces ---
  // Face f sits between cells f-1 and f, so both sides load contiguously.
  let f = 1;
  for (; f + 1 < n; f += 2) {
    const lo = <usize>(f - 1) * 8;
    const ro = <usize>f * 8;
    vHllc(
      v128.load(rr + lo),
      v128.load(ru + lo),
      v128.load(rp + lo),
      v128.load(lr + ro),
      v128.load(lu + ro),
      v128.load(lp + ro),
      f,
    );
  }
  for (; f < n; f++) {
    const lo = <usize>(f - 1) * 8;
    const ro = <usize>f * 8;
    hllcScalar(
      load<f64>(rr + lo),
      load<f64>(ru + lo),
      load<f64>(rp + lo),
      load<f64>(lr + ro),
      load<f64>(lu + ro),
      load<f64>(lp + ro),
      f,
    );
  }

  return maxSpeed;
}

// ---------------------------------------------------------------------------
// update
// ---------------------------------------------------------------------------

/**
 * Conservative update, well-balanced area source and friction, over every cell.
 *
 * The valve source into cell 0 stays on the TypeScript side: it is scalar, touches one cell,
 * and needs the cylinder state, so there is nothing for SIMD to do and a good deal of
 * plumbing to avoid.
 */
export function update(n: i32, dt: f64, kLin: f64, darcy: f64): void {
  const rho = fld(I_RHO);
  const mom = fld(I_MOM);
  const en = fld(I_EN);
  const f0 = fld(I_F0);
  const f1 = fld(I_F1);
  const f2 = fld(I_F2);
  const fp = fld(I_FP);
  const areaFace = fld(I_AREAFACE);
  const invVol = fld(I_INVVOL);
  const invDia = fld(I_INVDIA);
  const contractionK = fld(I_CONTRACTIONK);
  const uMeanArr = fld(I_UMEAN);

  const half = f64x2.splat(0.5);
  const one = f64x2.splat(1);
  const zeroV = f64x2.splat(0);
  const rFloor = f64x2.splat(1e-7);
  const minInt = f64x2.splat(MIN_INTERNAL);
  const dtV = f64x2.splat(dt);
  const kLinV = f64x2.splat(kLin);
  // `darcy * 0.5` and `dt * MEAN_FLOW_RATE` are loop-invariant, and left-associativity in
  // the TS puts them in exactly this grouping.
  const darcyHalf = f64x2.splat(darcy * 0.5);
  const meanStep = f64x2.splat(dt * MEAN_FLOW_RATE);

  let i = 0;
  for (; i + 1 < n; i += 2) {
    const o = <usize>i * 8;
    const aL = v128.load(areaFace + o);
    const aR = v128.load(areaFace + o + 8);
    const k = f64x2.mul(dtV, v128.load(invVol + o));

    let nr = f64x2.sub(
      v128.load(rho + o),
      f64x2.mul(
        k,
        f64x2.sub(f64x2.mul(aR, v128.load(f0 + o + 8)), f64x2.mul(aL, v128.load(f0 + o))),
      ),
    );
    const pFace = f64x2.mul(half, f64x2.add(v128.load(fp + o), v128.load(fp + o + 8)));
    let nm = f64x2.add(
      f64x2.sub(
        v128.load(mom + o),
        f64x2.mul(
          k,
          f64x2.sub(f64x2.mul(aR, v128.load(f1 + o + 8)), f64x2.mul(aL, v128.load(f1 + o))),
        ),
      ),
      f64x2.mul(f64x2.mul(k, pFace), f64x2.sub(aR, aL)),
    );
    let ne = f64x2.sub(
      v128.load(en + o),
      f64x2.mul(
        k,
        f64x2.sub(f64x2.mul(aR, v128.load(f2 + o + 8)), f64x2.mul(aL, v128.load(f2 + o))),
      ),
    );

    // `if (nr < 1e-7) nr = 1e-7` — leaves NaN untouched, as the scalar compare does.
    nr = v128.bitselect(rFloor, nr, f64x2.lt(nr, rFloor));

    const invNr = f64x2.div(one, nr);
    const uOld = f64x2.mul(nm, invNr);
    const uMean = v128.load(uMeanArr + o);

    // The contraction loss is signed by the direction it acts in: `max(u * k, 0)` is `|u| |k|`
    // when the flow runs into the narrowing and zero when it runs out of it.
    const kQuad = f64x2.add(
      f64x2.mul(f64x2.abs(uOld), f64x2.mul(darcyHalf, v128.load(invDia + o))),
      f64x2.max(f64x2.mul(uOld, v128.load(contractionK + o)), zeroV),
    );
    const uNew = f64x2.div(
      f64x2.add(uOld, f64x2.mul(f64x2.mul(kLinV, uMean), dtV)),
      f64x2.add(one, f64x2.mul(f64x2.add(kLinV, kQuad), dtV)),
    );
    nm = f64x2.mul(nr, uNew);

    v128.store(
      uMeanArr + o,
      f64x2.add(uMean, f64x2.mul(f64x2.sub(uNew, uMean), meanStep)),
    );

    const kinetic = f64x2.mul(f64x2.mul(f64x2.mul(half, nm), nm), invNr);
    // `if (ne - kinetic < MIN_INTERNAL) ne = kinetic + MIN_INTERNAL`
    ne = v128.bitselect(
      f64x2.add(kinetic, minInt),
      ne,
      f64x2.lt(f64x2.sub(ne, kinetic), minInt),
    );

    v128.store(rho + o, nr);
    v128.store(mom + o, nm);
    v128.store(en + o, ne);
  }

  for (; i < n; i++) {
    const o = <usize>i * 8;
    const aL = load<f64>(areaFace + o);
    const aR = load<f64>(areaFace + o + 8);
    const k = dt * load<f64>(invVol + o);

    let nr = load<f64>(rho + o) - k * (aR * load<f64>(f0 + o + 8) - aL * load<f64>(f0 + o));
    const pFace = 0.5 * (load<f64>(fp + o) + load<f64>(fp + o + 8));
    let nm =
      load<f64>(mom + o) -
      k * (aR * load<f64>(f1 + o + 8) - aL * load<f64>(f1 + o)) +
      k * pFace * (aR - aL);
    let ne = load<f64>(en + o) - k * (aR * load<f64>(f2 + o + 8) - aL * load<f64>(f2 + o));

    if (nr < 1e-7) nr = 1e-7;

    const invNr = 1 / nr;
    const uOld = nm * invNr;
    const uMean = load<f64>(uMeanArr + o);

    const kQuad =
      Math.abs(uOld) * (darcy * 0.5 * load<f64>(invDia + o)) +
      Math.max(uOld * load<f64>(contractionK + o), 0);
    const uNew = (uOld + kLin * uMean * dt) / (1 + (kLin + kQuad) * dt);
    nm = nr * uNew;

    store<f64>(uMeanArr + o, uMean + (uNew - uMean) * (dt * MEAN_FLOW_RATE));

    const kinetic = 0.5 * nm * nm * invNr;
    if (ne - kinetic < MIN_INTERNAL) ne = kinetic + MIN_INTERNAL;

    store<f64>(rho + o, nr);
    store<f64>(mom + o, nm);
    store<f64>(en + o, ne);
  }
}

// ---------------------------------------------------------------------------
// Junction solve
// ---------------------------------------------------------------------------

/**
 * The junction solve, which in TypeScript is about a quarter of a V8's time on manifold exhausts.
 *
 * Six junctions, each finding the pressure its branches balance at: a closed-form estimate, two Newton
 * corrections that each run a Riemann solve per branch, and the final fluxes — about fifty Riemann solves
 * a sample. In TypeScript every one of them is a call to a function too large to inline, which boxes
 * each floating-point argument into a fresh heap object unless it is routed through a typed array
 * (`HLLC_IN` in eulerPipe.ts); left as arguments, that would be most of the audio thread's garbage.
 *
 * Unlike the cell loops this spans several ducts, each with its own kernel instance, so it has a block of
 * its own: the caller writes each branch's end state in, calls `solveJunction`, and reads the fluxes
 * back. Nothing crosses the boundary but two integers in and the imbalance fraction out.
 *
 * The arithmetic is `EulerPipe.junctionFlux` and `ExhaustSystem.solveJunctions` line for line, order of
 * operations included, with one exception it cannot avoid: `Math.pow` here is AssemblyScript's, which
 * differs from the JavaScript engine's in the last bit now and then. In a nonlinear solver one bit grows,
 * so the output first differs after some tens of milliseconds, in the eighth significant figure, and never
 * becomes audible — `test/kernel.test.ts` holds it to both. That is why this kernel has its own switch:
 * the cell kernel's standard is bit-exactness, and folding this in would cost it that.
 */

/** Branches a junction may have. */
const J_CAP: i32 = 16;

// Per-branch fields: the end state in, the fluxes and diagnostics out.
const J_RHO: i32 = 0;
const J_U: i32 = 1;
const J_P: i32 = 2;
const J_TOWARD: i32 = 3;
const J_RHOC: i32 = 4;
const J_AREA: i32 = 5;
const J_C: i32 = 6;
/** Area of the face the flux passes through, which is what turns a flux density into kg/s. */
const J_FACE: i32 = 7;
const J_F0: i32 = 8;
const J_F1: i32 = 9;
const J_F2: i32 = 10;
const J_FP: i32 = 11;
const J_CLAMPS: i32 = 12;
const J_SUPERSONIC: i32 = 13;
const J_FIELDS: i32 = 14;

const JBASE: usize = memory.data(J_FIELDS * J_CAP * 8, 16);

@inline function jget(field: i32, i: i32): f64 {
  return load<f64>(JBASE + ((<usize>(field * J_CAP + i)) << 3));
}
@inline function jset(field: i32, i: i32, v: f64): void {
  store<f64>(JBASE + ((<usize>(field * J_CAP + i)) << 3), v);
}

export function junctionBase(): usize {
  return JBASE;
}
export function junctionCapacity(): i32 {
  return J_CAP;
}
export function junctionFields(): i32 {
  return J_FIELDS;
}

let jGamma: f64 = 1.33;
let jInvGamma: f64 = 1 / 1.33;
let jInvGm1: f64 = 1 / 0.33;
let jPAmb: f64 = 101325;
let jR: f64 = 287;
let jTAmb: f64 = 293;
let jCp: f64 = 1000;
let jMinRho: f64 = 0;
let jMinP: f64 = 0;
let jMaxSpeed: f64 = 1400;
let jAmbientC: f64 = 343;
let jTol: f64 = 0.005;

/** The gas constants and limits the solve uses, set once, so none of them crosses per call. */
export function setJunctionConstants(
  gamma: f64,
  invGamma: f64,
  invGm1: f64,
  pAmb: f64,
  r: f64,
  tAmb: f64,
  cp: f64,
  minRho: f64,
  minP: f64,
  maxSpeed: f64,
  ambientC: f64,
  tol: f64,
): void {
  jGamma = gamma;
  jInvGamma = invGamma;
  jInvGm1 = invGm1;
  jPAmb = pAmb;
  jR = r;
  jTAmb = tAmb;
  jCp = cp;
  jMinRho = minRho;
  jMinP = minP;
  jMaxSpeed = maxSpeed;
  jAmbientC = ambientC;
  jTol = tol;
}

// The last Riemann solve's four fluxes.
let hF0: f64 = 0;
let hF1: f64 = 0;
let hF2: f64 = 0;
let hFP: f64 = 0;

/** `hllcSolve` from eulerPipe.ts, scalar, into the four globals above. */
function junctionHllc(rL: f64, uL: f64, pL: f64, rR: f64, uR: f64, pR: f64): void {
  const invRL = 1 / rL;
  const invRR = 1 / rR;
  const cL = Math.sqrt(jGamma * pL * invRL);
  const cR = Math.sqrt(jGamma * pR * invRR);
  const eL = pL * jInvGm1 + 0.5 * rL * uL * uL;
  const eR = pR * jInvGm1 + 0.5 * rR * uR * uR;

  const sL = Math.min(uL - cL, uR - cR);
  const sR = Math.max(uL + cL, uR + cR);

  if (sL >= 0) {
    hF0 = rL * uL;
    hF1 = rL * uL * uL + pL;
    hF2 = (eL + pL) * uL;
    hFP = pL;
    return;
  }
  if (sR <= 0) {
    hF0 = rR * uR;
    hF1 = rR * uR * uR + pR;
    hF2 = (eR + pR) * uR;
    hFP = pR;
    return;
  }

  const mL = rL * (sL - uL);
  const mR = rR * (sR - uR);
  const denom = mL - mR;
  const sStar = Math.abs(denom) < 1e-12 ? 0 : (pR - pL + mL * uL - mR * uR) / denom;

  hFP = Math.max(pL + mL * (sStar - uL), 1e-3);

  if (sStar >= 0) {
    const f = mL / (sL - sStar);
    hF0 = rL * uL + sL * (f - rL);
    hF1 = rL * uL * uL + pL + sL * (f * sStar - rL * uL);
    hF2 = (eL + pL) * uL + sL * (f * (eL * invRL + (sStar - uL) * (sStar + pL / mL)) - eL);
  } else {
    const f = mR / (sR - sStar);
    hF0 = rR * uR + sR * (f - rR);
    hF1 = rR * uR * uR + pR + sR * (f * sStar - rR * uR);
    hF2 = (eR + pR) * uR + sR * (f * (eR * invRR + (sStar - uR) * (sStar + pR / mR)) - eR);
  }
}

/** `clamp` from dsp.ts: comparisons, so a NaN input comes out as `x`, not as a bound. */
@inline function jclamp(x: f64, lo: f64, hi: f64): f64 {
  return x < lo ? lo : x > hi ? hi : x;
}

/**
 * `EulerPipe.junctionFlux` for branch `i`: the mass flux, kg/s, it passes at this junction pressure.
 * Committing records its fluxes and counts its clamps; a probe changes nothing.
 */
function branchFlux(i: i32, outlet: bool, gauge: f64, tJunction: f64, commit: bool): f64 {
  const rho = jget(J_RHO, i);
  const u = jget(J_U, i);
  const p = jget(J_P, i);
  const toward = jget(J_TOWARD, i);
  const rhoCEnd = jget(J_RHOC, i);
  const area = jget(J_FACE, i);

  const cEnd = Math.sqrt((jGamma * Math.max(p, jMinP)) / Math.max(rho, jMinRho));
  const outward = outlet ? u : -u;
  if (outward >= cEnd) {
    if (commit) jset(J_SUPERSONIC, i, jget(J_SUPERSONIC, i) + 1);
    junctionHllc(rho, u, p, rho, u, p);
  } else {
    const returning = gauge - toward;
    const pGhost = Math.max(jPAmb + gauge, 1e-3);
    if (commit && (rho < jMinRho || rhoCEnd < jMinRho * jAmbientC)) {
      jset(J_CLAMPS, i, jget(J_CLAMPS, i) + 1);
    }
    const rhoSafe = Math.max(rho, jMinRho);
    const cLocal = Math.sqrt((jGamma * Math.max(p, jMinP)) / rhoSafe);
    const uLimit = Math.min(5 * cLocal, jMaxSpeed);
    const rhoC = Math.max(rhoCEnd, jMinRho * jAmbientC);
    const uRaw = outlet ? (toward - returning) / rhoC : (returning - toward) / rhoC;
    if (commit && Math.abs(uRaw) > uLimit) jset(J_CLAMPS, i, jget(J_CLAMPS, i) + 1);
    const uGhost = jclamp(uRaw, -uLimit, uLimit);
    const inflow = outlet ? uGhost < 0 : uGhost > 0;
    const rGhost = inflow
      ? Math.max(
          pGhost / (jR * Math.max(tJunction - (uGhost * uGhost) / (2 * jCp), jTAmb)),
          jMinRho,
        )
      : Math.max(rho * Math.pow(pGhost / p, jInvGamma), jMinRho);
    if (outlet) junctionHllc(rho, u, p, rGhost, uGhost, pGhost);
    else junctionHllc(rGhost, uGhost, pGhost, rho, u, p);
  }
  if (commit) {
    jset(J_F0, i, hF0);
    jset(J_F1, i, hF1);
    jset(J_F2, i, hF2);
    jset(J_FP, i, hFP);
  }
  return hF0 * area;
}

/**
 * Solve one junction whose first `nOut` branches end at it with their outlets and the rest with their
 * inlets, from the end states already written. Leaves each branch's committed fluxes and clamp counts,
 * and returns the mass imbalance as a fraction of the flow through it, or -1 if nothing flows.
 */
export function solveJunction(nEnds: i32, nOut: i32): f64 {
  let num: f64 = 0;
  let den: f64 = 0;
  let pMin: f64 = Infinity;
  let pMax: f64 = 0;
  let mIn: f64 = 0;
  let hIn: f64 = 0;
  let scaleGuess: f64 = 0;

  for (let i = 0; i < nEnds; i++) {
    const rho = jget(J_RHO, i);
    const u = jget(J_U, i);
    const p = jget(J_P, i);
    const area = jget(J_AREA, i);
    const w = area / jget(J_C, i);
    num += 2 * w * jget(J_TOWARD, i);
    den += w;
    if (p < pMin) pMin = p;
    if (p > pMax) pMax = p;
    scaleGuess += Math.abs(rho * area * u);
    const into = i < nOut ? u : -u;
    if (into > 0) {
      const m = rho * area * into;
      mIn += m;
      hIn += m * (p / (rho * jR) + (into * into) / (2 * jCp));
    }
    jset(J_CLAMPS, i, 0);
    jset(J_SUPERSONIC, i, 0);
  }

  const last = nEnds - 1;
  const tJunction =
    mIn > 1e-12 ? hIn / mIn : jget(J_P, last) / (Math.max(jget(J_RHO, last), 1e-7) * jR);

  let gauge = jclamp(jPAmb + (den > 0 ? num / den : 0), 0.3 * pMin, 3 * pMax) - jPAmb;

  if (den > 0) {
    const tol = jTol * Math.max(scaleGuess, 1e-9);
    for (let iter = 0; iter < 2; iter++) {
      let r: f64 = 0;
      for (let i = 0; i < nOut; i++) r += branchFlux(i, true, gauge, tJunction, false);
      for (let i = nOut; i < nEnds; i++) r -= branchFlux(i, false, gauge, tJunction, false);
      if (!isFinite(r) || Math.abs(r) <= tol) break;
      const next = jclamp(jPAmb + gauge + r / den, 0.3 * pMin, 3 * pMax) - jPAmb;
      if (next == gauge) break;
      gauge = next;
    }
  }

  let signed: f64 = 0;
  let scale: f64 = 0;
  for (let i = 0; i < nOut; i++) {
    const f = branchFlux(i, true, gauge, tJunction, true);
    signed += f;
    scale += Math.abs(f);
  }
  for (let i = nOut; i < nEnds; i++) {
    const f = branchFlux(i, false, gauge, tJunction, true);
    signed -= f;
    scale += Math.abs(f);
  }
  return scale > 1e-9 ? Math.abs(signed) / scale : -1;
}
