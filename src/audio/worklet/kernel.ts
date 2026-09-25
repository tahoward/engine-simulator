/**
 * Loader for the wasm SIMD Euler kernel.
 *
 * The kernel owns its fields: every array lives in the instance's linear memory, and the
 * `Float64Array`s handed back here are *views* onto that memory rather than copies. That is
 * the whole trick that makes this a drop-in — `EulerPipe` keeps using `this.rho[i]` from
 * TypeScript for boundaries, junctions and the thermal pass, while the two hot loops run in
 * wasm over the same bytes. Nothing is marshalled per substep.
 *
 * One instance per duct, all sharing one compiled `WebAssembly.Module`. Instances are cheap
 * (a page of memory each) and this way no duct's allocation can invalidate another duct's
 * views, which is exactly what would happen if a single shared memory had to grow.
 */

import { KERNEL_WASM_BASE64 } from './kernelWasm.js';

/**
 * Field order. **Must match the `I_*` indices in `kernel/euler.ts`.**
 *
 * The differential test asserts on every one of these, so a reordering that desynchronised
 * the two sides would fail loudly rather than quietly solve the wrong equations.
 */
export const KERNEL_FIELDS = [
  'rho',
  'mom',
  'en',
  'pr',
  'pu',
  'pp',
  'sr',
  'su',
  'sp',
  'lr',
  'lu',
  'lp',
  'rr',
  'ru',
  'rp',
  'f0',
  'f1',
  'f2',
  'fp',
  'areaFace',
  'invVol',
  'invDia',
  'contractionK',
  'uMean',
] as const;

export type KernelField = (typeof KERNEL_FIELDS)[number];

interface KernelExports {
  memory: WebAssembly.Memory;
  baseOffset(): number;
  capacity(): number;
  stride(): number;
  setConstants(gamma: number, meanFlowRate: number): void;
  reconstruct(n: number, dt: number, limiter: number): number;
  update(n: number, dt: number, kLin: number, darcy: number): void;
  ioOffset(): number;
  reconstructIo(n: number, limiter: number): void;
  updateIo(n: number): void;
}

/** Slots of the kernel's scalar I/O block. **Must match the `IO_*` offsets in `kernel/euler.ts`.** */
const IO_DT = 0;
const IO_KLIN = 1;
const IO_DARCY = 2;
const IO_MAX_SPEED = 3;

function decode(b64: string): Uint8Array {
  // `atob` in browsers and worklets, `Buffer` in Node. Neither environment has both
  // reliably, and this runs in both (vitest renders seconds of audio in Node).
  if (typeof atob === 'function') {
    const raw = atob(b64);
    const out = new Uint8Array(raw.length);
    for (let i = 0; i < raw.length; i++) out[i] = raw.charCodeAt(i);
    return out;
  }
  // Reached through `globalThis` so the project does not need `@types/node` for one line.
  const g = globalThis as unknown as {
    Buffer?: { from(s: string, enc: string): Uint8Array };
  };
  if (g.Buffer) return new Uint8Array(g.Buffer.from(b64, 'base64'));
  throw new Error('no base64 decoder available');
}

let cached: WebAssembly.Module | null = null;
let failed = false;

/**
 * Compile once, lazily.
 *
 * Lazily because synchronous `WebAssembly.Module` construction is capped at 4 kB on a
 * document's main thread, and this module is larger than that. An AudioWorklet and Node are
 * both exempt, so compiling on first *use* keeps the restriction irrelevant — as long as
 * nothing on the main thread touches the kernel, which nothing does.
 */
export function kernelModule(): WebAssembly.Module | null {
  if (cached !== null || failed) return cached;
  try {
    cached = new WebAssembly.Module(decode(KERNEL_WASM_BASE64) as BufferSource);
  } catch {
    // A host without SIMD, or without WebAssembly at all. The solver has a complete
    // TypeScript path; losing the kernel costs speed and nothing else.
    failed = true;
  }
  return cached;
}

/** True if the wasm kernel is usable at all in this environment. */
export function kernelAvailable(): boolean {
  return kernelModule() !== null;
}

export class EulerKernel {
  private readonly ex: KernelExports;
  /** Slots per field, including the extra face and the alignment slack. */
  readonly stride: number;
  readonly fields: Record<KernelField, Float64Array>;
  /** The scalars each step passes, through memory: see `ioOffset` in `kernel/euler.ts`. */
  private readonly io: Float64Array;

  private constructor(ex: KernelExports, gamma: number, meanFlowRate: number) {
    this.ex = ex;
    this.stride = ex.stride();
    const base = ex.baseOffset();
    const buf = ex.memory.buffer;
    const fields = {} as Record<KernelField, Float64Array>;
    for (let k = 0; k < KERNEL_FIELDS.length; k++) {
      fields[KERNEL_FIELDS[k]!] = new Float64Array(buf, base + k * this.stride * 8, this.stride);
    }
    this.fields = fields;
    this.io = new Float64Array(buf, ex.ioOffset(), 4);
    ex.setConstants(gamma, meanFlowRate);
  }

  /**
   * A kernel for a duct of `n` cells, or `null` if this build cannot serve it.
   *
   * `null` rather than a throw, because the caller has a working TypeScript path and a duct
   * longer than the compiled capacity is a performance problem, not a correctness one.
   */
  static create(n: number, gamma: number, meanFlowRate: number): EulerKernel | null {
    const mod = kernelModule();
    if (mod === null) return null;
    let ex: KernelExports;
    try {
      ex = new WebAssembly.Instance(mod, {}).exports as unknown as KernelExports;
    } catch {
      return null;
    }
    // `n + 2`: the flux arrays carry face `n`, and the vector loops may touch one slot past
    // the last cell when the tail is even.
    if (n + 2 > ex.capacity()) return null;
    return new EulerKernel(ex, gamma, meanFlowRate);
  }

  /** Primitives, slopes, half-step and interior faces. Returns the largest `|u| + c`. */
  reconstruct(n: number, dt: number, limiter: number): number {
    this.io[IO_DT] = dt;
    this.ex.reconstructIo(n, limiter);
    return this.io[IO_MAX_SPEED]!;
  }

  /** Conservative update, area source and friction, over every cell. */
  update(n: number, dt: number, kLin: number, darcy: number): void {
    const io = this.io;
    io[IO_DT] = dt;
    io[IO_KLIN] = kLin;
    io[IO_DARCY] = darcy;
    this.ex.updateIo(n);
  }
}

interface JunctionExports {
  memory: WebAssembly.Memory;
  junctionBase(): number;
  junctionCapacity(): number;
  junctionFields(): number;
  setJunctionConstants(
    gamma: number,
    invGamma: number,
    invGm1: number,
    pAmb: number,
    r: number,
    tAmb: number,
    cp: number,
    minRho: number,
    minP: number,
    maxSpeed: number,
    ambientC: number,
    tol: number,
  ): void;
  solveJunction(nEnds: number, nOut: number): number;
}

/**
 * Per-branch slots in the junction block. **Must match the `J_*` indices in `kernel/euler.ts`.**
 * The first eight are written by the caller; the rest are the kernel's answer.
 */
export const JUNCTION_FIELDS = [
  'rho',
  'u',
  'p',
  'toward',
  'rhoC',
  'area',
  'c',
  'faceArea',
  'f0',
  'f1',
  'f2',
  'fp',
  'clamps',
  'supersonic',
] as const;

export type JunctionField = (typeof JUNCTION_FIELDS)[number];

/** Constants the junction solve needs, set once. */
export interface JunctionConstants {
  gamma: number;
  pAmb: number;
  r: number;
  tAmb: number;
  cp: number;
  minRho: number;
  minP: number;
  maxSpeed: number;
  ambientC: number;
  tol: number;
}

/**
 * The junction solve in wasm: one instance serving every junction in an exhaust.
 *
 * Its block is separate from the cell fields, so this instance's cell block simply goes unused. The
 * caller writes each branch's end state into `fields`, calls `solve`, and reads each branch's fluxes
 * and clamp counts back out, all through typed arrays: the only floating-point value to cross a call is
 * the imbalance `solve` returns, once per junction.
 */
export class JunctionKernel {
  private readonly ex: JunctionExports;
  readonly capacity: number;
  readonly fields: Record<JunctionField, Float64Array>;

  private constructor(ex: JunctionExports, k: JunctionConstants) {
    this.ex = ex;
    this.capacity = ex.junctionCapacity();
    if (ex.junctionFields() !== JUNCTION_FIELDS.length) throw new Error('junction field count mismatch');
    const base = ex.junctionBase();
    const buf = ex.memory.buffer;
    const fields = {} as Record<JunctionField, Float64Array>;
    for (let f = 0; f < JUNCTION_FIELDS.length; f++) {
      fields[JUNCTION_FIELDS[f]!] = new Float64Array(buf, base + f * this.capacity * 8, this.capacity);
    }
    this.fields = fields;
    ex.setJunctionConstants(
      k.gamma,
      1 / k.gamma,
      1 / (k.gamma - 1),
      k.pAmb,
      k.r,
      k.tAmb,
      k.cp,
      k.minRho,
      k.minP,
      k.maxSpeed,
      k.ambientC,
      k.tol,
    );
  }

  /** A junction kernel, or `null` if this host cannot run one — the caller solves in TypeScript. */
  static create(k: JunctionConstants): JunctionKernel | null {
    const mod = kernelModule();
    if (mod === null) return null;
    try {
      const ex = new WebAssembly.Instance(mod, {}).exports as unknown as JunctionExports;
      if (typeof ex.solveJunction !== 'function') return null;
      return new JunctionKernel(ex, k);
    } catch {
      return null;
    }
  }

  /** Solve the junction just written. Returns the imbalance fraction, or -1 if nothing flowed. */
  solve(nEnds: number, nOut: number): number {
    return this.ex.solveJunction(nEnds, nOut);
  }
}
