/** Tiny DSP helpers. No allocations in any hot path. */

/** One-pole lowpass, `y += c * (x - y)`. */
export class OnePole {
  private y = 0;
  constructor(public c = 0.5) {}

  process(x: number): number {
    this.y += this.c * (x - this.y);
    return this.y;
  }

  /** Set the cutoff in Hz. */
  setCutoff(hz: number, sampleRate: number): void {
    this.c = 1 - Math.exp((-2 * Math.PI * hz) / sampleRate);
  }

  reset(): void {
    this.y = 0;
  }
}

/**
 * One structural mode: a two-pole band-pass that rings at `hz` with the given decay.
 *
 * Band-pass, not a plain all-pole resonator. An all-pole two-pole is a resonant *low-pass*: flat below
 * its peak, with a DC gain of about `1 / theta` — ten for the 780 Hz block mode. Driven by the combustion
 * pressure rise, whose energy sits at the firing harmonics, such a "casing" would mostly pass those
 * harmonics straight through, and the valve clacks would come out as a buzz at their own repetition
 * rate. Both rates scale with the number of cylinders, so the mechanical noise would rise in pitch as
 * cylinders were added — a signal periodic at 25 Hz on a single and 214 Hz on a V8, correlation 0.9,
 * where it should be the casing's own ring.
 *
 * A vibrating surface radiates in proportion to its acceleration, so a mode radiates nothing at DC. The
 * zeros at DC and at Nyquist, `(1 - z^-2) / 2`, say that. The poles alone set the ring, the decay and the
 * gain at resonance, so the calibrated levels upstream mean the same with the zeros as without.
 */
export class Resonator {
  private y1 = 0;
  private y2 = 0;
  private x1 = 0;
  private x2 = 0;
  private a1 = 0;
  private a2 = 0;

  constructor(hz: number, q: number, sampleRate: number) {
    this.set(hz, q, sampleRate);
  }

  set(hz: number, q: number, sampleRate: number): void {
    const r = Math.exp((-Math.PI * hz) / (q * sampleRate));
    const theta = (2 * Math.PI * hz) / sampleRate;
    this.a1 = 2 * r * Math.cos(theta);
    this.a2 = -r * r;
  }

  /** The frequency it rings at, Hz, read back from its poles. */
  frequency(sampleRate: number): number {
    const r = Math.sqrt(-this.a2);
    return (Math.acos(this.a1 / (2 * r)) * sampleRate) / (2 * Math.PI);
  }

  /** Excite with `x`; the peak ring amplitude is approximately `x`. */
  process(x: number): number {
    /**
     * An impulse of amplitude A rings with a peak of about A.
     *
     * The all-pole impulse response is `r^n sin((n+1) theta) / sin(theta)`, so the band-pass one is that
     * minus itself two samples late — about `2 cos(n theta)` for a lightly damped mode. Halving it puts
     * the peak back at about A, and excitation levels stay in physical units.
     */
    const y = 0.5 * (x - this.x2) + this.a1 * this.y1 + this.a2 * this.y2;
    this.x2 = this.x1;
    this.x1 = x;
    this.y2 = this.y1;
    this.y1 = y;
    return y;
  }

  reset(): void {
    this.y1 = 0;
    this.y2 = 0;
    this.x1 = 0;
    this.x2 = 0;
  }
}

/**
 * A mechanical impact of finite duration.
 *
 * Nothing in an engine hits anything else instantaneously. A valve seats over a contact time
 * of order a tenth of a millisecond, a piston crosses its clearance over rather longer, and
 * that duration is what band-limits the noise they make: a force pulse lasting `t` has
 * nothing much above `1/t` in it.
 *
 * Driving a resonator with a one-sample impulse instead asserts a spectrum flat to Nyquist,
 * and a two-pole resonator only sheds 12 dB/octave, so the excess comes straight out as
 * hiss an octave or two above the mode. On the muffled presets that would be worth 10 to
 * 16 dB in the 8 kHz octave — an audible buzz sitting above the engine.
 *
 * The pulse is a raised cosine, and it is scaled to preserve *area* rather than peak,
 * because for frequencies well below `1/t` a resonator responds to the impulse's integral.
 * So the ring amplitude at the mode is unchanged and only the far skirts lose energy.
 */
export class Impact {
  private readonly window: Float64Array;
  private pos = 0;
  private amp = 0;

  /** @param seconds Contact duration. @param sampleRate Audio rate. */
  constructor(seconds: number, sampleRate: number) {
    const n = Math.max(2, Math.round(seconds * sampleRate));
    this.window = new Float64Array(n);
    let sum = 0;
    for (let i = 0; i < n; i++) {
      this.window[i] = 0.5 * (1 - Math.cos((2 * Math.PI * (i + 0.5)) / n));
      sum += this.window[i]!;
    }
    // Unit area, so `trigger(a)` delivers the same impulse a bare `a` would have.
    for (let i = 0; i < n; i++) this.window[i]! /= sum;
    this.pos = n;
  }

  /** Start a new impact. Retriggering mid-pulse takes the louder of the two. */
  trigger(amp: number): void {
    if (this.pos < this.window.length && amp <= this.amp) return;
    this.amp = amp;
    this.pos = 0;
  }

  /** Next sample of the force pulse, zero when idle. */
  next(): number {
    if (this.pos >= this.window.length) return 0;
    return this.amp * this.window[this.pos++]!;
  }

  reset(): void {
    this.pos = this.window.length;
    this.amp = 0;
  }
}

/**
 * Deterministic white noise. `Math.random()` is fine for audio but a seeded
 * generator makes the physics tests reproducible.
 */
export class Noise {
  private s = 0;
  constructor(seed = 0x2f6e2b1) {
    this.s = seed >>> 0;
  }

  /** Uniform in [-1, 1). */
  next(): number {
    // xorshift32
    let x = this.s;
    x ^= x << 13;
    x ^= x >>> 17;
    x ^= x << 5;
    this.s = x >>> 0;
    return this.s / 0x80000000 - 1;
  }

  /**
   * Normally distributed, mean 0, standard deviation 1. Used for cycle-to-cycle
   * combustion scatter, where the tail shape matters: the occasional weak cycle is
   * what makes an idle sound alive, and a uniform distribution never gives one.
   */
  gaussian(): number {
    // Box-Muller. The 1e-7 floor keeps log() away from zero.
    const u1 = Math.max((this.next() + 1) / 2, 1e-7);
    const u2 = (this.next() + 1) / 2;
    return Math.sqrt(-2 * Math.log(u1)) * Math.cos(2 * Math.PI * u2);
  }
}

/**
 * Fractional-delay line, used for the ground-reflected path to the listener and
 * for each mouth's extra path in `EngineSim`.
 * Linear interpolation is plenty: the path difference changes only when the
 * listener moves, and the artefacts sit far below the reflection itself.
 */
export class Delay {
  private readonly buf: Float32Array;
  private write = 0;
  private delaySamples = 0;

  constructor(maxSamples: number) {
    this.buf = new Float32Array(Math.max(2, Math.ceil(maxSamples)));
  }

  setDelay(samples: number): void {
    this.delaySamples = clamp(samples, 0, this.buf.length - 2);
  }

  process(x: number): number {
    this.buf[this.write] = x;
    const read = this.write - this.delaySamples;
    const i = Math.floor(read);
    const frac = read - i;
    const a = this.buf[((i % this.buf.length) + this.buf.length) % this.buf.length]!;
    const b = this.buf[(((i + 1) % this.buf.length) + this.buf.length) % this.buf.length]!;
    this.write = (this.write + 1) % this.buf.length;
    return a + (b - a) * frac;
  }

  reset(): void {
    this.buf.fill(0);
    this.write = 0;
  }
}

/**
 * Gentle saturation on the master output. The simulation can legitimately produce
 * enormous pressures when someone bolts on a 30 mm pipe at 9000 rpm; clipping
 * harshly there sounds like a bug even though it is the physics being honest.
 */
export function softClip(x: number): number {
  // The cubic `x - x^3 / (3 k^2)` has unit slope at zero and flattens to `2k/3` with zero slope at
  // `x = k`, so a knee at 1.5 lands exactly on full scale and the curve joins the clamp smoothly. A
  // knee at 1.2, where the cubic has only reached 0.8, would make anything louder jump to 1.
  if (x > 1.5) return 1;
  if (x < -1.5) return -1;
  return x - (x * x * x) / 6.75;
}

/**
 * `Math.hypot(a, b)`, bit for bit as V8 computes it: scaled by the larger magnitude, with a compensated sum.
 *
 * For the per-sample path. `Math.hypot` is a builtin call TurboFan does not lower, so each use would box both
 * arguments and the result into fresh heap objects — six junctions' worth every sample. Checked against
 * `Math.hypot` on twenty million random pairs spanning thirteen decades each, with no differences.
 */
export function hypot2(a: number, b: number): number {
  const x = Math.abs(a);
  const y = Math.abs(b);
  if (x === Infinity || y === Infinity) return Infinity;
  const max = x > y ? x : y;
  if (max === 0) return 0;
  if (max !== max) return NaN;
  const nx = x / max;
  const ny = y / max;
  // Kahan summation of the two squares, starting from zero.
  let sum = 0;
  let comp = 0;
  let summand = nx * nx - comp;
  let prelim = sum + summand;
  comp = prelim - sum - summand;
  sum = prelim;
  summand = ny * ny - comp;
  prelim = sum + summand;
  sum = prelim;
  return Math.sqrt(sum) * max;
}

export function clamp(x: number, lo: number, hi: number): number {
  return x < lo ? lo : x > hi ? hi : x;
}

/** Wraps a crank angle into [0, 720). */
export function wrapCycle(deg: number): number {
  let d = deg % 720;
  if (d < 0) d += 720;
  return d;
}

/**
 * How far `deg` is past `ref`, the shorter way round a 720-degree cycle,
 * in (-360, 360]. Used so valve/ignition windows work across the 0/720 seam.
 */
export function cycleDelta(deg: number, ref: number): number {
  let d = (deg - ref) % 720;
  if (d < 0) d += 720;
  if (d > 360) d -= 720;
  return d;
}

/**
 * Position within an event window that may wrap past 720, as a 0..1 fraction.
 * Returns -1 when `deg` is outside the window.
 */
export function windowPhase(deg: number, open: number, close: number): number {
  let span = (close - open) % 720;
  if (span <= 0) span += 720;
  let rel = (deg - open) % 720;
  if (rel < 0) rel += 720;
  return rel <= span ? rel / span : -1;
}
