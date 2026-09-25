/** Minimal spectral analysis for the physics tests. Not used by the app. */

/** In-place iterative radix-2 FFT. `re`/`im` length must be a power of two. */
export function fft(re: Float64Array, im: Float64Array): void {
  const n = re.length;
  if (n !== im.length || (n & (n - 1)) !== 0) {
    throw new Error('fft: length must match and be a power of two');
  }

  // Bit-reversal permutation.
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      [re[i], re[j]] = [re[j]!, re[i]!];
      [im[i], im[j]] = [im[j]!, im[i]!];
    }
  }

  for (let len = 2; len <= n; len <<= 1) {
    const ang = (-2 * Math.PI) / len;
    const wRe = Math.cos(ang);
    const wIm = Math.sin(ang);
    for (let i = 0; i < n; i += len) {
      let curRe = 1;
      let curIm = 0;
      for (let k = 0; k < len / 2; k++) {
        const uRe = re[i + k]!;
        const uIm = im[i + k]!;
        const vRe = re[i + k + len / 2]! * curRe - im[i + k + len / 2]! * curIm;
        const vIm = re[i + k + len / 2]! * curIm + im[i + k + len / 2]! * curRe;
        re[i + k] = uRe + vRe;
        im[i + k] = uIm + vIm;
        re[i + k + len / 2] = uRe - vRe;
        im[i + k + len / 2] = uIm - vIm;
        const nextRe = curRe * wRe - curIm * wIm;
        curIm = curRe * wIm + curIm * wRe;
        curRe = nextRe;
      }
    }
  }
}

/**
 * Magnitude spectrum of `signal`, zero-padded or truncated to `size`.
 * Returns bins 0..size/2.
 */
export function magnitudeSpectrum(signal: Float32Array, size: number): Float64Array {
  const re = new Float64Array(size);
  const im = new Float64Array(size);
  const n = Math.min(signal.length, size);
  for (let i = 0; i < n; i++) re[i] = signal[i]!;
  fft(re, im);
  const half = size / 2 + 1;
  const mag = new Float64Array(half);
  for (let i = 0; i < half; i++) mag[i] = Math.hypot(re[i]!, im[i]!);
  return mag;
}

/** Hann window, applied in place. Use when the signal does not decay to zero. */
export function hann(signal: Float32Array): Float32Array {
  const out = new Float32Array(signal.length);
  for (let i = 0; i < signal.length; i++) {
    out[i] = signal[i]! * 0.5 * (1 - Math.cos((2 * Math.PI * i) / (signal.length - 1)));
  }
  return out;
}

export interface Peak {
  hz: number;
  mag: number;
}

/**
 * Local maxima of a magnitude spectrum between `loHz` and `hiHz`, strongest first.
 * Frequencies are parabolically interpolated so a resonance sitting between bins is
 * still located to well under one bin.
 */
export function findPeaks(
  mag: Float64Array,
  sampleRate: number,
  size: number,
  loHz: number,
  hiHz: number,
  relThreshold = 0.05,
): Peak[] {
  const binHz = sampleRate / size;
  const lo = Math.max(1, Math.floor(loHz / binHz));
  const hi = Math.min(mag.length - 2, Math.ceil(hiHz / binHz));

  let maxMag = 0;
  for (let i = lo; i <= hi; i++) if (mag[i]! > maxMag) maxMag = mag[i]!;

  const peaks: Peak[] = [];
  for (let i = lo; i <= hi; i++) {
    const m = mag[i]!;
    if (m < maxMag * relThreshold) continue;
    if (m <= mag[i - 1]! || m < mag[i + 1]!) continue;
    // Parabolic interpolation on the three points around the peak.
    const a = mag[i - 1]!;
    const b = m;
    const c = mag[i + 1]!;
    const denom = a - 2 * b + c;
    const shift = denom === 0 ? 0 : (0.5 * (a - c)) / denom;
    peaks.push({ hz: (i + shift) * binHz, mag: m });
  }

  return peaks.sort((x, y) => y.mag - x.mag);
}

/** Energy in a narrow band around `hz`, as a fraction of total energy in `mag`. */
export function bandEnergy(
  mag: Float64Array,
  sampleRate: number,
  size: number,
  hz: number,
  widthHz: number,
): number {
  const binHz = sampleRate / size;
  const lo = Math.max(0, Math.floor((hz - widthHz / 2) / binHz));
  const hi = Math.min(mag.length - 1, Math.ceil((hz + widthHz / 2) / binHz));
  let e = 0;
  for (let i = lo; i <= hi; i++) e += mag[i]! * mag[i]!;
  return e;
}
