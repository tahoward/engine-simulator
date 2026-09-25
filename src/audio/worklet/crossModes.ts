/**
 * Cross-wise acoustic modes of a chamber: the resonances across the can rather than along it.
 *
 * The duct solver is one-dimensional, so it carries only the plane wave, whose pressure is uniform
 * across the section. A wide can also resonates across its width. A flat oval 300 mm wide has its first
 * such mode near `c / 2W`, about 900 Hz in hot gas, far below where a round can of the same area has
 * one. That mode is what makes oval and offset-pipe mufflers sound different from round ones.
 *
 * The model is the standard modal expansion of a cavity driven by volume flows. Inside the body the
 * pressure is
 *
 *     p = p_plane + sum_N a_N(t) Phi_N(y, z, x)
 *
 * where `p_plane` is the duct solver's and each `Phi_N = psi_n(y, z) eps_m cos(m pi x / L)` is a
 * cross-wise mode `psi_n` of the section times a standing wave along the body. Each amplitude obeys
 *
 *     a'' + 2 zeta w a' + w^2 a = (rho c^2 / V) sum_j Phi_N(r_j) Q_j'
 *
 * with `w = c sqrt(k_n^2 + (m pi / L)^2)` and `Q_j` the volume flow entering through pipe `j`. The
 * pipes feel the modes back as extra pressure at their openings, `sum_N a_N Phi_N(r_j)`, applied to
 * the gas in the pipe cell next to each end plate. The power a mode draws from a pipe is exactly the
 * work that pressure does on the pipe's gas, so the exchange conserves energy.
 *
 * `Phi_N(r_j)` is the mode averaged over the pipe's opening. A pipe on the centreline of a symmetric
 * section sees none of the antisymmetric modes, so it is the offset that decides which modes a
 * muffler rings with. Only modes the pipe grid can resolve are kept (see `modeCutoffK`). A round can
 * with centred pipes has none, and is left to the plane-wave solver alone.
 *
 * The section modes come from a Rayleigh-Ritz solution of the Neumann Helmholtz problem, so one code
 * path serves every shape: exact for a rectangle, and within a fraction of a percent of the Bessel
 * roots for a circle (see test/crossModes.test.ts).
 */

import { GAS, type Section, insideSection, sectionArea } from '../../model/spec.js';

const GAMMA = GAS.gammaExh;

/**
 * Modal damping ratio: wall and visco-thermal loss in a steel can. Radiation into the pipes is not in
 * it; that loss comes out of the coupling itself. Measured muffler cavity modes have Q of 20 to 50.
 */
const MODE_ZETA = 0.02;

/** Most modes kept per chamber, so a very large can cannot load the audio thread. */
const MAX_MODES = 24;

/** Substeps between refreshes of the chamber's sound speed and pressure. */
const REFRESH_INTERVAL = 16;

/**
 * Largest mode wavenumber worth keeping, rad/m, for a grid of cell `dx`.
 *
 * The same five-cells-per-wavelength limit as `resolutionCutoffRad`. It is a wavenumber rather than a
 * frequency because `w = c k` for every mode, so the grid resolves the modes a pipe cell can carry
 * whatever the gas temperature is.
 */
export function modeCutoffK(dx: number): number {
  return (2 * Math.PI) / (5 * dx);
}

export interface SectionMode {
  /** Cross-wise wavenumber, rad/m. */
  k: number;
  /** The mode, normalised to mean square 1 over the section, averaged over each pipe's opening. */
  atPipes: number[];
}

export interface PipeOpening {
  /** Offset of the pipe's centre from the section's, m, along the width. */
  offset: number;
  diameter: number;
}

const modeCache = new Map<string, SectionMode[]>();

/**
 * Cross-wise modes of a section with wavenumber below `kMax`, excluding the plane wave.
 *
 * Only modes symmetric about the width axis: the pipes sit on it, so the others cannot be driven.
 * Cached, since a drag rebuilds the pipe many times without changing the chamber.
 */
export function sectionModes(s: Section, kMax: number, pipes: PipeOpening[]): SectionMode[] {
  const key = `${s.section}:${s.width}:${s.height}:${kMax}:${pipes.map((p) => `${p.offset},${p.diameter}`).join(';')}`;
  const hit = modeCache.get(key);
  if (hit) return hit;
  const modes = solveSectionModes(s, kMax, pipes);
  if (modeCache.size > 64) modeCache.clear();
  modeCache.set(key, modes);
  return modes;
}

function solveSectionModes(s: Section, kMax: number, pipes: PipeOpening[]): SectionMode[] {
  const W = s.width;
  const H = s.height;
  // Enough cosines to resolve every mode below kMax with a few to spare for the curved wall.
  const P = Math.min(Math.ceil((kMax * W) / Math.PI) + 4, 16);
  const Q = Math.min(Math.ceil((kMax * H) / (2 * Math.PI)) + 2, 6); // even orders 0, 2, ..
  const nb = P * Q;
  const ky = (p: number) => (p * Math.PI) / W;
  const kz = (q: number) => (2 * q * Math.PI) / H;

  // Midpoint quadrature over the half-section z >= 0, every basis function being even in z. Each
  // cell is weighted by the fraction of it inside the wall, so a curved wall is integrated smoothly.
  const NY = 64;
  const NZ = 32;
  const hy = W / NY;
  const hz = H / 2 / NZ;
  const SUB = 4;
  const M = new Float64Array(nb * nb);
  const K = new Float64Array(nb * nb);
  const phi = new Float64Array(nb);
  const gy = new Float64Array(nb);
  const gz = new Float64Array(nb);
  const cosY = new Float64Array(P);
  const sinY = new Float64Array(P);
  const cosZ = new Float64Array(Q);
  const sinZ = new Float64Array(Q);
  for (let iy = 0; iy < NY; iy++) {
    const y = -W / 2 + (iy + 0.5) * hy;
    for (let iz = 0; iz < NZ; iz++) {
      const z = (iz + 0.5) * hz;
      let inside = 0;
      for (let a = 0; a < SUB; a++) {
        for (let b = 0; b < SUB; b++) {
          const sy = y + ((a + 0.5) / SUB - 0.5) * hy;
          const sz = z + ((b + 0.5) / SUB - 0.5) * hz;
          if (insideSection(s, sy, sz)) inside++;
        }
      }
      if (inside === 0) continue;
      // Doubled for the mirror half.
      const w = (2 * hy * hz * inside) / (SUB * SUB);
      for (let p = 0; p < P; p++) {
        const t = ky(p) * (y + W / 2);
        cosY[p] = Math.cos(t);
        sinY[p] = Math.sin(t);
      }
      for (let q = 0; q < Q; q++) {
        const t = kz(q) * (z + H / 2);
        cosZ[q] = Math.cos(t);
        sinZ[q] = Math.sin(t);
      }
      for (let p = 0; p < P; p++) {
        for (let q = 0; q < Q; q++) {
          const i = p * Q + q;
          phi[i] = cosY[p]! * cosZ[q]!;
          gy[i] = -ky(p) * sinY[p]! * cosZ[q]!;
          gz[i] = -kz(q) * cosY[p]! * sinZ[q]!;
        }
      }
      for (let i = 0; i < nb; i++) {
        const pi = phi[i]! * w;
        const gyi = gy[i]! * w;
        const gzi = gz[i]! * w;
        for (let j = i; j < nb; j++) {
          M[i * nb + j] += pi * phi[j]!;
          K[i * nb + j] += gyi * gy[j]! + gzi * gz[j]!;
        }
      }
    }
  }
  for (let i = 0; i < nb; i++) {
    for (let j = 0; j < i; j++) {
      M[i * nb + j] = M[j * nb + i]!;
      K[i * nb + j] = K[j * nb + i]!;
    }
  }

  const { values, vectors } = generalisedEigen(K, M, nb);
  const area = sectionArea(s);
  const floor = (0.2 * Math.PI) / Math.max(W, H);

  // Sample points covering each pipe's opening in equal areas, for the opening average.
  const samples = pipes.map((pipe) => {
    const pts: Array<[number, number]> = [];
    const R = pipe.diameter / 2;
    const NR = 4;
    const NT = 8;
    for (let i = 0; i < NR; i++) {
      const r = R * Math.sqrt((i + 0.5) / NR);
      for (let j = 0; j < NT; j++) {
        const t = ((j + 0.5 * (i % 2)) / NT) * 2 * Math.PI;
        pts.push([pipe.offset + r * Math.cos(t), r * Math.sin(t)]);
      }
    }
    return pts;
  });

  const out: SectionMode[] = [];
  for (let m = 0; m < nb; m++) {
    const lambda = values[m]!;
    if (!(lambda > floor * floor)) continue;
    const k = Math.sqrt(lambda);
    if (k >= kMax) continue;
    // Scale to mean square 1: v' M v is the integral of psi^2 over the section.
    let norm = 0;
    for (let i = 0; i < nb; i++) {
      let mv = 0;
      for (let j = 0; j < nb; j++) mv += M[i * nb + j]! * vectors[j * nb + m]!;
      norm += vectors[i * nb + m]! * mv;
    }
    const scale = Math.sqrt(area / norm);
    const atPipes = samples.map((pts) => {
      let acc = 0;
      for (const [y, z] of pts) {
        for (let p = 0; p < P; p++) {
          const cy = Math.cos(ky(p) * (y + W / 2));
          for (let q = 0; q < Q; q++) {
            acc += vectors[(p * Q + q) * nb + m]! * cy * Math.cos(kz(q) * (z + H / 2));
          }
        }
      }
      return (acc / pts.length) * scale;
    });
    out.push({ k, atPipes });
  }
  out.sort((a, b) => a.k - b.k);
  return out;
}

/**
 * Solve `K v = lambda M v` for symmetric `K` and positive-definite `M`, both `n x n` row-major.
 * Cholesky reduces it to a standard problem, which cyclic Jacobi solves. Eigenvectors are columns.
 */
export function generalisedEigen(
  K: Float64Array,
  M: Float64Array,
  n: number,
): { values: Float64Array; vectors: Float64Array } {
  // M = L L'.
  const L = new Float64Array(n * n);
  for (let i = 0; i < n; i++) {
    for (let j = 0; j <= i; j++) {
      let s = M[i * n + j]!;
      for (let k = 0; k < j; k++) s -= L[i * n + k]! * L[j * n + k]!;
      if (i === j) L[i * n + i] = Math.sqrt(Math.max(s, 1e-300));
      else L[i * n + j] = s / L[j * n + j]!;
    }
  }
  // C = inv(L) K inv(L)', by forward substitution on the rows and then on the columns.
  const T = new Float64Array(n * n);
  for (let c = 0; c < n; c++) {
    for (let i = 0; i < n; i++) {
      let s = K[i * n + c]!;
      for (let k = 0; k < i; k++) s -= L[i * n + k]! * T[k * n + c]!;
      T[i * n + c] = s / L[i * n + i]!;
    }
  }
  const C = new Float64Array(n * n);
  for (let r = 0; r < n; r++) {
    for (let i = 0; i < n; i++) {
      let s = T[r * n + i]!;
      for (let k = 0; k < i; k++) s -= L[i * n + k]! * C[r * n + k]!;
      C[r * n + i] = s / L[i * n + i]!;
    }
  }

  // Cyclic Jacobi.
  const V = new Float64Array(n * n);
  for (let i = 0; i < n; i++) V[i * n + i] = 1;
  for (let sweep = 0; sweep < 60; sweep++) {
    let off = 0;
    let diag = 0;
    for (let i = 0; i < n; i++) {
      diag += C[i * n + i]! * C[i * n + i]!;
      for (let j = i + 1; j < n; j++) off += C[i * n + j]! * C[i * n + j]!;
    }
    if (off <= 1e-24 * diag) break;
    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        const apq = C[p * n + q]!;
        if (Math.abs(apq) < 1e-300) continue;
        const theta = (C[q * n + q]! - C[p * n + p]!) / (2 * apq);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < n; k++) {
          const ckp = C[k * n + p]!;
          const ckq = C[k * n + q]!;
          C[k * n + p] = c * ckp - s * ckq;
          C[k * n + q] = s * ckp + c * ckq;
        }
        for (let k = 0; k < n; k++) {
          const cpk = C[p * n + k]!;
          const cqk = C[q * n + k]!;
          C[p * n + k] = c * cpk - s * cqk;
          C[q * n + k] = s * cpk + c * cqk;
        }
        for (let k = 0; k < n; k++) {
          const vkp = V[k * n + p]!;
          const vkq = V[k * n + q]!;
          V[k * n + p] = c * vkp - s * vkq;
          V[k * n + q] = s * vkp + c * vkq;
        }
      }
    }
  }

  // Back to the original problem: v = inv(L') y.
  const values = new Float64Array(n);
  const vectors = new Float64Array(n * n);
  for (let m = 0; m < n; m++) {
    values[m] = C[m * n + m]!;
    for (let i = n - 1; i >= 0; i--) {
      let s = V[i * n + m]!;
      for (let k = i + 1; k < n; k++) s -= L[k * n + i]! * vectors[k * n + m]!;
      vectors[i * n + m] = s / L[i * n + i]!;
    }
  }
  return { values, vectors };
}

/** One chamber as the duct discretised it. */
export interface ChamberPlacement {
  section: Section;
  /** Positions of the inlet and outlet end plates along the duct, m. */
  xIn: number;
  xOut: number;
  inlet: PipeOpening;
  outlet: PipeOpening;
}

/**
 * The cross-wise modes of every chamber in one duct, stepped alongside it.
 *
 * Holds the duct's own state arrays and writes into them, like the valve source does. Flat typed
 * arrays throughout, and every float in or out goes through a field: this runs every substep on the
 * audio thread.
 */
export class CrossModes {
  readonly count: number;
  /** Substep, set by the duct before `step`, so no float crosses the call. */
  dt = 0;

  private readonly chambers: number;
  private readonly cellIn: Int32Array;
  private readonly cellOut: Int32Array;
  /** First cell of each body and one past its last, for the sound speed and pressure there. */
  private readonly bodyFrom: Int32Array;
  private readonly bodyTo: Int32Array;
  private readonly areaIn: Float64Array;
  private readonly areaOut: Float64Array;
  private readonly volume: Float64Array;
  private readonly modeFrom: Int32Array;
  private readonly modeTo: Int32Array;
  /** Per chamber: rho c^2 / V, refreshed from the gas. */
  private readonly gain: Float64Array;

  /** Per mode: total wavenumber, the mode at each pipe, and its state. */
  private readonly kTotal: Float64Array;
  private readonly phiIn: Float64Array;
  private readonly phiOut: Float64Array;
  private readonly omega: Float64Array;
  private readonly amp: Float64Array;
  /** `a' - G S`, so the drive enters as `S` rather than its derivative. */
  private readonly aux: Float64Array;

  private refreshCounter = 0;

  constructor(
    placements: ChamberPlacement[],
    private readonly n: number,
    dx: number,
    private readonly rho: Float64Array,
    private readonly mom: Float64Array,
    private readonly en: Float64Array,
    private readonly areaCell: Float64Array,
    private readonly invVol: Float64Array,
  ) {
    const kMax = modeCutoffK(dx);
    const cellIn: number[] = [];
    const cellOut: number[] = [];
    const bodyFrom: number[] = [];
    const bodyTo: number[] = [];
    const areaIn: number[] = [];
    const areaOut: number[] = [];
    const volume: number[] = [];
    const modeFrom: number[] = [];
    const modeTo: number[] = [];
    const kTotal: number[] = [];
    const phiIn: number[] = [];
    const phiOut: number[] = [];

    const cellOf = (x: number) => Math.min(Math.max(Math.floor(x / dx), 0), n - 1);
    for (const ch of placements) {
      const bodyLength = ch.xOut - ch.xIn;
      // The pipe cells on either side of the end plates.
      const iIn = cellOf(ch.xIn - 0.5 * dx);
      const iOut = cellOf(ch.xOut + 0.5 * dx);
      if (iOut - iIn < 2 || bodyLength <= 0) continue;
      const modes = sectionModes(ch.section, kMax, [ch.inlet, ch.outlet]);
      const start = kTotal.length;
      for (const mode of modes) {
        for (let m = 0; kTotal.length - start < MAX_MODES; m++) {
          const kl = (m * Math.PI) / bodyLength;
          const k = Math.hypot(mode.k, kl);
          if (k >= kMax) break;
          const eps = m === 0 ? 1 : Math.SQRT2;
          const pIn = eps * mode.atPipes[0]!;
          const pOut = eps * (m % 2 === 0 ? 1 : -1) * mode.atPipes[1]!;
          // A mode neither pipe can drive does nothing but cost time.
          if (Math.abs(pIn) < 1e-3 && Math.abs(pOut) < 1e-3) continue;
          kTotal.push(k);
          phiIn.push(pIn);
          phiOut.push(pOut);
        }
      }
      if (kTotal.length === start) continue;
      cellIn.push(iIn);
      cellOut.push(iOut);
      bodyFrom.push(cellOf(ch.xIn) + 1);
      bodyTo.push(Math.max(cellOf(ch.xOut), cellOf(ch.xIn) + 2));
      areaIn.push((Math.PI * ch.inlet.diameter * ch.inlet.diameter) / 4);
      areaOut.push((Math.PI * ch.outlet.diameter * ch.outlet.diameter) / 4);
      volume.push(sectionArea(ch.section) * bodyLength);
      modeFrom.push(start);
      modeTo.push(kTotal.length);
    }

    this.chambers = cellIn.length;
    this.count = kTotal.length;
    this.cellIn = Int32Array.from(cellIn);
    this.cellOut = Int32Array.from(cellOut);
    this.bodyFrom = Int32Array.from(bodyFrom);
    this.bodyTo = Int32Array.from(bodyTo);
    this.areaIn = Float64Array.from(areaIn);
    this.areaOut = Float64Array.from(areaOut);
    this.volume = Float64Array.from(volume);
    this.modeFrom = Int32Array.from(modeFrom);
    this.modeTo = Int32Array.from(modeTo);
    this.gain = new Float64Array(this.chambers);
    this.kTotal = Float64Array.from(kTotal);
    this.phiIn = Float64Array.from(phiIn);
    this.phiOut = Float64Array.from(phiOut);
    this.omega = new Float64Array(this.count);
    this.amp = new Float64Array(this.count);
    this.aux = new Float64Array(this.count);
  }

  /** Follow the gas in each body: the modes sit at `c k`, and the drive scales with `rho c^2 = gamma p`. */
  refresh(): void {
    const { rho, mom, en } = this;
    for (let ch = 0; ch < this.chambers; ch++) {
      let c2 = 0;
      let p = 0;
      const from = this.bodyFrom[ch]!;
      const to = Math.min(this.bodyTo[ch]!, this.n);
      for (let i = from; i < to; i++) {
        const r = rho[i]!;
        const u = mom[i]! / r;
        const pi = Math.max((GAMMA - 1) * (en[i]! - 0.5 * r * u * u), 1e-3);
        p += pi;
        c2 += (GAMMA * pi) / r;
      }
      const cells = Math.max(to - from, 1);
      const c = Math.sqrt(c2 / cells);
      this.gain[ch] = (GAMMA * (p / cells)) / this.volume[ch]!;
      for (let k = this.modeFrom[ch]!; k < this.modeTo[ch]!; k++) this.omega[k] = c * this.kTotal[k]!;
    }
  }

  /** Advance every mode by `this.dt` and push its pressure back onto the pipes. */
  step(): void {
    if (--this.refreshCounter <= 0) {
      this.refresh();
      this.refreshCounter = REFRESH_INTERVAL;
    }
    const dt = this.dt;
    const { rho, mom, en, areaCell, invVol, amp, aux, omega, phiIn, phiOut } = this;
    for (let ch = 0; ch < this.chambers; ch++) {
      const iIn = this.cellIn[ch]!;
      const iOut = this.cellOut[ch]!;
      const uIn = mom[iIn]! / rho[iIn]!;
      const uOut = mom[iOut]! / rho[iOut]!;
      // Volume flows in +x: into the body through the inlet, out of it through the outlet.
      const qIn = uIn * areaCell[iIn]!;
      const qOut = uOut * areaCell[iOut]!;
      const g = this.gain[ch]!;
      let pIn = 0;
      let pOut = 0;
      for (let k = this.modeFrom[ch]!; k < this.modeTo[ch]!; k++) {
        const w = omega[k]!;
        const drive = g * (phiIn[k]! * qIn - phiOut[k]! * qOut);
        // Symplectic Euler on (a, a' - G S): stable for w dt < 2, and w dt stays under 0.5 here.
        const a = amp[k]!;
        const b = aux[k]! - dt * (w * w * a + 2 * MODE_ZETA * w * (aux[k]! + drive));
        const na = a + dt * (b + drive);
        aux[k] = b;
        amp[k] = na;
        pIn += na * phiIn[k]!;
        pOut += na * phiOut[k]!;
      }
      // The pressure at each opening pushes on the pipe's gas: back up the inlet, on down the outlet.
      const dmIn = -dt * pIn * this.areaIn[ch]! * invVol[iIn]!;
      mom[iIn] = mom[iIn]! + dmIn;
      en[iIn] = en[iIn]! + dmIn * uIn;
      const dmOut = dt * pOut * this.areaOut[ch]! * invVol[iOut]!;
      mom[iOut] = mom[iOut]! + dmOut;
      en[iOut] = en[iOut]! + dmOut * uOut;
    }
  }

  reset(): void {
    this.amp.fill(0);
    this.aux.fill(0);
  }

  /**
   * Acoustic energy held in the modes, J. With `Phi` of mean square 1 over the body, the potential
   * energy `V a^2 / (2 rho c^2)` is `a^2 / 2G`, and the kinetic energy has `a'/w` in place of `a`.
   * Taken between steps, where the drive is small beside `a'`.
   */
  energy(): number {
    let e = 0;
    for (let ch = 0; ch < this.chambers; ch++) {
      const g = this.gain[ch]!;
      if (!(g > 0)) continue;
      for (let k = this.modeFrom[ch]!; k < this.modeTo[ch]!; k++) {
        const w = this.omega[k]!;
        const a = this.amp[k]!;
        const v = this.aux[k]!;
        e += (a * a + (w > 0 ? (v * v) / (w * w) : 0)) / (2 * g);
      }
    }
    return e;
  }

  /** Frequency of every mode kept, at the current gas state, Hz. */
  frequencies(): number[] {
    return Array.from(this.omega, (w) => w / (2 * Math.PI));
  }
}
