//! Cross-wise acoustic modes of a chamber: the resonances across the can rather than along it.
//!
//! The duct solver carries only the plane wave. A wide can also resonates across its width, and that
//! mode is what makes oval and offset-pipe mufflers sound unlike round ones. Each mode is a damped
//! oscillator driven by the volume flows through the chamber's pipes, which feel it back as extra
//! pressure at their openings:
//!
//! ```text
//!     a'' + 2 zeta w a' + w^2 a = (rho c^2 / V) sum_j Phi_N(r_j) Q_j'
//! ```
//!
//! The section modes come from a Rayleigh-Ritz solution of the Neumann Helmholtz problem, so one code
//! path serves every shape.

use std::cell::RefCell;
use std::collections::HashMap;

use crate::math::{self, PI, SQRT_2};
use crate::spec::{Section, gas, inside_section, section_area};

const GAMMA: f64 = gas::GAMMA_EXH;

/// Modal damping ratio: wall and visco-thermal loss in a steel can.
const MODE_ZETA: f64 = 0.02;

/// Most modes kept per chamber.
const MAX_MODES: usize = 24;

/// Substeps between refreshes of the chamber's sound speed and pressure.
const REFRESH_INTERVAL: i32 = 16;

/// Largest mode wavenumber worth keeping, rad/m, for a grid of cell `dx`: five cells a wavelength.
pub fn mode_cutoff_k(dx: f64) -> f64 {
    (2.0 * PI) / (5.0 * dx)
}

#[derive(Clone, Debug)]
pub struct SectionMode {
    /// Cross-wise wavenumber, rad/m.
    pub k: f64,
    /// The mode, normalised to mean square 1 over the section, averaged over each pipe's opening.
    pub at_pipes: Vec<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PipeOpening {
    /// Offset of the pipe's centre from the section's, m, along the width.
    pub offset: f64,
    pub diameter: f64,
}

thread_local! {
    static MODE_CACHE: RefCell<HashMap<String, Vec<SectionMode>>> = RefCell::new(HashMap::new());
}

/// Cross-wise modes of a section with wavenumber below `k_max`, excluding the plane wave. Only modes
/// symmetric about the width axis, since the pipes sit on it. Cached.
pub fn section_modes(s: &Section, k_max: f64, pipes: &[PipeOpening]) -> Vec<SectionMode> {
    let key = format!(
        "{:?}:{:?}:{:?}:{:?}:{}",
        s.section,
        s.width,
        s.height,
        k_max,
        pipes.iter().map(|p| format!("{:?},{:?}", p.offset, p.diameter)).collect::<Vec<_>>().join(";")
    );
    if let Some(hit) = MODE_CACHE.with(|c| c.borrow().get(&key).cloned()) {
        return hit;
    }
    let modes = solve_section_modes(s, k_max, pipes);
    MODE_CACHE.with(|c| {
        let mut c = c.borrow_mut();
        if c.len() > 64 {
            c.clear();
        }
        c.insert(key, modes.clone());
    });
    modes
}

fn solve_section_modes(s: &Section, k_max: f64, pipes: &[PipeOpening]) -> Vec<SectionMode> {
    let w_ = s.width;
    let h_ = s.height;
    let p_n = math::min(((k_max * w_) / PI).ceil() + 4.0, 16.0) as usize;
    let q_n = math::min(((k_max * h_) / (2.0 * PI)).ceil() + 2.0, 6.0) as usize;
    let nb = p_n * q_n;
    let ky = |p: usize| (p as f64 * PI) / w_;
    let kz = |q: usize| (2.0 * q as f64 * PI) / h_;

    const NY: usize = 64;
    const NZ: usize = 32;
    let hy = w_ / NY as f64;
    let hz = h_ / 2.0 / NZ as f64;
    const SUB: usize = 4;
    let mut m = vec![0.0; nb * nb];
    let mut k = vec![0.0; nb * nb];
    let mut phi = vec![0.0; nb];
    let mut gy = vec![0.0; nb];
    let mut gz = vec![0.0; nb];
    let mut cos_y = vec![0.0; p_n];
    let mut sin_y = vec![0.0; p_n];
    let mut cos_z = vec![0.0; q_n];
    let mut sin_z = vec![0.0; q_n];
    for iy in 0..NY {
        let y = -w_ / 2.0 + (iy as f64 + 0.5) * hy;
        for iz in 0..NZ {
            let z = (iz as f64 + 0.5) * hz;
            let mut inside = 0usize;
            for a in 0..SUB {
                for b in 0..SUB {
                    let sy = y + ((a as f64 + 0.5) / SUB as f64 - 0.5) * hy;
                    let sz = z + ((b as f64 + 0.5) / SUB as f64 - 0.5) * hz;
                    if inside_section(s, sy, sz) {
                        inside += 1;
                    }
                }
            }
            if inside == 0 {
                continue;
            }
            let w = (2.0 * hy * hz * inside as f64) / (SUB * SUB) as f64;
            for p in 0..p_n {
                let t = ky(p) * (y + w_ / 2.0);
                cos_y[p] = math::cos(t);
                sin_y[p] = math::sin(t);
            }
            for q in 0..q_n {
                let t = kz(q) * (z + h_ / 2.0);
                cos_z[q] = math::cos(t);
                sin_z[q] = math::sin(t);
            }
            for p in 0..p_n {
                for q in 0..q_n {
                    let i = p * q_n + q;
                    phi[i] = cos_y[p] * cos_z[q];
                    gy[i] = -ky(p) * sin_y[p] * cos_z[q];
                    gz[i] = -kz(q) * cos_y[p] * sin_z[q];
                }
            }
            for i in 0..nb {
                let pi = phi[i] * w;
                let gyi = gy[i] * w;
                let gzi = gz[i] * w;
                for j in i..nb {
                    m[i * nb + j] += pi * phi[j];
                    k[i * nb + j] += gyi * gy[j] + gzi * gz[j];
                }
            }
        }
    }
    for i in 0..nb {
        for j in 0..i {
            m[i * nb + j] = m[j * nb + i];
            k[i * nb + j] = k[j * nb + i];
        }
    }

    let (values, vectors) = generalised_eigen(&k, &m, nb);
    let area = section_area(s);
    let floor = (0.2 * PI) / math::max(w_, h_);

    let samples: Vec<Vec<(f64, f64)>> = pipes
        .iter()
        .map(|pipe| {
            let mut pts = Vec::new();
            let r_ = pipe.diameter / 2.0;
            const NR: usize = 4;
            const NT: usize = 8;
            for i in 0..NR {
                let r = r_ * math::sqrt((i as f64 + 0.5) / NR as f64);
                for j in 0..NT {
                    let t = ((j as f64 + 0.5 * (i % 2) as f64) / NT as f64) * 2.0 * PI;
                    pts.push((pipe.offset + r * math::cos(t), r * math::sin(t)));
                }
            }
            pts
        })
        .collect();

    let mut out: Vec<SectionMode> = Vec::new();
    for mi in 0..nb {
        let lambda = values[mi];
        if !(lambda > floor * floor) {
            continue;
        }
        let kk = math::sqrt(lambda);
        if kk >= k_max {
            continue;
        }
        let mut norm = 0.0;
        for i in 0..nb {
            let mut mv = 0.0;
            for j in 0..nb {
                mv += m[i * nb + j] * vectors[j * nb + mi];
            }
            norm += vectors[i * nb + mi] * mv;
        }
        let scale = math::sqrt(area / norm);
        let at_pipes = samples
            .iter()
            .map(|pts| {
                let mut acc = 0.0;
                for &(y, z) in pts {
                    for p in 0..p_n {
                        let cy = math::cos(ky(p) * (y + w_ / 2.0));
                        for q in 0..q_n {
                            acc += vectors[(p * q_n + q) * nb + mi] * cy * math::cos(kz(q) * (z + h_ / 2.0));
                        }
                    }
                }
                (acc / pts.len() as f64) * scale
            })
            .collect();
        out.push(SectionMode { k: kk, at_pipes });
    }
    out.sort_by(|a, b| a.k.partial_cmp(&b.k).unwrap_or(std::cmp::Ordering::Equal));
    out
}

/// Solve `K v = lambda M v` for symmetric `K` and positive-definite `M`, both `n x n` row-major:
/// Cholesky, then cyclic Jacobi. Eigenvectors are columns.
pub fn generalised_eigen(k_mat: &[f64], m_mat: &[f64], n: usize) -> (Vec<f64>, Vec<f64>) {
    let mut l = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut s = m_mat[i * n + j];
            for kk in 0..j {
                s -= l[i * n + kk] * l[j * n + kk];
            }
            if i == j {
                l[i * n + i] = math::sqrt(math::max(s, 1e-300));
            } else {
                l[i * n + j] = s / l[j * n + j];
            }
        }
    }
    let mut t = vec![0.0; n * n];
    for c in 0..n {
        for i in 0..n {
            let mut s = k_mat[i * n + c];
            for kk in 0..i {
                s -= l[i * n + kk] * t[kk * n + c];
            }
            t[i * n + c] = s / l[i * n + i];
        }
    }
    let mut cm = vec![0.0; n * n];
    for r in 0..n {
        for i in 0..n {
            let mut s = t[r * n + i];
            for kk in 0..i {
                s -= l[i * n + kk] * cm[r * n + kk];
            }
            cm[r * n + i] = s / l[i * n + i];
        }
    }

    let mut v = vec![0.0; n * n];
    for i in 0..n {
        v[i * n + i] = 1.0;
    }
    for _sweep in 0..60 {
        let mut off = 0.0;
        let mut diag = 0.0;
        for i in 0..n {
            diag += cm[i * n + i] * cm[i * n + i];
            for j in i + 1..n {
                off += cm[i * n + j] * cm[i * n + j];
            }
        }
        if off <= 1e-24 * diag {
            break;
        }
        for p in 0..n {
            for q in p + 1..n {
                let apq = cm[p * n + q];
                if apq.abs() < 1e-300 {
                    continue;
                }
                let theta = (cm[q * n + q] - cm[p * n + p]) / (2.0 * apq);
                let sgn = math::sign(if theta == 0.0 || theta.is_nan() { 1.0 } else { theta });
                let tt = sgn / (theta.abs() + math::sqrt(theta * theta + 1.0));
                let c = 1.0 / math::sqrt(tt * tt + 1.0);
                let s = tt * c;
                for kk in 0..n {
                    let ckp = cm[kk * n + p];
                    let ckq = cm[kk * n + q];
                    cm[kk * n + p] = c * ckp - s * ckq;
                    cm[kk * n + q] = s * ckp + c * ckq;
                }
                for kk in 0..n {
                    let cpk = cm[p * n + kk];
                    let cqk = cm[q * n + kk];
                    cm[p * n + kk] = c * cpk - s * cqk;
                    cm[q * n + kk] = s * cpk + c * cqk;
                }
                for kk in 0..n {
                    let vkp = v[kk * n + p];
                    let vkq = v[kk * n + q];
                    v[kk * n + p] = c * vkp - s * vkq;
                    v[kk * n + q] = s * vkp + c * vkq;
                }
            }
        }
    }

    let mut values = vec![0.0; n];
    let mut vectors = vec![0.0; n * n];
    for mi in 0..n {
        values[mi] = cm[mi * n + mi];
        for i in (0..n).rev() {
            let mut s = v[i * n + mi];
            for kk in i + 1..n {
                s -= l[kk * n + i] * vectors[kk * n + mi];
            }
            vectors[i * n + mi] = s / l[i * n + i];
        }
    }
    (values, vectors)
}

/// One chamber as the duct discretised it.
#[derive(Clone, Debug)]
pub struct ChamberPlacement {
    pub section: Section,
    /// Positions of the inlet and outlet end plates along the duct, m.
    pub x_in: f64,
    pub x_out: f64,
    pub inlet: PipeOpening,
    pub outlet: PipeOpening,
}

/// The cross-wise modes of every chamber in one duct, stepped alongside it on the duct's own state.
pub struct CrossModes {
    pub count: usize,
    n: usize,
    cell_in: Vec<usize>,
    cell_out: Vec<usize>,
    body_from: Vec<usize>,
    body_to: Vec<usize>,
    area_in: Vec<f64>,
    area_out: Vec<f64>,
    volume: Vec<f64>,
    mode_from: Vec<usize>,
    mode_to: Vec<usize>,
    /// Per chamber: rho c^2 / V, refreshed from the gas.
    gain: Vec<f64>,
    k_total: Vec<f64>,
    phi_in: Vec<f64>,
    phi_out: Vec<f64>,
    omega: Vec<f64>,
    amp: Vec<f64>,
    /// `a' - G S`, so the drive enters as `S` rather than its derivative.
    aux: Vec<f64>,
    refresh_counter: i32,
}

impl CrossModes {
    pub fn new(placements: &[ChamberPlacement], n: usize, dx: f64) -> CrossModes {
        let k_max = mode_cutoff_k(dx);
        let mut cm = CrossModes {
            count: 0,
            n,
            cell_in: Vec::new(),
            cell_out: Vec::new(),
            body_from: Vec::new(),
            body_to: Vec::new(),
            area_in: Vec::new(),
            area_out: Vec::new(),
            volume: Vec::new(),
            mode_from: Vec::new(),
            mode_to: Vec::new(),
            gain: Vec::new(),
            k_total: Vec::new(),
            phi_in: Vec::new(),
            phi_out: Vec::new(),
            omega: Vec::new(),
            amp: Vec::new(),
            aux: Vec::new(),
            refresh_counter: 0,
        };
        let cell_of = |x: f64| -> usize {
            let c = math::min(math::max((x / dx).floor(), 0.0), n as f64 - 1.0);
            c as usize
        };
        for ch in placements {
            let body_length = ch.x_out - ch.x_in;
            let i_in = cell_of(ch.x_in - 0.5 * dx);
            let i_out = cell_of(ch.x_out + 0.5 * dx);
            if (i_out as i64 - i_in as i64) < 2 || body_length <= 0.0 {
                continue;
            }
            let modes = section_modes(&ch.section, k_max, &[ch.inlet, ch.outlet]);
            let start = cm.k_total.len();
            for mode in &modes {
                let mut mi = 0usize;
                while cm.k_total.len() - start < MAX_MODES {
                    let kl = (mi as f64 * PI) / body_length;
                    let k = math::hypot(&[mode.k, kl]);
                    if k >= k_max {
                        break;
                    }
                    let eps = if mi == 0 { 1.0 } else { SQRT_2 };
                    let p_in = eps * mode.at_pipes[0];
                    let p_out = eps * (if mi.is_multiple_of(2) { 1.0 } else { -1.0 }) * mode.at_pipes[1];
                    mi += 1;
                    if p_in.abs() < 1e-3 && p_out.abs() < 1e-3 {
                        continue;
                    }
                    cm.k_total.push(k);
                    cm.phi_in.push(p_in);
                    cm.phi_out.push(p_out);
                }
            }
            if cm.k_total.len() == start {
                continue;
            }
            cm.cell_in.push(i_in);
            cm.cell_out.push(i_out);
            cm.body_from.push(cell_of(ch.x_in) + 1);
            cm.body_to.push(cell_of(ch.x_out).max(cell_of(ch.x_in) + 2));
            cm.area_in.push((PI * ch.inlet.diameter * ch.inlet.diameter) / 4.0);
            cm.area_out.push((PI * ch.outlet.diameter * ch.outlet.diameter) / 4.0);
            cm.volume.push(section_area(&ch.section) * body_length);
            cm.mode_from.push(start);
            cm.mode_to.push(cm.k_total.len());
        }
        cm.count = cm.k_total.len();
        cm.gain = vec![0.0; cm.cell_in.len()];
        cm.omega = vec![0.0; cm.count];
        cm.amp = vec![0.0; cm.count];
        cm.aux = vec![0.0; cm.count];
        cm
    }

    /// Follow the gas in each body: the modes sit at `c k`, and the drive scales with `gamma p`.
    fn refresh(&mut self, rho: &[f64], mom: &[f64], en: &[f64]) {
        for ch in 0..self.cell_in.len() {
            let mut c2 = 0.0;
            let mut p = 0.0;
            let from = self.body_from[ch];
            let to = self.body_to[ch].min(self.n);
            for i in from..to {
                let r = rho[i];
                let u = mom[i] / r;
                let pi = math::max((GAMMA - 1.0) * (en[i] - 0.5 * r * u * u), 1e-3);
                p += pi;
                c2 += (GAMMA * pi) / r;
            }
            let cells = (to as f64 - from as f64).max(1.0);
            let c = math::sqrt(c2 / cells);
            self.gain[ch] = (GAMMA * (p / cells)) / self.volume[ch];
            for k in self.mode_from[ch]..self.mode_to[ch] {
                self.omega[k] = c * self.k_total[k];
            }
        }
    }

    /// Advance every mode by `dt` and push its pressure back onto the pipes.
    pub fn step(&mut self, dt: f64, rho: &[f64], mom: &mut [f64], en: &mut [f64], area_cell: &[f64], inv_vol: &[f64]) {
        self.refresh_counter -= 1;
        if self.refresh_counter <= 0 {
            self.refresh(rho, mom, en);
            self.refresh_counter = REFRESH_INTERVAL;
        }
        for ch in 0..self.cell_in.len() {
            let i_in = self.cell_in[ch];
            let i_out = self.cell_out[ch];
            let u_in = mom[i_in] / rho[i_in];
            let u_out = mom[i_out] / rho[i_out];
            let q_in = u_in * area_cell[i_in];
            let q_out = u_out * area_cell[i_out];
            let g = self.gain[ch];
            let mut p_in = 0.0;
            let mut p_out = 0.0;
            for k in self.mode_from[ch]..self.mode_to[ch] {
                let w = self.omega[k];
                let drive = g * (self.phi_in[k] * q_in - self.phi_out[k] * q_out);
                let a = self.amp[k];
                let b = self.aux[k] - dt * (w * w * a + 2.0 * MODE_ZETA * w * (self.aux[k] + drive));
                let na = a + dt * (b + drive);
                self.aux[k] = b;
                self.amp[k] = na;
                p_in += na * self.phi_in[k];
                p_out += na * self.phi_out[k];
            }
            let dm_in = -dt * p_in * self.area_in[ch] * inv_vol[i_in];
            mom[i_in] += dm_in;
            en[i_in] += dm_in * u_in;
            let dm_out = dt * p_out * self.area_out[ch] * inv_vol[i_out];
            mom[i_out] += dm_out;
            en[i_out] += dm_out * u_out;
        }
    }

    pub fn reset(&mut self) {
        self.amp.fill(0.0);
        self.aux.fill(0.0);
    }

    /// Acoustic energy held in the modes, J.
    pub fn energy(&self) -> f64 {
        let mut e = 0.0;
        for ch in 0..self.cell_in.len() {
            let g = self.gain[ch];
            if !(g > 0.0) {
                continue;
            }
            for k in self.mode_from[ch]..self.mode_to[ch] {
                let w = self.omega[k];
                let a = self.amp[k];
                let v = self.aux[k];
                e += (a * a + if w > 0.0 { (v * v) / (w * w) } else { 0.0 }) / (2.0 * g);
            }
        }
        e
    }

    /// Frequency of every mode kept, at the current gas state, Hz.
    pub fn frequencies(&self) -> Vec<f64> {
        self.omega.iter().map(|w| w / (2.0 * PI)).collect()
    }
}
