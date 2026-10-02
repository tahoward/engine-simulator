//! Sound radiated by the walls of a muffler can.
//!
//! The gas inside a chamber pushes on its shell and end plates, and they ring. Each structural mode is
//! a damped oscillator driven by the chamber's pressure projected onto the mode's shape, and what it
//! radiates is the net volume velocity of the wall moving in that shape:
//!
//! ```text
//!     M w'' + (M w / Q) w' + M w^2 w = F,   F = int (p - p_amb) phi dS,   M = rho_s h int phi^2 dS
//!     q = w' int phi dS
//! ```
//!
//! That volume velocity radiates through a `FarField` sized to the can, as a mouth's does. The
//! walls are taken as they are drawn: a round can's shell rings in its breathing modes, which are all
//! a uniform pressure can drive, and its end plates as clamped discs; an oval or rectangular can's
//! faces and end plates as flat simply supported panels, which for an oval is stiffer than its curved
//! faces really are. Real cans are stiffened with ribs, double skins and packing, so these are the
//! plain can's modes, and a loud one.
//!
//! The only free choice is the damping, `SHELL_ZETA`. Everything else is steel and the drawing.

use crate::cross_modes::ChamberPlacement;
use crate::euler_pipe::{EulerPipe, WALL_RHO};
use crate::math::{self, PI};
use crate::radiation::FarField;
use crate::spec::{ChamberSection, ambient_sound_speed, gas, section_area};

/// Steel: Young's modulus, Pa, and Poisson's ratio.
const STEEL_E: f64 = 200e9;
const STEEL_NU: f64 = 0.3;

/// Modal damping ratio of a welded steel can.
const SHELL_ZETA: f64 = 0.02;

/// Most modes kept per can, the lowest first.
const MAX_SHELL_MODES: usize = 24;

/// Lowest a mode is kept at, Hz.
const MIN_MODE_HZ: f64 = 60.0;

/// Odd mode numbers tried along the can and across each face: only odd ones have a net push, so only
/// they are driven by a uniform pressure or radiate a net volume velocity.
const ODD: [f64; 4] = [1.0, 3.0, 5.0, 7.0];

/// `lambda` of the first three axisymmetric modes of a clamped circular plate.
const CLAMPED_DISC_LAMBDA: [f64; 3] = [3.1962, 6.3064, 9.4395];

/// Bending wave speed factor `sqrt(E / (12 rho (1 - nu^2)))`, m/s: a plate's `sqrt(D / (rho h))` is
/// this times its thickness.
fn plate_speed() -> f64 {
    math::sqrt(STEEL_E / (12.0 * WALL_RHO * (1.0 - STEEL_NU * STEEL_NU)))
}

/// Where a mode takes its pressure from.
#[derive(Clone, Debug)]
enum Drive {
    /// Weights over the duct's cells, m^2: the mode's shape integrated over each cell's stretch of wall.
    Cells(Vec<(usize, f64)>),
    /// One cell's pressure over the whole plate, weighted by the plate's `int phi dS`, m^2.
    Cell(usize, f64),
}

/// One structural mode as a bandpass on velocity, with the gains in and out.
#[derive(Clone, Debug)]
struct Mode {
    hz: f64,
    drive: Drive,
    /// Velocity at resonance per unit force, over the bandpass's unit peak: `Q / (M w)`.
    gain_in: f64,
    /// `int phi dS` times how many identical walls ring together, m^2.
    gain_out: f64,
    b0: f64,
    a1: f64,
    a2: f64,
    x1: f64,
    x2: f64,
    y1: f64,
    y2: f64,
}

impl Mode {
    fn new(hz: f64, mass: f64, drive: Drive, gain_out: f64, sample_rate: f64) -> Mode {
        let q = 1.0 / (2.0 * SHELL_ZETA);
        let w = 2.0 * PI * hz;
        // Bandpass of unit peak, the bilinear transform of `(w/Q) s / (s^2 + (w/Q) s + w^2)`.
        let w0 = w / sample_rate;
        let alpha = math::sin(w0) / (2.0 * q);
        let a0 = 1.0 + alpha;
        Mode {
            hz,
            drive,
            gain_in: q / (mass * w),
            gain_out,
            b0: alpha / a0,
            a1: (-2.0 * math::cos(w0)) / a0,
            a2: (1.0 - alpha) / a0,
            x1: 0.0,
            x2: 0.0,
            y1: 0.0,
            y2: 0.0,
        }
    }

    /// Volume velocity out of the wall, m^3/s, for this sample's chamber pressures.
    #[inline]
    fn process(&mut self, duct: &EulerPipe) -> f64 {
        let force = match &self.drive {
            Drive::Cells(w) => w.iter().map(|&(i, k)| (duct.pressure_at(i) - gas::P_AMB) * k).sum::<f64>(),
            Drive::Cell(i, k) => (duct.pressure_at(*i) - gas::P_AMB) * k,
        };
        let x = force * self.gain_in;
        let y = self.b0 * (x - self.x2) - self.a1 * self.y1 - self.a2 * self.y2;
        self.x2 = self.x1;
        self.x1 = x;
        self.y2 = self.y1;
        self.y1 = y;
        y * self.gain_out
    }
}

/// The shell and end plates of one chamber, and their radiation. On cache lines of its own, as each
/// may be stepped on a thread of its own.
#[repr(align(128))]
pub struct ChamberShell {
    duct: usize,
    modes: Vec<Mode>,
    far_field: FarField,
}

impl ChamberShell {
    /// The shell of `chamber`, in duct `duct` whose cells are `dx` long, walls `wall` thick, m.
    pub fn new(duct: usize, chamber: &ChamberPlacement, dx: f64, cells: usize, wall: f64, sample_rate: f64) -> ChamberShell {
        let s = &chamber.section;
        let length = math::max(chamber.x_out - chamber.x_in, 1e-3);
        let surface_mass = WALL_RHO * wall;
        let bending = wall * plate_speed();
        let top = 0.45 * sample_rate;

        // The body's cells, and the first and last of them for the end plates.
        let first = ((chamber.x_in / dx).floor() as usize).min(cells - 1);
        let last = ((chamber.x_out / dx).ceil() as usize).max(first + 1).min(cells) - 1;
        let axial = |m: f64| -> Vec<(usize, f64)> {
            (first..=last)
                .filter_map(|i| {
                    let x = ((i as f64 + 0.5) * dx - chamber.x_in) / length;
                    (0.0..=1.0).contains(&x).then(|| (i, math::sin(m * PI * x) * dx))
                })
                .collect()
        };

        // (Hz in vacuo, modal mass kg, drive, `int phi dS` of one wall m^2, of all the walls ringing
        // together m^2)
        let mut found: Vec<(f64, f64, Drive, f64, f64)> = Vec::new();
        match s.section {
            ChamberSection::Round => {
                let a = s.width / 2.0;
                // Breathing modes of the shell, Donnell-Mushtari with no lobes:
                // `W^2 = (1 - nu^2) + beta lambda^4`, `lambda = m pi a / L`, `beta = h^2 / (12 a^2)`.
                let ring = math::sqrt(STEEL_E / (WALL_RHO * (1.0 - STEEL_NU * STEEL_NU))) / (2.0 * PI * a);
                let beta = (wall * wall) / (12.0 * a * a);
                let circumference = 2.0 * PI * a;
                for m in ODD {
                    let lambda = (m * PI * a) / length;
                    let hz = ring * math::sqrt(1.0 - STEEL_NU * STEEL_NU + beta * lambda * lambda * lambda * lambda);
                    let w: Vec<(usize, f64)> = axial(m).into_iter().map(|(i, k)| (i, k * circumference)).collect();
                    let mass = surface_mass * circumference * (length / 2.0);
                    let j = circumference * (2.0 * length) / (m * PI);
                    found.push((hz, mass, Drive::Cells(w), j, j));
                }
                // End plates: clamped discs.
                for lambda in CLAMPED_DISC_LAMBDA {
                    let hz = ((lambda * lambda) / (2.0 * PI * a * a)) * bending;
                    let (j, i2) = clamped_disc_integrals(lambda, a);
                    for cell in [first, last] {
                        found.push((hz, surface_mass * i2, Drive::Cell(cell, j), j, j));
                    }
                }
            }
            ChamberSection::Oval | ChamberSection::Rect => {
                // Faces: two of the width and two of the height, each simply supported round its edges. An
                // oval's are shallow cylindrical panels curved as the ellipse is at their middles, which
                // stiffens them in membrane: `w^2 = (D / rho h) k^4 + (E / rho R^2) (k_m / k)^4`.
                let (a, b) = (s.width / 2.0, s.height / 2.0);
                for (face, curvature) in [(s.width, b / (a * a)), (s.height, a / (b * b))] {
                    let curvature = if s.section == ChamberSection::Oval { curvature } else { 0.0 };
                    for m in ODD {
                        for n in ODD {
                            let (km, kn) = ((m * PI) / length, (n * PI) / face);
                            let k2 = km * km + kn * kn;
                            let bend = bending * k2;
                            let membrane = (STEEL_E / WALL_RHO) * curvature * curvature * (km * km / k2).powi(2);
                            let hz = math::sqrt(bend * bend + membrane) / (2.0 * PI);
                            let across = (2.0 * face) / (n * PI);
                            let w: Vec<(usize, f64)> = axial(m).into_iter().map(|(i, k)| (i, k * across)).collect();
                            let mass = surface_mass * (length / 2.0) * (face / 2.0);
                            let j = across * (2.0 * length) / (m * PI);
                            found.push((hz, mass, Drive::Cells(w), j, 2.0 * j));
                        }
                    }
                }
                // End plates.
                for m in ODD {
                    for n in ODD {
                        let hz = (PI / 2.0) * bending * ((m / s.width).powi(2) + (n / s.height).powi(2));
                        let area = s.width * s.height;
                        let j = area * 4.0 / (PI * PI * m * n);
                        for cell in [first, last] {
                            found.push((hz, surface_mass * area / 4.0, Drive::Cell(cell, j), j, j));
                        }
                    }
                }
            }
        }

        // The gas inside is a spring on every wall that changes its volume: pushing out by `w` drops
        // the pressure on this wall by `gamma p dV / V`, `dV` the volume all the walls sweep together.
        let volume = section_area(s) * length;
        let gas_spring = |j_one: f64, j_all: f64| (gas::GAMMA_EXH * gas::P_AMB * j_one * j_all) / volume;
        let mut modes: Vec<(f64, f64, Drive, f64)> = found
            .into_iter()
            .map(|(hz, mass, drive, j_one, j_all)| {
                let w2 = (2.0 * PI * hz).powi(2) + gas_spring(j_one, j_all) / mass;
                (math::sqrt(w2) / (2.0 * PI), mass, drive, j_all)
            })
            .filter(|m| m.0 >= MIN_MODE_HZ && m.0 < top)
            .collect();
        modes.sort_by(|a, b| a.0.total_cmp(&b.0));
        modes.truncate(MAX_SHELL_MODES);
        let modes = modes.into_iter().map(|(hz, mass, drive, out)| Mode::new(hz, mass, drive, out, sample_rate)).collect();

        let radius = math::sqrt(section_area(s) / PI);
        ChamberShell { duct, modes, far_field: FarField::new(sample_rate, (2.0 * ambient_sound_speed()) / radius) }
    }

    /// The duct the chamber is in.
    pub fn duct(&self) -> usize {
        self.duct
    }

    /// What the can radiates this sample, Pa at 1 m.
    #[inline]
    pub fn process(&mut self, duct: &EulerPipe) -> f64 {
        let mut q = 0.0;
        for m in self.modes.iter_mut() {
            q += m.process(duct);
        }
        self.far_field.process(q)
    }

    /// The frequencies it rings at, Hz, lowest first.
    pub fn frequencies(&self) -> Vec<f64> {
        self.modes.iter().map(|m| m.hz).collect()
    }
}

/// `int phi dS` and `int phi^2 dS` of a clamped disc of radius `a` in the axisymmetric mode `lambda`,
/// m^2, with the mode normalised to one at the centre.
fn clamped_disc_integrals(lambda: f64, a: f64) -> (f64, f64) {
    let ratio = bessel_j0(lambda) / bessel_i0(lambda);
    let shape = |r: f64| bessel_j0(lambda * r) - ratio * bessel_i0(lambda * r);
    let centre = shape(0.0);
    const N: usize = 400;
    let (mut j, mut i2) = (0.0, 0.0);
    for k in 0..N {
        let r = (k as f64 + 0.5) / N as f64;
        let phi = shape(r) / centre;
        j += phi * r;
        i2 += phi * phi * r;
    }
    let scale = (2.0 * PI * a * a) / N as f64;
    (j * scale, i2 * scale)
}

/// Bessel function `J0` and modified Bessel function `I0` by their power series: enough for the
/// arguments a plate's first few modes need.
fn bessel_j0(x: f64) -> f64 {
    bessel_series(x, -1.0)
}

fn bessel_i0(x: f64) -> f64 {
    bessel_series(x, 1.0)
}

fn bessel_series(x: f64, sign: f64) -> f64 {
    let q = (x * x) / 4.0;
    let mut term = 1.0;
    let mut sum = 1.0;
    for k in 1..60 {
        term *= (sign * q) / (k as f64 * k as f64);
        sum += term;
    }
    sum
}
