//! Where the engine's exhaust ports are, and where a straight-run pipe from one ends up.
//!
//! Plain arithmetic on the spec, so the exhaust can be compiled to fit the engine: a manifold
//! chaining one bank's runners, or two banks' downpipes meeting behind the engine, only snaps together
//! if its pipes are the right lengths, and those lengths are set by where the ports are.

use crate::math::{self, PI};
use crate::spec::{
    EngineSpec, PipeSegment, clearance_volume, crank_pins, cylinder_spacing, firing_plan, physical_bank,
    physical_bank_count,
};

pub type Vec3 = [f64; 3];

/// A cylinder's exhaust port: where its pipe attaches, and which way the port points.
pub struct Port {
    pub position: Vec3,
    pub direction: Vec3,
}

pub fn exhaust_port_of(spec: &EngineSpec, cylinder: usize) -> Port {
    let plan = firing_plan(spec);
    let pins = crank_pins(spec);
    let pin = pins.iter().position(|p| p.cylinders.contains(&cylinder)).unwrap_or(0);
    let z = (pin as f64 - (pins.len() as f64 - 1.0) / 2.0) * cylinder_spacing(spec);

    let deg = PI / 180.0;
    let bank_rotation = if plan.banks.get(cylinder).copied().unwrap_or(0) == 0 { 0.0 } else { -spec.v_angle * deg };
    let rot = bank_rotation + if plan.bank_count > 1 { (spec.v_angle / 2.0) * deg } else { 0.0 };

    let crown_offset = spec.bore * 0.34;
    let bore_area = (PI * spec.bore * spec.bore) / 4.0;
    let deck_y = spec.stroke / 2.0 + spec.rod_length + crown_offset + clearance_volume(spec) / bore_area;
    let head_height = spec.bore * 0.52;

    let side = if physical_bank_count(spec) > 1 && physical_bank(spec, cylinder) == 0 { -1.0 } else { 1.0 };
    let c = math::cos(rot);
    let s = math::sin(rot);
    let px = side * spec.bore * 1.15;
    let py = deck_y + head_height * 0.45;
    // Straight out of the side of the head, square to it.
    Port { position: [px * c - py * s, px * s + py * c, z], direction: [side * c, side * s, 0.0] }
}

/// `dir` turned by `yaw` about the vertical, then by `pitch` about its new horizontal right axis.
pub fn turn_dir(dir: Vec3, yaw: f64, pitch: f64) -> Vec3 {
    let [mut x, mut y, mut z] = normalise(dir);
    if yaw != 0.0 {
        let c = math::cos(yaw);
        let s = math::sin(yaw);
        let nx = x * c + z * s;
        let nz = -x * s + z * c;
        x = nx;
        z = nz;
    }
    if pitch != 0.0 {
        let (mut kx, mut ky, mut kz) = (-z, 0.0, x);
        let kl = math::hypot(&[kx, ky, kz]);
        if kl < 1e-5 {
            // Straight up or down: the yaw says which way it pitches away from vertical.
            kx = math::sin(yaw);
            ky = 0.0;
            kz = math::cos(yaw);
        } else {
            kx /= kl;
            ky /= kl;
            kz /= kl;
        }
        let c = math::cos(pitch);
        let s = math::sin(pitch);
        let cx = ky * z - kz * y;
        let cy = kz * x - kx * z;
        let cz = kx * y - ky * x;
        let (nx, ny, nz) = (x * c + cx * s, y * c + cy * s, z * c + cz * s);
        x = nx;
        y = ny;
        z = nz;
    }
    normalise([x, y, z])
}

/// The yaw and pitch that `turn_dir` needs to turn `from` onto `to`.
pub fn turn_between_dirs(from: Vec3, to: Vec3) -> (f64, f64) {
    let f = normalise(from);
    let t = normalise(to);
    let fh = math::hypot(&[f[0], f[2]]);
    let th = math::hypot(&[t[0], t[2]]);
    let mut yaw = 0.0;
    if fh > 1e-5 && th > 1e-5 {
        let cross = f[2] * t[0] - f[0] * t[2];
        let dot = f[0] * t[0] + f[2] * t[2];
        yaw = math::atan2(cross / (fh * th), dot / (fh * th));
    } else if th > 1e-5 {
        // From straight up or down, the yaw is the way it pitches away to: see `turn_dir`.
        yaw = math::atan2(-t[2], t[0]);
    }
    let elevation = |v: Vec3| math::asin(math::min(math::max(v[1], -1.0), 1.0));
    (yaw, elevation(t) - elevation(f))
}

/// Where a straight-run pipe starting at `origin` along `dir` ends, and which way it is then going.
pub fn sweep_end(segments: &[PipeSegment], origin: Vec3, dir: Vec3) -> (Vec3, Vec3) {
    let mut d = normalise(dir);
    let mut p = origin;
    for seg in segments {
        d = turn_dir(d, seg.yaw, seg.pitch);
        p[0] += d[0] * seg.length;
        p[1] += d[1] * seg.length;
        p[2] += d[2] * seg.length;
    }
    (p, d)
}

pub fn distance(a: Vec3, b: Vec3) -> f64 {
    math::hypot(&[a[0] - b[0], a[1] - b[1], a[2] - b[2]])
}

fn normalise(v: Vec3) -> Vec3 {
    let h = math::hypot(&[v[0], v[1], v[2]]);
    // `|| 1`: a zero or NaN length divides by one.
    let l = if h == 0.0 || h.is_nan() { 1.0 } else { h };
    [v[0] / l, v[1] / l, v[2] / l]
}
