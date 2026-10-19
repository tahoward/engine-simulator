//! Where the engine's exhaust ports are, and where a straight-run pipe from one ends up.
//!
//! Plain arithmetic on the spec, so the exhaust can be compiled to fit the engine: a manifold
//! chaining one bank's runners, or two banks' downpipes meeting behind the engine, only snaps together
//! if its pipes are the right lengths, and those lengths are set by where the ports are.

use crate::math::{self, PI};
use crate::spec::{
    EngineSpec, PipeSegment, clearance_volume, cylinder_z, firing_plan, is_rotary, physical_bank,
    physical_bank_count,
};

pub type Vec3 = [f64; 3];

/// A cylinder's exhaust port: where its pipe attaches, and which way the port points.
pub struct Port {
    pub position: Vec3,
    pub direction: Vec3,
}

pub fn exhaust_port_of(spec: &EngineSpec, cylinder: usize) -> Port {
    if is_rotary(spec) {
        return rotor_port(spec, cylinder, ROTOR_EXHAUST_PORT);
    }
    let plan = firing_plan(spec);
    let z = cylinder_z(spec, cylinder);

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

/// Where round a rotary's housing its peripheral ports are, as the trochoid's parameter `alpha`, degrees:
/// either side of the waist where the exhaust stroke meets the intake, the exhaust low on the housing's
/// side, pointing out of it, and the intake high, pointing up and out. The web app's `ROTOR_EXHAUST_PORT`
/// and `ROTOR_INTAKE_PORT`; see `trochoid_at`.
pub const ROTOR_EXHAUST_PORT: f64 = 240.0;
pub const ROTOR_INTAKE_PORT: f64 = 315.0;

/// How far a rotary's port flanges stand out from the trochoid, m: the housing's wall.
pub const HOUSING_WALL: f64 = 0.03;

/// The point of a rotary's trochoid at parameter `alpha`, degrees, across the shaft in the scene's frame,
/// x across and y up, m, and the way out of the housing there, a unit vector.
///
/// The trochoid is the path the apexes trace, `e (cos 3a, sin 3a) + R (cos a, sin a)`, turned a quarter
/// turn so its long axis stands upright, as Mazda's do: its waists face the sides, the spark plugs at the
/// waist on -x, where the chambers fire, and the ports at the one on +x.
pub fn trochoid_at(spec: &EngineSpec, alpha: f64) -> ([f64; 2], [f64; 2]) {
    let a = alpha * (PI / 180.0);
    let (e, r) = (spec.eccentricity, spec.rotor_radius);
    let (s1, c1) = (math::sin(a), math::cos(a));
    let (s3, c3) = (math::sin(3.0 * a), math::cos(3.0 * a));
    let point = [-(e * s3 + r * s1), e * c3 + r * c1];
    // The tangent, the way the parameter runs, turned outward.
    let tangent = [-(3.0 * e * c3 + r * c1), -(3.0 * e * s3 + r * s1)];
    let h = math::hypot(&[tangent[0], tangent[1]]);
    (point, [tangent[1] / h, -tangent[0] / h])
}

/// Rotor `rotor`'s peripheral port at `alpha` round its housing: its flange, a housing wall out from the
/// trochoid, and the way it points.
pub fn rotor_port(spec: &EngineSpec, rotor: usize, alpha: f64) -> Port {
    let (p, n) = trochoid_at(spec, alpha);
    let z = cylinder_z(spec, rotor);
    Port {
        position: [p[0] + n[0] * HOUSING_WALL, p[1] + n[1] * HOUSING_WALL, z],
        direction: [n[0], n[1], 0.0],
    }
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
    // A zero or NaN length divides by one.
    let l = if h == 0.0 || h.is_nan() { 1.0 } else { h };
    [v[0] / l, v[1] / l, v[2] / l]
}
