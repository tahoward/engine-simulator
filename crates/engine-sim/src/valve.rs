//! Poppet valve: cam lift profile plus compressible flow through the resulting annular opening.

use std::sync::OnceLock;

use crate::dsp::window_phase;
use crate::math::{self, PI};
use crate::spec::gas;

/// Discharge coefficient of a poppet valve at moderate lift, referred to the curtain area.
pub const VALVE_CD: f64 = 0.72;

/// Lift, m, at crank angle `deg`: `sin(pi u)^2` across the window, zero outside it.
#[inline]
pub fn valve_lift(deg: f64, open: f64, close: f64, max_lift: f64) -> f64 {
    let u = window_phase(deg, open, close);
    if u < 0.0 {
        return 0.0;
    }
    let s = math::sin(PI * u);
    max_lift * s * s
}

/// Effective flow area, m^2, for a given lift: the curtain, until the port throat takes over.
#[inline]
pub fn valve_flow_area(lift: f64, valve_dia: f64) -> f64 {
    if lift <= 0.0 {
        return 0.0;
    }
    let curtain = PI * valve_dia * lift;
    let throat = ((PI * valve_dia * valve_dia) / 4.0) * 0.85;
    math::min(curtain, throat)
}

/// An orifice solve: the mass flow, and the throat state a caller may need.
#[derive(Clone, Copy, Debug, Default)]
pub struct Orifice {
    /// Mass flow, kg/s, always positive.
    pub mdot: f64,
    /// Critical pressure ratio at the gamma given.
    pub critical: f64,
    /// Throat static over upstream stagnation temperature, when it flows.
    pub throat_t: f64,
}

/// Isentropic compressible mass flow through an orifice, kg/s, always positive. Choked below the
/// critical pressure ratio.
#[inline]
pub fn orifice_mass_flow(area: f64, cd: f64, p_up: f64, t_up: f64, p_down: f64, gamma: f64) -> f64 {
    orifice_solve(area, cd, p_up, t_up, p_down, gamma).mdot
}

pub fn orifice_solve(area: f64, cd: f64, p_up: f64, t_up: f64, p_down: f64, gamma: f64) -> Orifice {
    let c = gamma_constants(gamma);
    let mut out = Orifice { mdot: 0.0, critical: c.critical, throat_t: 0.0 };
    if area <= 0.0 || p_up <= p_down || p_up <= 0.0 || t_up <= 0.0 {
        return out;
    }
    let mut pr = p_down / p_up;
    if pr < c.critical {
        pr = c.critical;
    }
    let p2 = math::pow(pr, c.exp2);
    out.throat_t = p2 / pr;
    let term = math::pow(pr, c.exp1) - p2;
    if term <= 0.0 {
        return out;
    }
    let flux = math::sqrt(c.flux_scale * term);
    out.mdot = (cd * area * p_up * flux) / math::sqrt(gas::R * t_up);
    out
}

/// Constants of the isentropic orifice equation that depend only on gamma.
#[derive(Clone, Copy, Debug)]
struct GammaConstants {
    critical: f64,
    exp1: f64,
    exp2: f64,
    flux_scale: f64,
}

/// Gammas are taken in steps of 1/400: a step moves the mass flow by well under 0.1%, and every fixed
/// gamma in use is an exact multiple of it.
const GAMMA_STEPS: f64 = 400.0;
const TABLE_KEYS: usize = 1024;

fn constants_for_key(key: f64) -> GammaConstants {
    let gamma = key / GAMMA_STEPS;
    GammaConstants {
        critical: math::pow(2.0 / (gamma + 1.0), gamma / (gamma - 1.0)),
        exp1: 2.0 / gamma,
        exp2: (gamma + 1.0) / gamma,
        flux_scale: (2.0 * gamma) / (gamma - 1.0),
    }
}

fn gamma_constants(gamma_raw: f64) -> GammaConstants {
    static TABLE: OnceLock<Vec<GammaConstants>> = OnceLock::new();
    let key = math::round(gamma_raw * GAMMA_STEPS);
    if key >= 0.0 && key < TABLE_KEYS as f64 {
        let table = TABLE.get_or_init(|| (0..TABLE_KEYS).map(|k| constants_for_key(k as f64)).collect());
        return table[key as usize];
    }
    constants_for_key(key)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lift_is_zero_outside_the_window() {
        assert_eq!(valve_lift(100.0, 130.0, 375.0, 0.01), 0.0);
        assert!(valve_lift(250.0, 130.0, 375.0, 0.01) > 0.0);
        // Across the seam.
        assert!(valve_lift(5.0, 700.0, 20.0, 0.01) > 0.0);
    }

    #[test]
    fn choked_flow_does_not_depend_on_downstream_pressure() {
        let a = orifice_mass_flow(1e-4, 1.0, 5e5, 1000.0, 1e4, 1.33);
        let b = orifice_mass_flow(1e-4, 1.0, 5e5, 1000.0, 5e4, 1.33);
        assert_eq!(a, b);
    }
}
