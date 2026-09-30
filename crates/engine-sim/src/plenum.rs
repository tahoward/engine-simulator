//! Intake plenum: one finite control volume between the throttle and the intake runners.
//!
//! It is finite so that gas back-flowing up an intake valve during overlap is held and handed back on
//! the next intake stroke. That residual dilution is the dominant reason a real engine's exhaust
//! temperature collapses at light load, so the composition is tracked as well as the mass.

use crate::math::{self, PI, clamp};
use crate::spec::{EngineSpec, displacement, gas, gas_energy, gas_enthalpy, gas_gamma, gas_temperature};
use crate::valve::orifice_mass_flow;

/// Floor on plenum mass, kg.
const MIN_MASS: f64 = 1e-7;

/// Throttle area at the closed stop, as a fraction of the full bore: leakage and the idle bypass.
const IDLE_BYPASS: f64 = 0.002;

/// Discharge coefficient of the butterfly, closed and wide open.
const CD_CLOSED: f64 = 0.25;
const CD_OPEN: f64 = 0.75;

/// Air velocity through a wide-open throttle at peak rpm, m/s, used to size the bore.
const THROTTLE_DESIGN_VELOCITY: f64 = 25.0;
const THROTTLE_DESIGN_RPM: f64 = 7000.0;

/// Plenum volume as a multiple of total swept volume, when not given explicitly.
const PLENUM_VOLUME_RATIO: f64 = 1.5;

fn total_displacement(spec: &EngineSpec) -> f64 {
    displacement(spec) * math::max(spec.cylinders as f64, 1.0)
}

/// Plenum volume, m^3: the spec's, or sized for the engine.
pub fn plenum_volume_of(spec: &EngineSpec) -> f64 {
    if spec.plenum_volume > 0.0 {
        return spec.plenum_volume;
    }
    PLENUM_VOLUME_RATIO * total_displacement(spec)
}

/// Throttle bore, m: the spec's, or sized to pass peak airflow at the design velocity.
pub fn throttle_dia_of(spec: &EngineSpec) -> f64 {
    if spec.throttle_dia > 0.0 {
        return spec.throttle_dia;
    }
    let area = (total_displacement(spec) * (THROTTLE_DESIGN_RPM / 120.0)) / THROTTLE_DESIGN_VELOCITY;
    math::sqrt((4.0 * area) / PI)
}

pub struct IntakePlenum {
    mass: f64,
    /// Sensible internal energy, J.
    energy: f64,
    burned_mass: f64,
    fuel_mass: f64,
    volume: f64,
    /// The throttle plate's opening, 0..1, and the idle air valve's, as more of the plate's.
    opening: f64,
    bypass: f64,
    /// Effective throttle area, m^2.
    area: f64,
}

impl IntakePlenum {
    pub fn new(spec: &EngineSpec) -> IntakePlenum {
        let volume = math::max(plenum_volume_of(spec), 1e-5);
        let mass = (gas::P_AMB * volume) / (gas::R * gas::T_AMB);
        IntakePlenum {
            mass,
            energy: mass * gas_energy(gas::T_AMB),
            burned_mass: 0.0,
            fuel_mass: 0.0,
            volume,
            opening: spec.throttle,
            bypass: 0.0,
            area: IntakePlenum::throttle_area(spec),
        }
    }

    /// Set the throttle to `opening`, 0..1, in place of the spec's, until the next `set_geometry`.
    pub fn set_opening(&mut self, spec: &EngineSpec, opening: f64) {
        self.opening = opening;
        self.area = IntakePlenum::throttle_area_at(spec, self.opening + self.bypass);
    }

    /// Open the idle air valve, the bypass round the throttle plate, by `bypass`: as much air as that
    /// much more of the plate's opening would pass.
    pub fn set_bypass(&mut self, spec: &EngineSpec, bypass: f64) {
        if bypass == self.bypass {
            return;
        }
        self.bypass = bypass;
        self.area = IntakePlenum::throttle_area_at(spec, self.opening + self.bypass);
    }

    /// Rebuild geometry in place, keeping the gas state.
    pub fn set_geometry(&mut self, spec: &EngineSpec) {
        self.opening = spec.throttle;
        self.area = IntakePlenum::throttle_area_at(spec, self.opening + self.bypass);
        let v = math::max(plenum_volume_of(spec), 1e-5);
        if v == self.volume {
            return;
        }
        let scale = v / self.volume;
        self.volume = v;
        self.mass *= scale;
        self.energy *= scale;
        self.burned_mass *= scale;
        self.fuel_mass *= scale;
    }

    pub fn temp(&self) -> f64 {
        clamp(gas_temperature(self.energy / math::max(self.mass, MIN_MASS)), 150.0, 3000.0)
    }

    /// Absolute pressure, Pa.
    pub fn pressure(&self) -> f64 {
        (math::max(self.mass, MIN_MASS) * gas::R * self.temp()) / self.volume
    }

    pub fn burned_fraction(&self) -> f64 {
        clamp(self.burned_mass / math::max(self.mass, MIN_MASS), 0.0, 1.0)
    }

    pub fn fuel_fraction(&self) -> f64 {
        clamp(self.fuel_mass / math::max(self.mass, MIN_MASS), 0.0, 1.0)
    }

    /// Effective throttle flow area, m^2: geometric area times the plate's discharge coefficient.
    pub fn throttle_area(spec: &EngineSpec) -> f64 {
        IntakePlenum::throttle_area_at(spec, spec.throttle)
    }

    pub fn throttle_area_at(spec: &EngineSpec, opening: f64) -> f64 {
        let d = throttle_dia_of(spec);
        let bore = (PI * d * d) / 4.0;
        let open = 1.0 - math::cos(clamp(opening, 0.0, 1.0) * (PI / 2.0));
        let geometric = bore * (IDLE_BYPASS + (1.0 - IDLE_BYPASS) * open);
        geometric * (CD_CLOSED + (CD_OPEN - CD_CLOSED) * open)
    }

    /// Advance by `dt`, drawing through the throttle from air at `p_up` (Pa) and `t_up` (K): the
    /// atmosphere, or a turbocharger's charge air. `valve_flow` is the net mass flow to the cylinders,
    /// kg/s, positive out of the plenum; `backflow`, 0 or more, is the part of it flowing back in, at
    /// its own temperature and composition. Returns the flow in through the throttle, kg/s.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        dt: f64,
        p_up: f64,
        t_up: f64,
        valve_flow: f64,
        backflow: f64,
        backflow_temp: f64,
        backflow_burned: f64,
        backflow_fuel: f64,
    ) -> f64 {
        let p = self.pressure();
        let t = self.temp();
        let burned = self.burned_fraction();
        let fuel = self.fuel_fraction();

        let area = self.area;
        let throttle_flow = if p < p_up {
            orifice_mass_flow(area, 1.0, p_up, t_up, p, gas::GAMMA_AIR)
        } else {
            -orifice_mass_flow(area, 1.0, p, t, p_up, gas_gamma(t))
        };

        let h_own = gas_enthalpy(t);
        let h_throttle = if throttle_flow >= 0.0 { throttle_flow * gas_enthalpy(t_up) } else { throttle_flow * h_own };
        let drawn = valve_flow + backflow;
        let h_valve = -drawn * h_own + backflow * gas_enthalpy(backflow_temp);

        self.energy += (h_throttle + h_valve) * dt;
        self.mass += (throttle_flow - valve_flow) * dt;

        let burned_in = -drawn * burned + backflow * backflow_burned;
        self.burned_mass += burned_in * dt;
        let fuel_throttle = if throttle_flow >= 0.0 { 0.0 } else { throttle_flow * fuel };
        self.fuel_mass += (fuel_throttle - drawn * fuel + backflow * backflow_fuel) * dt;

        if self.mass < MIN_MASS {
            self.mass = MIN_MASS;
            self.energy = MIN_MASS * gas_energy(math::max(t, 150.0));
        }
        self.burned_mass = clamp(self.burned_mass, 0.0, self.mass);
        self.fuel_mass = clamp(self.fuel_mass, 0.0, self.mass - self.burned_mass);

        let e_min = self.mass * gas_energy(150.0);
        if self.energy < e_min {
            self.energy = e_min;
        }
        if !self.energy.is_finite() || !self.mass.is_finite() {
            self.reset();
        }
        throttle_flow
    }

    pub fn reset(&mut self) {
        self.mass = (gas::P_AMB * self.volume) / (gas::R * gas::T_AMB);
        self.energy = self.mass * gas_energy(gas::T_AMB);
        self.burned_mass = 0.0;
        self.fuel_mass = 0.0;
    }
}
