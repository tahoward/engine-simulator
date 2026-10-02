//! Afterfire: unburned fuel that leaves a cylinder and burns in its exhaust.
//!
//! A spark cut, a misfire or a burn still going when the exhaust valve opens sends fuel, and the air to
//! burn it, out through the valve. Each cylinder keeps a pocket of it in the leading stretch of its
//! primary: filled by what its valve sends out, and pushed on down the pipe by the gas that follows.
//! There the mixture is lit by hot gas, as a blowdown from a cylinder that fired, or a pulse coming back
//! from the collector, and its ignition delay runs out. Hot and already mixed, it burns in a few
//! milliseconds, and its heat goes into the gas of those cells, so the pressure it raises travels down
//! the exhaust like any other pulse.
//!
//! The delay follows an Arrhenius law integrated over the temperature history (Livengood and Wu's
//! integral), and nothing reacts below the temperature where hydrocarbons stop oxidising in an exhaust
//! port. A pocket too lean to carry a flame never lights, and a steady engine leaves too little in the
//! pipe to count, so it costs nothing until there is something to burn.

use crate::cylinder::COMBUSTION_EFFICIENCY;
use crate::dsp::Noise;
use crate::math::{self, PI, clamp};
use crate::spec::gas;

/// Length of each primary's afterfire zone, m: the head port and the start of the primary, where the
/// next blowdown is still hot enough to light it.
pub const AFTERFIRE_ZONE_LENGTH: f64 = 0.4;

/// Burnable fuel below which a pocket is not counted, as a share of a full charge's stoichiometric fuel.
const POCKET_FLOOR: f64 = 0.01;

/// Leanest mixture that carries a flame, as the burnable fuel's share of the gas it is in, at room
/// temperature: gasoline's lean limit, an equivalence ratio of about 0.55. The limit falls with
/// temperature by Zabetakis's form of the Burgess-Wheeler rule, by `LEAN_LIMIT_PER_K` of itself per
/// kelvin, to about 0.3 of itself at 1300 K; no lower than `LEAN_LIMIT_FLOOR` of itself.
const Y_LEAN: f64 = 0.036;
const LEAN_LIMIT_PER_K: f64 = 0.000721;
const LEAN_LIMIT_FLOOR: f64 = 0.2;

/// Below this temperature, K, nothing reacts: hydrocarbons stop oxidising in an exhaust port near it.
const T_IGNITION_FLOOR: f64 = 850.0;

/// Ignition delay of the mixture, `TAU_REF exp(E_OVER_R (1/T - 1/T_REF_IGN))`: 1 ms at 1100 K, 5 ms at
/// 1000 K, 38 ms at 900 K, the order of shock-tube delays for gasoline near one atmosphere.
const TAU_REF: f64 = 1e-3;
const T_REF_IGN: f64 = 1100.0;
const E_OVER_R: f64 = 18000.0;

/// Spread of the ignition threshold from one pocket to the next, log-normal.
const IGNITION_SCATTER: f64 = 0.6;

/// Least share of the burnable fuel one ignition burns; the rest of it is drawn at random.
const BURN_SHARE_MIN: f64 = 0.5;

/// How long a pocket burns, s: typically, and at least and at most. Hot and already mixed, it burns
/// nearly all at once rather than behind a slow flame front.
const BURN_TIME: f64 = 1.5e-3;
const BURN_TIME_MIN: f64 = 0.7e-3;
const BURN_TIME_MAX: f64 = 4e-3;

/// Leanest burnable fuel share of the gas that carries a flame at `t`, K.
pub fn lean_limit(t: f64) -> f64 {
    Y_LEAN * math::max(1.0 - LEAN_LIMIT_PER_K * (t - gas::T_AMB), LEAN_LIMIT_FLOOR)
}

/// One cylinder's pocket of unburned mixture in the leading cells of its primary. On cache lines of
/// its own, as each may be stepped on a thread of its own.
#[derive(Clone, Debug)]
#[repr(align(128))]
pub struct Pocket {
    /// Unburned fuel and air in the zone, kg.
    pub fuel: f64,
    pub air: f64,
    /// Livengood-Wu integral of time over ignition delay, and where this pocket lights.
    progress: f64,
    threshold: f64,
    /// The burn under way: its heat, J, how long it has run and lasts, s.
    burn_energy: f64,
    burn_time: f64,
    burn_duration: f64,
    /// Heat planned but not yet taken by the pipe, J.
    carry: f64,
    /// Heat to release over the next sample, W, and what was planned for the last one, J.
    rate: f64,
    planned: f64,
    noise: Noise,
    /// Ignitions so far, and the heat released into the pipe, J.
    pub events: u64,
    pub heat_released: f64,
}

impl Pocket {
    pub fn new(seed: f64) -> Pocket {
        let mut noise = Noise::new(seed);
        let threshold = math::exp(IGNITION_SCATTER * noise.gaussian());
        Pocket {
            fuel: 0.0,
            air: 0.0,
            progress: 0.0,
            threshold,
            burn_energy: 0.0,
            burn_time: 0.0,
            burn_duration: 0.0,
            carry: 0.0,
            rate: 0.0,
            planned: 0.0,
            noise,
            events: 0,
            heat_released: 0.0,
        }
    }

    /// Fuel the pocket's air can burn, kg.
    pub fn burnable(&self) -> f64 {
        math::min(self.fuel, self.air / gas::AFR_STOICH)
    }

    /// Heat to release over the next sample, W.
    pub fn rate(&self) -> f64 {
        self.rate
    }

    pub fn burning(&self) -> bool {
        self.burn_duration > 0.0
    }

    /// Empty the pocket and end any burn.
    pub fn clear(&mut self) {
        self.fuel = 0.0;
        self.air = 0.0;
        self.progress = 0.0;
        self.burn_duration = 0.0;
        self.carry = 0.0;
        self.rate = 0.0;
        self.planned = 0.0;
    }

    /// Advance by `dt`. `fuel_in` and `air_in` are what the valve sent out this sample, kg, and
    /// `inflow` all the gas it sent, kg; `zone_mass` is the gas in the zone, kg; `hottest` gives the
    /// temperature of its hottest cell, K, read only when there is something to burn; `taken` is the
    /// heat the pipe took of the last sample's rate, J; `floor` is the burnable fuel below which the
    /// pocket is not counted, kg.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        dt: f64,
        fuel_in: f64,
        air_in: f64,
        inflow: f64,
        zone_mass: f64,
        hottest: impl FnOnce() -> f64,
        taken: f64,
        floor: f64,
    ) {
        self.fuel += fuel_in;
        self.air += air_in;
        let burning = self.burning();
        if burning {
            self.heat_released += taken;
            self.carry += self.planned - taken;
        }
        if !burning && self.fuel <= 0.0 {
            self.rate = 0.0;
            self.planned = 0.0;
            return;
        }

        let mass = math::max(zone_mass, 1e-12);
        // The gas the valve sends in pushes as much of the mixed zone on down the pipe.
        if inflow > 0.0 {
            let keep = mass / (mass + inflow);
            self.fuel *= keep;
            self.air *= keep;
        }
        self.fuel = math::min(self.fuel, mass);
        self.air = math::min(self.air, mass - self.fuel);

        let burnable = self.burnable();
        if !burning && burnable < floor {
            self.progress = 0.0;
            self.rate = 0.0;
            self.planned = 0.0;
            return;
        }

        if !burning {
            let hottest = hottest();
            if burnable / mass >= lean_limit(hottest) && hottest >= T_IGNITION_FLOOR {
                self.progress += dt / (TAU_REF * math::exp(E_OVER_R * (1.0 / hottest - 1.0 / T_REF_IGN)));
            } else {
                self.progress = 0.0;
            }
            if self.progress >= self.threshold {
                self.ignite(burnable);
            }
        }

        if self.burning() {
            self.plan(dt);
        } else {
            self.rate = 0.0;
            self.planned = 0.0;
        }
    }

    fn ignite(&mut self, burnable: f64) {
        let share = BURN_SHARE_MIN + (1.0 - BURN_SHARE_MIN) * (self.noise.next() + 1.0) / 2.0;
        let burned = share * burnable;
        self.fuel -= burned;
        self.air = math::max(self.air - burned * gas::AFR_STOICH, 0.0);
        self.burn_energy = burned * gas::FUEL_LHV * COMBUSTION_EFFICIENCY;
        self.burn_time = 0.0;
        self.burn_duration = clamp(BURN_TIME * math::exp(0.4 * self.noise.gaussian()), BURN_TIME_MIN, BURN_TIME_MAX);
        self.carry = 0.0;
        self.progress = 0.0;
        self.threshold = math::exp(IGNITION_SCATTER * self.noise.gaussian());
        self.events += 1;
    }

    /// The heat for the next sample: a raised cosine over the burn, integrated exactly over the sample so
    /// the whole burn releases exactly its energy, and whatever the pipe could not take yet. Past twice
    /// the burn's length, what is left over is let go.
    fn plan(&mut self, dt: f64) {
        let d = self.burn_duration;
        let shape = |t: f64| {
            let u = math::min(t / d, 1.0);
            u - math::sin(2.0 * PI * u) / (2.0 * PI)
        };
        let t0 = self.burn_time;
        let t1 = t0 + dt;
        let fresh = self.burn_energy * (shape(t1) - shape(t0));
        self.burn_time = t1;
        if t1 > 2.0 * d {
            self.burn_duration = 0.0;
            self.carry = 0.0;
            self.rate = 0.0;
            self.planned = 0.0;
            return;
        }
        self.planned = fresh + math::max(self.carry, 0.0);
        self.carry = 0.0;
        self.rate = self.planned / dt;
    }
}

/// Every cylinder's pocket.
#[derive(Clone, Debug, Default)]
pub struct Afterfire {
    pockets: Vec<Pocket>,
    floor: f64,
}

impl Afterfire {
    /// For `cylinders` cylinders, each drawing `full_charge` of air, kg, at stoichiometric fuel fraction
    /// `stoich_fuel`.
    pub fn new(cylinders: usize, full_charge: f64, stoich_fuel: f64) -> Afterfire {
        let pockets =
            (0..cylinders).map(|b| Pocket::new(0x6a09e667_u32 as f64 + b as f64 * 0x9e3779b as f64)).collect();
        Afterfire { pockets, floor: POCKET_FLOOR * full_charge * stoich_fuel }
    }

    pub fn set_charge(&mut self, full_charge: f64, stoich_fuel: f64) {
        self.floor = POCKET_FLOOR * full_charge * stoich_fuel;
    }

    /// Advance cylinder `b`'s pocket; see `Pocket::step`.
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        b: usize,
        dt: f64,
        fuel_in: f64,
        air_in: f64,
        inflow: f64,
        zone_mass: f64,
        hottest: impl FnOnce() -> f64,
        taken: f64,
    ) {
        let floor = self.floor;
        self.pockets[b].step(dt, fuel_in, air_in, inflow, zone_mass, hottest, taken, floor);
    }

    /// Every pocket, and the burnable fuel below which a pocket is not counted, kg: for stepping the
    /// pockets apart, with `Pocket::step`.
    pub fn pockets_mut(&mut self) -> (&mut [Pocket], f64) {
        (&mut self.pockets, self.floor)
    }

    /// Heat cylinder `b`'s pocket releases over the next sample, W.
    #[inline]
    pub fn heat_rate(&self, b: usize) -> f64 {
        self.pockets[b].rate
    }

    pub fn pocket(&self, b: usize) -> &Pocket {
        &self.pockets[b]
    }

    /// Ignitions so far, over every cylinder.
    pub fn events(&self) -> u64 {
        self.pockets.iter().map(|p| p.events).sum()
    }

    /// Heat released into the pipes so far, J.
    pub fn heat_released(&self) -> f64 {
        self.pockets.iter().map(|p| p.heat_released).sum()
    }

    pub fn clear(&mut self) {
        for p in self.pockets.iter_mut() {
            p.clear();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DT: f64 = 1.0 / 48000.0;
    /// A zone of 0.2 g of gas, and a floor of a hundredth of a 12 mg pocket.
    const ZONE: f64 = 2e-4;
    const FLOOR: f64 = 1.2e-7;

    /// Step `pocket` for `seconds` in a zone at `temp`, the pipe taking all the heat it is given.
    fn hold(pocket: &mut Pocket, seconds: f64, temp: f64) {
        for _ in 0..(seconds / DT) as usize {
            let taken = pocket.planned;
            pocket.step(DT, 0.0, 0.0, 0.0, ZONE, || temp, taken, FLOOR);
        }
    }

    fn stoichiometric(fuel: f64) -> Pocket {
        let mut p = Pocket::new(1.0);
        p.fuel = fuel;
        p.air = fuel * gas::AFR_STOICH;
        p
    }

    /// Air alone has nothing to burn.
    #[test]
    fn a_pocket_of_air_alone_never_ignites() {
        let mut p = Pocket::new(1.0);
        p.air = 1e-4;
        hold(&mut p, 0.2, 1300.0);
        assert_eq!(p.events, 0);
    }

    /// Fuel with no air to burn it does not burn.
    #[test]
    fn a_rich_pocket_with_no_air_never_ignites() {
        let mut p = Pocket::new(1.0);
        p.fuel = 1e-5;
        hold(&mut p, 0.2, 1300.0);
        assert_eq!(p.events, 0);
    }

    /// A charge in a cold header waits there, then lights within milliseconds of hot gas reaching it.
    #[test]
    fn a_charge_sitting_in_a_cold_header_waits_for_hot_gas() {
        let mut p = stoichiometric(1.2e-5);
        hold(&mut p, 0.05, 600.0);
        assert_eq!(p.events, 0);
        assert_eq!(p.fuel, 1.2e-5, "nothing washes it out while the valve is shut");
        hold(&mut p, 0.01, 1200.0);
        assert_eq!(p.events, 1);
    }

    /// The gas the valve sends in carries the pocket on down the pipe, in proportion.
    #[test]
    fn a_pocket_washes_out_with_the_flow_behind_it() {
        let mut p = Pocket::new(1.0);
        p.fuel = 1e-6;
        p.air = 1e-5;
        let inflow = 5e-6;
        for _ in 0..10 {
            p.step(DT, 0.0, 0.0, inflow, ZONE, || 600.0, 0.0, FLOOR);
        }
        let expected = 1e-6 * (ZONE / (ZONE + inflow)).powi(10);
        assert!((p.fuel / expected - 1.0).abs() < 1e-12, "{} left, {expected} expected", p.fuel);
    }

    /// A mixture too lean to carry a flame never lights, however hot.
    #[test]
    fn too_dilute_a_pocket_does_not_pop() {
        let mut p = stoichiometric(lean_limit(1300.0) * ZONE * 0.9);
        hold(&mut p, 0.2, 1300.0);
        assert_eq!(p.events, 0);
    }

    /// A burn releases the fuel it burns times its heating value.
    #[test]
    fn the_heat_released_is_the_fuel_burned_times_its_heating_value() {
        let mut p = stoichiometric(1.2e-5);
        let before = p.fuel;
        hold(&mut p, 0.02, 1200.0);
        assert_eq!(p.events, 1);
        let burned = before - p.fuel;
        let expected = burned * gas::FUEL_LHV * COMBUSTION_EFFICIENCY;
        assert!(burned > 0.5 * before - 1e-12 && burned <= before, "burned {burned} of {before}");
        assert!((p.heat_released / expected - 1.0).abs() < 1e-9, "{} J, {expected} J", p.heat_released);
    }

    /// Heat the pipe cannot take at once is released on the following samples, not lost.
    #[test]
    fn heat_the_pipe_refuses_is_released_later_not_lost() {
        let mut p = stoichiometric(1.2e-5);
        let before = p.fuel;
        for _ in 0..(0.03 / DT) as usize {
            let taken = p.planned * 0.5;
            p.step(DT, 0.0, 0.0, 0.0, ZONE, || 1200.0, taken, FLOOR);
        }
        let expected = (before - p.fuel) * gas::FUEL_LHV * COMBUSTION_EFFICIENCY;
        assert_eq!(p.events, 1);
        assert!((p.heat_released / expected - 1.0).abs() < 1e-3, "{} J of {expected} J", p.heat_released);
    }

    /// Ignition comes at irregular moments, but the same ones every run.
    #[test]
    fn ignition_is_irregular_but_the_same_every_run() {
        let lights = |seed: f64| {
            let mut p = Pocket::new(seed);
            let mut at = Vec::new();
            for k in 0..(1.0 / DT) as usize {
                // A stoichiometric charge arriving every 20 ms, into gas at 950 K.
                let (f, a) = if k % 960 == 0 { (6e-6, 6e-6 * gas::AFR_STOICH) } else { (0.0, 0.0) };
                let taken = p.planned;
                let before = p.events;
                p.step(DT, f, a, 0.0, ZONE, || 950.0, taken, FLOOR);
                if p.events > before {
                    at.push(k);
                }
            }
            at
        };
        let a = lights(3.0);
        assert!(a.len() > 5, "{} ignitions", a.len());
        assert_eq!(a, lights(3.0));
        let gaps: Vec<usize> = a.windows(2).map(|w| w[1] - w[0]).collect();
        assert!(gaps.iter().any(|&g| g != gaps[0]), "evenly spaced: {gaps:?}");
    }
}
