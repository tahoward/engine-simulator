/**
 * Intake plenum: one finite control volume between the throttle and the intake valves.
 *
 * It exists for one reason, and it is not the intake noise. With an *infinite*
 * fixed-pressure plenum, gas that back-flows up an intake valve during overlap disappears
 * into the reservoir and never comes back. A real manifold holds it, and hands it straight
 * back on the next intake stroke.
 *
 * Without that path the exhaust temperature would barely move with load. Residual dilution is the dominant reason a real engine's EGT collapses at idle:
 * burned gas absorbs heat and carries no fuel, so a cylinder breathing 30% residual
 * releases far less energy per kilogram of charge than one breathing clean air. Throttle a
 * real engine and the residual fraction climbs steeply, because the cylinder is pumping
 * against a near-vacuum in the manifold and spits a much larger share of its exhaust back
 * up the intake. None of that could happen without somewhere to put the back-flow.
 *
 * So the composition is tracked, not just the mass: `burnedMass` is how much of what is in
 * here is spent gas. That fraction rides back into the cylinder, where it dilutes the
 * charge and shows up as less heat release, a cooler exhaust, a slower wave, and a lower
 * note — with nothing fitted anywhere along the way.
 *
 * `fuelMass` is how much is fuel. Only air comes in past the throttle: the fuel is injected in each
 * runner (see `IntakeRunners`), so what fuel the plenum holds is what the cylinders pushed back up
 * their runners with their own mixture.
 *
 * Thermodynamically this is the same filling-and-emptying model as the cylinder, minus the
 * moving boundary, and it deliberately shares the cylinder's temperature-dependent `cv`
 * and energy datum so that enthalpy handed across a valve is conserved exactly.
 */

import {
  GAS,
  type EngineSpec,
  displacement,
  gasEnergy,
  gasEnthalpy,
  gasGamma,
  gasTemperature,
} from '../../model/spec.js';
import { clamp } from './dsp.js';
import { orificeMassFlow } from './valve.js';

/** Floor on plenum mass, kg. Keeps `energy/mass` sane if someone asks for a silly volume. */
const MIN_MASS = 1e-7;

/**
 * Throttle area at the closed stop, as a fraction of the full bore.
 *
 * Every throttle leaks, and most engines add a deliberate idle bypass on top. Without it a
 * fully closed throttle would pump the manifold down to nothing and the engine could not
 * idle at all — which is true of the hardware too, and is why the bypass exists.
 */
const IDLE_BYPASS = 0.002;

/**
 * Discharge coefficient of the butterfly, closed and wide open.
 *
 * A throttle plate is a very poor orifice near its stop and a good one wide open: the
 * geometric gap either side of a nearly-shut plate is mostly filled by the plate's own wake,
 * and measured butterfly discharge coefficients run 0.2-0.3 there against 0.7-0.8 at full
 * travel. Only the two endpoints are needed — see `throttleArea`.
 */
const CD_CLOSED = 0.25;
const CD_OPEN = 0.75;

/**
 * Air velocity through a wide-open throttle at peak rpm, m/s, used to size the bore.
 *
 * A throttle body is sized so the engine can breathe at its power peak. This has to be
 * *derived* rather than typed, because the same fixed 40 mm bore and 0.75 litre manifold that
 * suit a 500 cc single are absurd on a 6 litre V8 — eight cylinders drawing through a
 * single's throttle into a manifold smaller than one of their own strokes. That mis-sizing
 * would make the V8 idle at 5000 rpm with the throttle shut: the plenum would be stiff enough
 * for the explicit step to undershoot into the mass floor, and that floor *adds* mass, so the
 * manifold would pressurise and feed the engine air it should not have.
 *
 * The velocity is checked against hardware rather than guessed: a 500 cc single carries a
 * 40 mm throttle, and passing its peak airflow through 40 mm works out at 23 m/s. 90 m/s — a
 * plausible-sounding figure, but one that belongs to a *port* — would size that same engine at
 * 20 mm, throttling it so hard that combustion weakens and the cycle-to-cycle variation
 * collapses.
 */
const THROTTLE_DESIGN_VELOCITY = 25;
const THROTTLE_DESIGN_RPM = 7000;

/** Plenum volume as a multiple of total swept volume, when not given explicitly. */
const PLENUM_VOLUME_RATIO = 1.5;

/** Total swept volume of the whole engine, m^3. */
function totalDisplacement(spec: EngineSpec): number {
  return displacement(spec) * Math.max(spec.cylinders, 1);
}

/**
 * Plenum volume, m^3. `spec.plenumVolume` overrides; 0 or less means "size it for me".
 */
export function plenumVolumeOf(spec: EngineSpec): number {
  if (spec.plenumVolume > 0) return spec.plenumVolume;
  return PLENUM_VOLUME_RATIO * totalDisplacement(spec);
}

/** Throttle bore, m. `spec.throttleDia` overrides; 0 or less means "size it for me". */
export function throttleDiaOf(spec: EngineSpec): number {
  if (spec.throttleDia > 0) return spec.throttleDia;
  // Area to pass peak airflow at the design velocity.
  const area =
    (totalDisplacement(spec) * (THROTTLE_DESIGN_RPM / 120)) / THROTTLE_DESIGN_VELOCITY;
  return Math.sqrt((4 * area) / Math.PI);
}

export class IntakePlenum {
  /** Gas mass held in the plenum, kg. */
  private mass = 0;
  /**
   * Sensible internal energy, J: `mass * gasEnergy(T)`. State variable, for the reason given
   * in `Cylinder`.
   */
  private energy = 0;
  /** Of `mass`, how much is burned gas, kg. */
  private burnedMass = 0;
  /** Of `mass`, how much is unburned fuel, kg. */
  private fuelMass = 0;
  private volume = 0;
  /**
   * Commanded throttle area, m^2, cached.
   *
   * Recomputed when the spec changes, which is the only thing it depends on.
   */
  private area = 0;

  constructor(spec: EngineSpec) {
    this.volume = Math.max(plenumVolumeOf(spec), 1e-5);
    this.area = IntakePlenum.throttleArea(spec);
    this.mass = (GAS.pAmb * this.volume) / (GAS.R * GAS.tAmb);
    this.energy = this.mass * gasEnergy(GAS.tAmb);
  }

  /**
   * Set the throttle to `opening`, 0..1, in place of the spec's: what a dyno run drives it with.
   * Until the next `setGeometry`, which goes back to the spec's.
   */
  setOpening(spec: EngineSpec, opening: number): void {
    this.area = IntakePlenum.throttleAreaAt(spec, opening);
  }

  /** Rebuild geometry in place, keeping the gas state, so an edit does not click. */
  setGeometry(spec: EngineSpec): void {
    this.area = IntakePlenum.throttleArea(spec);
    const v = Math.max(plenumVolumeOf(spec), 1e-5);
    if (v === this.volume) return;
    // Scale the contents with the volume so pressure and temperature are continuous.
    const scale = v / this.volume;
    this.volume = v;
    this.mass *= scale;
    this.energy *= scale;
    this.burnedMass *= scale;
    this.fuelMass *= scale;
  }

  get temp(): number {
    return clamp(gasTemperature(this.energy / Math.max(this.mass, MIN_MASS)), 150, 3000);
  }

  /** Absolute pressure, Pa. */
  get pressure(): number {
    return (Math.max(this.mass, MIN_MASS) * GAS.R * this.temp) / this.volume;
  }

  /** Fraction of the contents that is burned gas, 0..1. */
  get burnedFraction(): number {
    return clamp(this.burnedMass / Math.max(this.mass, MIN_MASS), 0, 1);
  }

  /** Fraction of the contents that is unburned fuel, 0..1. */
  get fuelFraction(): number {
    return clamp(this.fuelMass / Math.max(this.mass, MIN_MASS), 0, 1);
  }

  /**
   * Effective throttle flow area, m^2 — geometric area already multiplied by the plate's
   * discharge coefficient, since both vary with opening and only the product matters here.
   *
   * `spec.throttle` is the plate position, and nothing is interposed between it and the
   * physics. That has a consequence worth knowing rather than hiding: a butterfly uncovers
   * area as `1 - cos(angle)` and its discharge coefficient climbs with opening too, so
   * effective area goes roughly as the square of that — and a throttle sized correctly for
   * peak rpm is much larger than the engine needs at part speed. On the 500 cc single at
   * 3200 rpm the top half of the travel therefore does very little, because by then the engine
   * is already unthrottled. Real engines behave the same way, which is why road vehicles put a
   * pedal map in front of the plate; that map is not modelled here, so the control is the bare
   * hardware.
   */
  static throttleArea(spec: EngineSpec): number {
    return IntakePlenum.throttleAreaAt(spec, spec.throttle);
  }

  /** `throttleArea` at plate position `opening`, 0..1, rather than the spec's. */
  static throttleAreaAt(spec: EngineSpec, opening: number): number {
    const d = throttleDiaOf(spec);
    const bore = (Math.PI * d * d) / 4;
    const open = 1 - Math.cos(clamp(opening, 0, 1) * (Math.PI / 2));
    const geometric = bore * (IDLE_BYPASS + (1 - IDLE_BYPASS) * open);
    return geometric * (CD_CLOSED + (CD_OPEN - CD_CLOSED) * open);
  }

  /**
   * Advance by `dt`.
   *
   * @param valveFlow Net mass flow to the cylinders, kg/s, positive out of the plenum.
   * @param backflow Of that, the part flowing back *in*, kg/s, 0 or more; `backflowTemp`,
   *   `backflowBurned` and `backflowFuel` describe it.
   *
   * The two are separate because on a multi-cylinder engine the net hides the back-flow: one
   * cylinder spitting exhaust up its runner while another draws is a small positive net, and taken
   * as that alone the spat gas would vanish and be replaced, in the bookkeeping, by clean plenum air.
   * The next cylinder would then burn it. That is harmless with a mild cam and badly wrong with a big
   * one, which reverts a lot at idle: an overcammed V8 would trap two and a half times the fresh
   * charge its throttle could pass.
   */
  step(
    dt: number,
    valveFlow: number,
    backflow: number,
    backflowTemp: number,
    backflowBurned: number,
    backflowFuel: number,
  ): void {
    const p = this.pressure;
    const t = this.temp;
    const burned = this.burnedFraction;
    const fuel = this.fuelFraction;

    // --- Throttle ---------------------------------------------------------------
    // Signed, because a plenum above ambient does blow back out past the throttle.
    // Cd of 1 here: `throttleArea` has already applied the plate's own, which unlike a
    // poppet valve's is a strong function of how far open it is.
    const area = this.area;
    // The orifice takes the upstream gas's gamma: ambient air coming in, the plenum's own
    // contents blowing back out.
    const throttleFlow =
      p < GAS.pAmb
        ? orificeMassFlow(area, 1, GAS.pAmb, GAS.tAmb, p, GAS.gammaAir)
        : -orificeMassFlow(area, 1, p, t, GAS.pAmb, gasGamma(t));

    // --- Energy and mass --------------------------------------------------------
    // dU/dt = (enthalpy in) - (enthalpy out). No moving boundary, so no p dV work.
    const hOwn = gasEnthalpy(t);
    const hThrottle =
      throttleFlow >= 0 ? throttleFlow * gasEnthalpy(GAS.tAmb) : throttleFlow * hOwn;
    // What the cylinders drew leaves at plenum temperature; what they spat arrives at its own.
    const drawn = valveFlow + backflow;
    const hValve = -drawn * hOwn + backflow * gasEnthalpy(backflowTemp);

    this.energy += (hThrottle + hValve) * dt;
    this.mass += (throttleFlow - valveFlow) * dt;

    // Composition. Fresh air enters past the throttle carrying no burned gas; back-flow
    // arrives carrying the cylinder's.
    const burnedIn = -drawn * burned + backflow * backflowBurned;
    this.burnedMass += burnedIn * dt;
    // Fuel arrives only with the back-flow, and leaves with whatever is drawn or blows back out.
    const fuelThrottle = throttleFlow >= 0 ? 0 : throttleFlow * fuel;
    this.fuelMass += (fuelThrottle - drawn * fuel + backflow * backflowFuel) * dt;

    if (this.mass < MIN_MASS) {
      this.mass = MIN_MASS;
      this.energy = MIN_MASS * gasEnergy(Math.max(t, 150));
    }
    this.burnedMass = clamp(this.burnedMass, 0, this.mass);
    this.fuelMass = clamp(this.fuelMass, 0, this.mass - this.burnedMass);

    const eMin = this.mass * gasEnergy(150);
    if (this.energy < eMin) this.energy = eMin;
    if (!Number.isFinite(this.energy) || !Number.isFinite(this.mass)) this.reset();
  }

  reset(): void {
    this.mass = (GAS.pAmb * this.volume) / (GAS.R * GAS.tAmb);
    this.energy = this.mass * gasEnergy(GAS.tAmb);
    this.burnedMass = 0;
    this.fuelMass = 0;
  }
}
