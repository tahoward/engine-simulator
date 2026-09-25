import { describe, expect, it } from 'vitest';
import {
  DEFAULT_ENGINE,
  GAS,
  clearanceVolume,
  crankState,
  cylinderVolume,
  d2PistonDTheta2,
  dPistonDTheta,
  dVolumeDTheta,
  displacement,
  makeCrankState,
  pistonPosition,
  type EngineSpec,
} from '../src/model/spec.js';
import { Cylinder, wiebe } from '../src/audio/worklet/cylinder.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';
import { PIPE_PRESETS, defaultConfig } from '../src/model/spec.js';
import { orificeMassFlow, valveFlowArea, valveLift } from '../src/audio/worklet/valve.js';

const FS = 48000;

/** Motor the cylinder from `fromDeg` to `toDeg` with both valves shut. */
function motor(spec: EngineSpec, cyl: Cylinder, toDeg: number, rpm: number): void {
  const omega = (rpm * 2 * Math.PI) / 60;
  const dt = 1 / FS;
  // Sub-step finely; this test is about the thermodynamics, not the integrator.
  const sub = 8;
  let guard = 0;
  while (cyl.angle < toDeg && guard++ < 4e6) {
    for (let s = 0; s < sub; s++) cyl.advance(spec, dt / sub, omega, 0, 0, GAS.tAmb, 900);
  }
}

describe('cylinder geometry', () => {
  const spec = DEFAULT_ENGINE;

  it('volume is clearance at TDC and clearance + displacement at BDC', () => {
    expect(cylinderVolume(spec, 0)).toBeCloseTo(clearanceVolume(spec), 10);
    expect(cylinderVolume(spec, 180)).toBeCloseTo(
      clearanceVolume(spec) + displacement(spec),
      10,
    );
  });

  it('realises the stated compression ratio', () => {
    const ratio = cylinderVolume(spec, 180) / cylinderVolume(spec, 0);
    expect(ratio).toBeCloseTo(spec.compressionRatio, 6);
  });

  it('dV/dtheta matches a numerical derivative', () => {
    // The energy equation leans on this analytically, so a mismatch would quietly
    // corrupt every pressure the model produces.
    const h = 1e-4;
    for (const deg of [10, 45, 90, 135, 200, 270, 350]) {
      const numeric =
        ((cylinderVolume(spec, deg + h) - cylinderVolume(spec, deg - h)) / (2 * h)) *
        (180 / Math.PI);
      expect(dVolumeDTheta(spec, deg)).toBeCloseTo(numeric, 8);
    }
  });

  it('displacement is about 498 cc for the default bore and stroke', () => {
    expect(displacement(spec) * 1e6).toBeGreaterThan(490);
    expect(displacement(spec) * 1e6).toBeLessThan(505);
  });
});

describe('motored compression', () => {
  const spec: EngineSpec = { ...DEFAULT_ENGINE };

  /**
   * Start just *after* intake valve closing (576 deg). Crossing IVC is what arms
   * combustion for the cycle, so beginning at 580 guarantees a genuinely motored
   * compression with no heat release.
   */
  const START = 580;

  function atStart(): Cylinder {
    const cyl = new Cylinder(spec, START);
    cyl.temp = 500; // above the 450 K wall, so heat leaves the gas
    cyl.mass = (GAS.pAmb * cylinderVolume(spec, START)) / (GAS.R * cyl.temp);
    return cyl;
  }

  it('conserves mass with the valves shut', () => {
    const cyl = atStart();
    const m0 = cyl.mass;
    motor(spec, cyl, 719, 3000);
    expect(cyl.mass).toBeCloseTo(m0, 12);
  });

  it('peak pressure approaches but stays under the isentropic value', () => {
    const cyl = atStart();
    const p1 = cyl.pressure(spec);
    const v1 = cylinderVolume(spec, START);

    motor(spec, cyl, 719.5, 3000);
    const p2 = cyl.pressure(spec);
    const v2 = cylinderVolume(spec, cyl.angle);

    const isentropic = p1 * Math.pow(v1 / v2, GAS.gammaCyl);
    const fraction = p2 / isentropic;
    // Woschni wall heat transfer removes a few percent of the compression work.
    // Anything above 1.0 would mean the energy balance is creating energy.
    expect(fraction).toBeGreaterThan(0.85);
    expect(fraction).toBeLessThan(1.0);
  });

  it('behaves as a polytropic process with a realistic exponent', () => {
    const cyl = atStart();
    const p1 = cyl.pressure(spec);
    const v1 = cylinderVolume(spec, START);

    motor(spec, cyl, 719.5, 3000);
    const p2 = cyl.pressure(spec);
    const v2 = cylinderVolume(spec, cyl.angle);

    const n = Math.log(p2 / p1) / Math.log(v1 / v2);
    // Real engines measure 1.30-1.35 on the compression line.
    expect(n).toBeGreaterThan(1.28);
    expect(n).toBeLessThan(1.36);
  });

  it('does not arm combustion when the intake valve closing is never crossed', () => {
    const cyl = atStart();
    motor(spec, cyl, 719.5, 3000);
    expect(cyl.burned).toBe(0);
  });
});

describe('wiebe combustion', () => {
  it('is monotonic, starts at zero and finishes burnt', () => {
    expect(wiebe(-5, 50)).toBe(0);
    expect(wiebe(0, 50)).toBe(0);
    expect(wiebe(50, 50)).toBe(1);
    expect(wiebe(200, 50)).toBe(1);

    let prev = 0;
    for (let d = 0; d <= 50; d += 0.5) {
      const x = wiebe(d, 50);
      expect(x).toBeGreaterThanOrEqual(prev);
      prev = x;
    }
  });

  it('releases most of the heat in the middle of the window', () => {
    // a=5, m=2 gives the familiar S-curve rather than a linear ramp.
    expect(wiebe(25, 50)).toBeGreaterThan(0.3);
    expect(wiebe(25, 50)).toBeLessThan(0.7);
  });
});

describe('valve flow', () => {
  const spec = DEFAULT_ENGINE;

  it('lift is zero outside the window and peaks inside it', () => {
    // The exhaust window wraps past 720, which is the case most likely to be broken.
    expect(valveLift(100, spec.evo, spec.evc, spec.maxLift)).toBe(0);
    expect(valveLift(spec.evo - 1, spec.evo, spec.evc, spec.maxLift)).toBe(0);

    const mid = (spec.evo + spec.evc) / 2;
    expect(valveLift(mid, spec.evo, spec.evc, spec.maxLift)).toBeCloseTo(spec.maxLift, 6);

    // Overlap: the exhaust valve is still open a crack just after TDC.
    expect(valveLift(370, spec.evo, spec.evc, spec.maxLift)).toBeGreaterThan(0);
    expect(valveLift(380, spec.evo, spec.evc, spec.maxLift)).toBe(0);
  });

  it('lift ramps smoothly, with no step at the seat', () => {
    // A discontinuity here would inject a broadband click every cycle.
    let prev = 0;
    let maxJump = 0;
    for (let d = spec.evo - 2; d < spec.evc + 2; d += 0.1) {
      const l = valveLift(d, spec.evo, spec.evc, spec.maxLift);
      maxJump = Math.max(maxJump, Math.abs(l - prev));
      prev = l;
    }
    expect(maxJump).toBeLessThan(spec.maxLift * 0.02);
  });

  it('flow area saturates at the port throat', () => {
    const dia = 0.034;
    const small = valveFlowArea(0.001, dia);
    const large = valveFlowArea(0.05, dia);
    // Curtain area at 1 mm lift.
    expect(small).toBeCloseTo(Math.PI * dia * 0.001 * Math.cos(Math.PI / 4), 9);
    // Way past the crossover, the throat rules and more lift buys nothing.
    expect(large).toBeCloseTo(valveFlowArea(0.1, dia), 12);
  });

  it('orifice flow chokes and then stops responding to downstream pressure', () => {
    const area = 5e-4;
    const pUp = 5e5;
    const t = 1200;
    const g = GAS.gammaCyl;
    const atCritical = orificeMassFlow(area, 0.72, pUp, t, pUp * 0.53, g);
    const wellBelow = orificeMassFlow(area, 0.72, pUp, t, 1e3, g);
    expect(wellBelow).toBeCloseTo(atCritical, 6);

    // Unchoked: less pressure drop means less flow.
    const mild = orificeMassFlow(area, 0.72, pUp, t, pUp * 0.9, g);
    expect(mild).toBeLessThan(atCritical);
    expect(mild).toBeGreaterThan(0);

    // No flow uphill or through a shut valve.
    expect(orificeMassFlow(area, 0.72, pUp, t, pUp * 1.1, g)).toBe(0);
    expect(orificeMassFlow(0, 0.72, pUp, t, 1e3, g)).toBe(0);
  });

  it('peak blowdown empties the cylinder on a plausible timescale', () => {
    // Sanity check on absolute magnitude: at exhaust-valve-opening conditions the
    // charge should dump in a couple of milliseconds, not microseconds or seconds.
    const area = valveFlowArea(spec.maxLift * 0.3, spec.exValveDia);
    const mdot = orificeMassFlow(area, 0.72, 5e5, 1200, GAS.pAmb, GAS.gammaCyl);
    const trapped = (5e5 * cylinderVolume(spec, 130)) / (GAS.R * 1200);
    const emptyingTime = trapped / mdot;
    expect(emptyingTime).toBeGreaterThan(5e-4);
    expect(emptyingTime).toBeLessThan(2e-2);
  });
});

describe('the gas state stays admissible while the cylinder empties', () => {
  /**
   * The exhaust stroke is the hard case for a filling-and-emptying model, and it is where
   * this used to fail.
   *
   * Compression work `-p dV/dt` and outflow enthalpy nearly cancel while gas is being pushed
   * out — exactly, at constant pressure, which is why temperature should hold steady. With
   * *temperature* as the state variable that near-zero residual gets divided by a mass
   * shrinking toward the residual, and it blew up: at part throttle the temperature ran to
   * its clamp several hundred times a second right at exhaust valve closing, with the
   * cylinder drained to 0.5% of a charge. Integrating internal energy instead keeps the
   * cancellation between terms of the same size.
   */
  it('emptying the cylinder converges under time refinement', () => {
    // The integrator tested directly, rather than against an analytic answer — wall heat
    // transfer is always active and legitimately changes the temperature, so a closed-form
    // target would be measuring the physics, not the scheme.
    //
    // Drive the exact quasi-steady constant-pressure outflow (from pV = mRT at fixed p and
    // T, mass tracks p V/(R T), so mdot = -(p/(R T)) dV/dt) and compare coarse against fine
    // steps. A sound integrator gives nearly the same answer; the old one, dividing a
    // near-cancelling residual by a vanishing mass, diverged as the cylinder emptied.
    const spec: EngineSpec = { ...DEFAULT_ENGINE };
    const omega = (3200 * 2 * Math.PI) / 60;

    const emptyOut = (subdiv: number) => {
      const cyl = new Cylinder(spec, 250); // mid exhaust stroke
      cyl.temp = 1100;
      cyl.mass = (GAS.pAmb * cylinderVolume(spec, 250)) / (GAS.R * 1100);
      const m0 = cyl.mass;
      const dt = 1 / (FS * subdiv);
      while (cyl.angle < 352) {
        const dVdt = dVolumeDTheta(spec, cyl.angle) * omega;
        cyl.advance(spec, dt, omega, -(GAS.pAmb / (GAS.R * cyl.temp)) * dVdt, 0, GAS.tAmb, 900);
      }
      return { temp: cyl.temp, massRatio: cyl.mass / m0, clamps: cyl.clampHits };
    };

    const coarse = emptyOut(1);
    const fine = emptyOut(16);

    // It really did empty substantially — about half the charge leaves over this stroke,
    // which is the regime that used to break. (Not more: dV/dtheta tapers toward TDC, so
    // the constant-pressure outflow tapers with it.)
    expect(fine.massRatio).toBeLessThan(0.6);
    expect(Math.abs(coarse.temp - fine.temp) / fine.temp).toBeLessThan(0.05);
    expect(coarse.clamps).toBe(0);
    expect(fine.clamps).toBe(0);
    expect(Number.isFinite(coarse.temp)).toBe(true);
  });

  it('never clamps at any steady operating point', () => {
    // The clamp truncates energy, so a hit means the integration has left the physics
    // behind. It used to fire hundreds of times a second below about 0.5 throttle while
    // staying silent at full load, which is what made it easy to miss.
    for (const throttle of [1, 0.75, 0.45, 0.3, 0.2, 0.1]) {
      const cfg = defaultConfig();
      cfg.engine = { ...cfg.engine, throttle, rpm: 3200 };
      cfg.pipe = PIPE_PRESETS[1]!.build();
      const sim = new EngineSim(FS, cfg);
      sim.render(FS * 2);
      const before = sim.cylinder.clampHits;
      sim.render(FS);
      expect(sim.cylinder.clampHits - before, `throttle ${throttle}`).toBe(0);
    }
  });

  it('reverse flow through the exhaust valve arrives at port temperature', () => {
    // During overlap the pipe can push gas back into the cylinder. Treating it as arriving
    // at *cylinder* temperature imports heat that was never there.
    const spec: EngineSpec = { ...DEFAULT_ENGINE };
    const hot = new Cylinder(spec, 400);
    const cold = new Cylinder(spec, 400);
    for (const c of [hot, cold]) {
      c.temp = 1200;
      c.mass = (GAS.pAmb * cylinderVolume(spec, 400)) / (GAS.R * 1200);
    }
    const omega = (3200 * 2 * Math.PI) / 60;
    // Same reverse flow, different port temperatures.
    for (let k = 0; k < 400; k++) {
      hot.advance(spec, 1 / FS / 8, omega, -0.01, 0, GAS.tAmb, 1400);
      cold.advance(spec, 1 / FS / 8, omega, -0.01, 0, GAS.tAmb, 500);
    }
    expect(hot.temp).toBeGreaterThan(cold.temp + 20);
  });

  it('free-running wind-down from full throttle to shut stays clean', () => {
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, freeRunning: true, throttle: 1, loadTorque: 14 };
    cfg.pipe = PIPE_PRESETS[1]!.build();
    const sim = new EngineSim(FS, cfg);
    sim.render(FS * 4);
    sim.setEngine({ throttle: 0 });

    const hits0 = sim.cylinder.clampHits;
    const buf = sim.render(FS * 4);
    for (const v of buf) {
      expect(Number.isFinite(v)).toBe(true);
      expect(Math.abs(v)).toBeLessThan(1);
    }
    expect(sim.cylinder.clampHits - hits0).toBe(0);
    expect(sim.pipeSolver.recoveries).toBe(0);
    // And it should actually have slowed down.
    expect(sim.rpm).toBeLessThan(7000);
  });
});

describe('consolidated crank state', () => {
  /**
   * `crankState` exists only to compute in one pass what the four individual functions compute
   * in four, saving five sin/cos/sqrt triples per substep. That makes it a second copy of the
   * same algebra, so it has to be pinned to the first copy — otherwise a future correction to
   * one of them silently applies to the renderer and not to the physics, or the reverse.
   */
  it('agrees with the individual functions across the whole cycle', () => {
    const spec = { ...DEFAULT_ENGINE };
    const out = makeCrankState();
    for (let deg = 0; deg < 720; deg += 0.5) {
      crankState(spec, deg, out);
      expect(out.position).toBeCloseTo(pistonPosition(spec, deg), 12);
      expect(out.dPosition).toBeCloseTo(dPistonDTheta(spec, deg), 12);
      expect(out.d2Position).toBeCloseTo(d2PistonDTheta2(spec, deg), 10);
      expect(out.volume).toBeCloseTo(cylinderVolume(spec, deg), 12);
      expect(out.dVolume).toBeCloseTo(dVolumeDTheta(spec, deg), 12);
    }
  });

  it('agrees for extreme geometry too', () => {
    // A very short rod exaggerates the obliquity terms, where a sign slip would hide at
    // ordinary proportions.
    const out = makeCrankState();
    for (const spec of [
      { ...DEFAULT_ENGINE, stroke: 0.12, rodLength: 0.125 },
      { ...DEFAULT_ENGINE, stroke: 0.04, rodLength: 0.3 },
    ]) {
      for (let deg = 0; deg < 720; deg += 3) {
        crankState(spec, deg, out);
        expect(out.position).toBeCloseTo(pistonPosition(spec, deg), 12);
        expect(out.dPosition).toBeCloseTo(dPistonDTheta(spec, deg), 12);
        expect(out.d2Position).toBeCloseTo(d2PistonDTheta2(spec, deg), 9);
        expect(out.dVolume).toBeCloseTo(dVolumeDTheta(spec, deg), 12);
      }
    }
  });

  it('reuses the object it is given', () => {
    const out = makeCrankState();
    expect(crankState({ ...DEFAULT_ENGINE }, 90, out)).toBe(out);
  });
});
