/**
 * The exhaust as a whole: an `ExhaustGraph` of ducts joined at junctions, with one duct fed by
 * each cylinder's exhaust valve.
 *
 * Any number of ducts meeting at any number of junctions, all built from the same `EulerPipe` and
 * all marched in lockstep. The shapes that come up most:
 *
 *   open        cyl ───── pipe ─────▸ mouth          (one per cylinder, no interaction)
 *
 *   collector   cyl1 ── primary ─┐
 *               cyl2 ── primary ─┼── collector ──▸ mouth      (`compileCollectorLayout`)
 *               ...             ─┘
 *
 *   manifold    cyl1 ── stub ── link ─┬─ link ─┬─ link ─┬── collector ──▸ mouth
 *               cyl2 ── stub ─────────┘        │        │
 *               cyl3 ── stub ──────────────────┘        │      (`compileLayout`: one per
 *               cyl4 ── stub ───────────────────────────┘       bank, or per side of a V)
 *
 * A junction is the interesting case, and the reason a V-twin with a shared header sounds
 * unlike two singles: each cylinder's blowdown pulse arrives at the junction and partly travels
 * *up the other primaries*, where it can either help scavenge those cylinders or block them,
 * depending on where the firing interval puts it. That cross-talk is a real tuning mechanism
 * and it falls out of the junction rather than being imposed. It is also what makes a
 * crossplane V8 sound unlike a flatplane one: identical firing intervals overall, but each
 * bank's collector sees a different pattern of arrivals.
 */

import {
  GAS,
  ambientSoundSpeed,
} from '../../model/spec.js';
import {
  endsAt,
  nodeOrder,
  pathToAir,
  radiatingDucts,
  validateGraph,
  valveDucts,
  type ExhaustDuct,
  type ExhaustGraph,
} from '../../model/exhaustGraph.js';
import { Noise, clamp, hypot2 } from './dsp.js';
import {
  DESIGN_WAVE_SPEED,
  EulerPipe,
  MIN_JUNCTION_P,
  MIN_JUNCTION_RHO,
  type EndState,
  type EulerPipeOptions,
  type ValveState,
} from './eulerPipe.js';
import { JunctionKernel } from './kernel.js';

/**
 * Turbulence intensity of the merge, as a fraction of the mixing mass flow.
 *
 * A collector is a violent shear layer: several pulsating streams, each arriving at a different
 * speed and at a different moment, forced into one pipe. That mixing is a broadband noise source
 * and a large part of what a real 4-into-1 sounds like. Left out, it is audible in exactly the way
 * you would expect — merged layouts come out *purer* than separate pipes: a tone-to-noise ratio of
 * 18.0 dB for a 2-into-1 twin against 5.9 dB for the same engine with two separate pipes, and
 * 17.4 dB for an inline four. A model that makes a collector *cleaner* than an open pipe has it
 * exactly backwards.
 *
 * This matters more the more cylinders there are, which is why multi-cylinder engines are the ones
 * that sound synthetic without it: their firing energy adds coherently, concentrating into fewer and
 * stronger spectral lines, while what little other broadband content the model has adds
 * incoherently. A V8 would come out about 9 dB more tonal than a single for no physical reason.
 */
const MERGE_TURBULENCE = 0.14;

export type { ExhaustLayout } from '../../model/spec.js';

/**
 * Result of one `advance`. **Reused between calls** — read it before advancing again.
 *
 * This is on the audio thread's per-sample path, so a fresh result object and three fresh
 * arrays 48,000 times a second is not free: the allocation and the garbage it would make
 * measure at 17% of total CPU, for arrays of two or three numbers. Port pressure is not here at all,
 * because nothing reads it; `primaries[b].portPressure` is there for anyone who needs it.
 */
export interface ExhaustResult {
  /** Volume flow out of each radiating mouth, m^3/s. */
  mouthFlows: Float64Array;
  /** Mass flow through each cylinder's exhaust valve, kg/s, positive out of the cylinder. */
  valveMassFlows: Float64Array;
  substeps: number;
}

/** Specific heat at constant pressure for exhaust gas, J/(kg K). */
const CP_EXH = (GAS.gammaExh * GAS.R) / (GAS.gammaExh - 1);

/** Relative mass-flux imbalance below which a junction is left alone. */
const JUNCTION_BALANCE_TOL = 0.005;

/**
 * One junction, with every duct end that meets there.
 *
 * Held as two lists rather than one list of tagged ends, so `endState` and `probeJunction` are called
 * with *literal* sides on the per-substep path rather than a variable string, and each end costs one
 * property load fewer. Worth about a point on a V8, measured.
 *
 * Outlets are always visited before inlets. That is not cosmetic: the common-pressure sum and the mass
 * residual are accumulated across ends, and floating-point addition is not associative, so the visiting
 * order is part of the result — the wasm junction solve takes its branches in the same order.
 */
interface JunctionNode {
  id: string;
  /** Ducts emptying into the node — their outlets. */
  outlets: EulerPipe[];
  /** Ducts leaving the node — their inlets. */
  inlets: EulerPipe[];
  /** Reused end states, outlets first then inlets, so the per-substep solve allocates nothing. */
  states: EndState[];
  /** Ducts leaving the node, and the share of the mixing noise each takes. */
  downstream: Array<{ duct: number; share: number }>;
  noise: Noise;
  lp1: number;
  lp2: number;
}

export class ExhaustSystem {
  /** One per cylinder: the duct its exhaust valve feeds. */
  readonly primaries: EulerPipe[];
  /** Ducts fed by a junction rather than a valve, in node order. */
  readonly collectors: EulerPipe[];
  /** What `collector` returns: see there. */
  private readonly mainCollector: EulerPipe | null;
  /** Cylinder 0's ducts from its valve to open air, in order. What `quarterWaveHz` adds up. */
  private readonly airPath: EulerPipe[];

  /** Ducts that radiate, in the order their flows are reported. */
  private readonly radiating: EulerPipe[];

  // Everything below is preallocated because `advance` runs once per audio sample.
  /** Every duct, primaries first. Marched in this order. */
  private readonly ducts: EulerPipe[];
  /** For each duct, its index in `radiating`, or -1. Saves an `indexOf` per substep. */
  private readonly radiatingIndex: Int32Array;
  private readonly result: ExhaustResult;
  /** The junctions, resolved at construction: the solve runs per substep. */
  private readonly nodes: JunctionNode[];
  /**
   * The junction solve in wasm, or `null` to solve in TypeScript — where wasm is unavailable, a junction
   * has more branches than the kernel takes, or the caller asked (`useJunctionKernel: false`, as the
   * tests do to compare the two).
   */
  private readonly junctionKernel: JunctionKernel | null;
  /**
   * The ducts a junction feeds, with the mass source going into each one's first cell.
   *
   * A compact list rather than one entry per duct indexed alongside `ducts`, which would hold `null`
   * for every valve-fed duct, making the array holey and giving `endStep` a polymorphic argument on
   * the per-substep path for no gain.
   */
  private readonly fedByNode: Array<{ pipe: EulerPipe; valve: ValveState }>;
  private readonly fedFlow: Float64Array;
  /** A fed duct's inlet pressure, temperature and area, from `EulerPipe.readPort`. */
  private readonly portState = new Float64Array(3);
  /**
   * Duration of the current substep, s, and the square root of the substep count. The merge noise
   * is advanced once per substep, so its filter is discretised on the substep and its white-noise
   * draws are scaled so the noise density does not depend on how many substeps a sample takes.
   */
  private substepDt = 0;
  private substepNoiseScale = 1;
  /**
   * Worst relative mass-flux imbalance a junction's per-duct solves produced, dimensionless.
   *
   * Diagnostic only: nothing is corrected from it. The node pressure is Newton-corrected toward
   * balance (see `solveJunctionsTs`), but the fluxes committed are whatever the per-duct solves give
   * at that pressure, so this is the imbalance that remains. It is kept because it measures how far
   * the common-pressure solve is from agreeing with itself, which is a useful warning that a
   * junction is being driven outside its range.
   */
  junctionResidual = 0;
  /** 0..1. Scales the merge turbulence; driven by the same control as throat noise. */
  private turbulence = 1;

  /**
   * Build one `EulerPipe` per duct in the graph and wire the junctions.
   *
   * `EulerPipe` takes `inletKind: 'valve' | 'junction'` and `outletKind: 'mouth' | 'junction'`, so any
   * duct in any graph is expressible as one; all this adds is the wiring.
   */
  constructor(
    graph: ExhaustGraph,
    cylinders: number,
    sampleRate: number,
    portGasTemp: number,
    opts: EulerPipeOptions,
  ) {
    const problems = validateGraph(graph, cylinders);
    if (problems.length > 0) {
      throw new Error(`exhaust graph cannot be solved: ${problems.join('; ')}`);
    }

    const order = nodeOrder(graph);
    const built = new Map<string, EulerPipe>();
    /** Ducts in build order: valve-fed first, then node-fed. */
    const ductList: ExhaustDuct[] = [];

    const valveFed = valveDucts(graph, cylinders);
    this.primaries = valveFed.map((duct) => {
      const pipe = new EulerPipe(duct!.segments, sampleRate, portGasTemp, {
        ...opts,
        inletKind: 'valve',
        outletKind: duct!.to.kind === 'mouth' ? 'mouth' : 'junction',
        inheritWall: opts.inheritWall,
      });
      built.set(duct!.id, pipe);
      ductList.push(duct!);
      return pipe;
    });

    /**
     * Node-fed ducts, upstream-first.
     *
     * A duct's inlet must not be a severe contraction on the pipes feeding it — four pipes cannot
     * empty into one pipe's worth of area, and a junction asked to do that drives the cell behind it
     * toward vacuum (see `JUNCTION_INLET_FRACTION` in eulerPipe.ts). Working out that feeding area
     * needs the upstream ducts to exist already, so they are built in topological order. A graph with
     * a loop in it cannot be ordered; those ducts are built last with no feeding area rather than
     * hanging, since a loop is a drawing mistake rather than something to solve.
     */
    const pending = graph.ducts.filter((d) => d.from.kind === 'node');
    this.collectors = [];
    const nodeFed: ExhaustDuct[] = [];
    for (let pass = 0; pass < pending.length + 1 && pending.length > 0; pass++) {
      let madeProgress = false;
      for (let i = 0; i < pending.length; i++) {
        const duct = pending[i]!;
        const node = (duct.from as { kind: 'node'; node: string }).node;
        const upstream = endsAt(graph, node).filter((e) => e.end === 'outlet');
        if (!upstream.every((e) => built.has(e.duct.id))) continue;
        let feedArea = 0;
        for (const e of upstream) feedArea += built.get(e.duct.id)!.outletArea;
        const pipe = new EulerPipe(duct.segments, sampleRate, portGasTemp, {
          ...opts,
          inletKind: 'junction',
          outletKind: duct.to.kind === 'mouth' ? 'mouth' : 'junction',
          // No head port on anything downstream of a junction; the port belongs to each runner.
          port: undefined,
          junctionInletArea: feedArea,
        });
        built.set(duct.id, pipe);
        nodeFed.push(duct);
        pending.splice(i, 1);
        i--;
        madeProgress = true;
      }
      if (!madeProgress) break;
    }
    for (const duct of pending) {
      const pipe = new EulerPipe(duct.segments, sampleRate, portGasTemp, {
        ...opts,
        inletKind: 'junction',
        outletKind: duct.to.kind === 'mouth' ? 'mouth' : 'junction',
        port: undefined,
      });
      built.set(duct.id, pipe);
      nodeFed.push(duct);
    }
    // Back into node order, so the noise seeds and the collector readouts are stable.
    nodeFed.sort((a, b) => {
      const an = order.indexOf((a.from as { node: string }).node);
      const bn = order.indexOf((b.from as { node: string }).node);
      return an - bn;
    });
    for (const duct of nodeFed) {
      this.collectors.push(built.get(duct.id)!);
      ductList.push(duct);
    }

    this.ducts = ductList.map((d) => built.get(d.id)!);
    const collectorDuct = nodeFed.find((d) => d.role === 'collector');
    this.mainCollector = collectorDuct ? built.get(collectorDuct.id)! : (this.collectors[0] ?? null);
    this.airPath = pathToAir(graph, 0).flatMap((d) => (built.has(d.id) ? [built.get(d.id)!] : []));
    const indexOfDuct = new Map<string, number>();
    ductList.forEach((d, i) => indexOfDuct.set(d.id, i));

    this.nodes = order.map((id, n) => {
      const all = endsAt(graph, id);
      const byDuct = (a: { duct: ExhaustDuct }, b: { duct: ExhaustDuct }) =>
        indexOfDuct.get(a.duct.id)! - indexOfDuct.get(b.duct.id)!;
      const outletEnds = all.filter((e) => e.end === 'outlet').sort(byDuct);
      const inletEnds = all.filter((e) => e.end === 'inlet').sort(byDuct);
      let areaSum = 0;
      for (const e of inletEnds) areaSum += built.get(e.duct.id)!.inletArea;
      return {
        id,
        outlets: outletEnds.map((e) => built.get(e.duct.id)!),
        inlets: inletEnds.map((e) => built.get(e.duct.id)!),
        states: new Array<EndState>(all.length),
        downstream: inletEnds.map((e) => ({
          duct: indexOfDuct.get(e.duct.id)!,
          share: areaSum > 0 ? built.get(e.duct.id)!.inletArea / areaSum : 1 / inletEnds.length,
        })),
        noise: new Noise(0x7f4a3b + n * 0x9e3779b),
        lp1: 0,
        lp2: 0,
      };
    });

    const widest = this.nodes.reduce((m, node) => Math.max(m, node.outlets.length + node.inlets.length), 0);
    const kernel =
      (opts.useJunctionKernel ?? opts.useKernel) === false || this.nodes.length === 0
        ? null
        : JunctionKernel.create({
            gamma: GAS.gammaExh,
            pAmb: GAS.pAmb,
            r: GAS.R,
            tAmb: GAS.tAmb,
            cp: CP_EXH,
            minRho: MIN_JUNCTION_RHO,
            minP: MIN_JUNCTION_P,
            maxSpeed: DESIGN_WAVE_SPEED,
            ambientC: ambientSoundSpeed(),
            tol: JUNCTION_BALANCE_TOL,
          });
    this.junctionKernel = kernel && widest <= kernel.capacity ? kernel : null;

    this.radiating = radiatingDucts(graph).map((d) => built.get(d.id)!);
    this.radiatingIndex = new Int32Array(this.ducts.length);
    for (let i = 0; i < this.ducts.length; i++) {
      this.radiatingIndex[i] = this.radiating.indexOf(this.ducts[i]!);
    }
    this.result = {
      mouthFlows: new Float64Array(this.radiating.length),
      valveMassFlows: new Float64Array(this.primaries.length),
      substeps: 1,
    };

    this.fedByNode = [];
    const fedSlot = new Map<string, number>();
    ductList.forEach((duct, i) => {
      if (duct.from.kind !== 'node') return;
      fedSlot.set(duct.id, this.fedByNode.length);
      this.fedByNode.push({
        pipe: this.ducts[i]!,
        valve: {
          throatArea: 0,
          cylPressure: GAS.pAmb,
          cylTemp: portGasTemp,
          cylGamma: GAS.gammaExh,
          extraMassFlow: 0,
        },
      });
    });
    this.fedFlow = new Float64Array(this.fedByNode.length);
    for (const node of this.nodes) {
      for (const d of node.downstream) d.duct = fedSlot.get(ductList[d.duct]!.id)!;
    }
  }

  /** Turbulence scale, 0..1, shared with the valve throat noise. */
  setTurbulence(value: number): void {
    this.turbulence = clamp(value, 0, 4);
  }

  /**
   * The collector, or null if nothing merges. Convenience for the single-duct readouts.
   *
   * The first duct the graph calls a collector, in node order, which on a compiled manifold is the one
   * after the bank's chain of links rather than the first link. A drawn graph need not label its ducts,
   * so failing that it is the first junction-fed duct.
   */
  get collector(): EulerPipe | null {
    return this.mainCollector;
  }

  /** How many mouths radiate. */
  get mouthCount(): number {
    return this.radiating.length;
  }

  /** Total cells across every duct, for the load readout. */
  get cells(): number {
    let n = 0;
    for (const d of this.ducts) n += d.n;
    return n;
  }

  get recoveries(): number {
    let r = 0;
    for (const d of this.ducts) r += d.recoveries;
    return r;
  }

  /**
   * Advance every duct by `dt`, substepping on the tightest CFL limit among them.
   *
   * A single shared substep count is deliberate. The ducts are coupled through the junction,
   * so they have to march in lockstep — letting each pick its own step would mean solving the
   * junction between states at different times.
   */
  advance(dt: number, valves: ValveState[]): ExhaustResult {
    const ducts = this.ducts;
    const nDucts = ducts.length;
    const nPrimaries = this.primaries.length;
    const { mouthFlows, valveMassFlows } = this.result;

    let substeps = 1;
    for (let i = 0; i < nDucts; i++) {
      const s = ducts[i]!.substepsFor(dt);
      if (s > substeps) substeps = s;
    }

    const h = dt / substeps;
    this.substepDt = h;
    this.substepNoiseScale = Math.sqrt(substeps);
    mouthFlows.fill(0);
    valveMassFlows.fill(0);

    for (let k = 0; k < substeps; k++) {
      // 1. Reconstruct every duct, leaving the boundary faces unset.
      for (let i = 0; i < nDucts; i++) ducts[i]!.beginStep(h);

      // 2. Solve the junctions, if any, while every duct is mid-step.
      if (this.nodes.length > 0) this.solveJunctions();

      // 3. Each duct's own boundaries: valve walls and radiating mouths.
      for (let i = 0; i < nDucts; i++) {
        const flow = ducts[i]!.applyOwnBoundaries(h);
        const rIdx = this.radiatingIndex[i]!;
        if (rIdx >= 0) mouthFlows[rIdx]! += flow;
      }

      // 4. Valve flux and the conservative update.
      for (let b = 0; b < nPrimaries; b++) {
        const valve = valves[b]!;
        const primary = this.primaries[b]!;
        const flow = primary.valveFluxFor(valve);
        primary.setEndStep(h, flow + (valve.extraMassFlow ?? 0));
        primary.endStepSet(valve);
        // Only what the duct actually took: see `EulerPipe.sourceScale`.
        valveMassFlows[b]! += flow * primary.sourceScale;
      }
      // 5. Everything a junction feeds, carrying that junction's share of the mixing noise.
      const fed = this.fedByNode;
      for (let i = 0; i < fed.length; i++) {
        const pipe = fed[i]!.pipe;
        pipe.setEndStep(h, this.fedFlow[i]!);
        pipe.endStepSet(fed[i]!.valve);
      }
      for (let i = 0; i < nDucts; i++) ducts[i]!.afterStep(h);
    }

    // Last-resort recovery, per duct. On an audio thread a single NaN propagates through the
    // whole array and the app goes permanently silent, so an inadmissible state is reset
    // rather than left to poison everything. `recoveries` makes it visible.
    let broken = false;
    for (let i = 0; i < nDucts; i++) broken = ducts[i]!.recoverIfBroken() || broken;

    const inv = broken ? 0 : 1 / substeps;
    for (let i = 0; i < mouthFlows.length; i++) mouthFlows[i]! *= inv;
    for (let i = 0; i < valveMassFlows.length; i++) valveMassFlows[i]! *= inv;
    this.result.substeps = substeps;
    return this.result;
  }

  /**
   * Constant-pressure junction.
   *
   * Each duct end supplies the wave travelling toward the junction, `a_k`. Writing the wave
   * returning into it as `b_k = p_J - a_k`, the mass flow into the junction from that duct is
   * `A_k (2 a_k - p_J) / c_k`, so conservation gives the common pressure in closed form:
   *
   *     p_J = sum(2 A_k a_k / c_k) / sum(A_k / c_k)
   *
   * No iteration, and it reduces to the right thing for two ducts (a plain area change) and
   * for one (a rigid end). Note this linearises the *returning* wave, the same approximation
   * the radiating mouth makes — justified there because radiation is weak, and more of a
   * stretch here because both sides carry large pulses. The outgoing waves stay fully
   * nonlinear, and the alternative is an iterative Riemann solve per substep.
   */
  private solveJunctions(): void {
    const k = this.junctionKernel;
    if (k) {
      const f = k.fields;
      for (const node of this.nodes) {
        const { outlets, inlets, states } = node;
        const nOut = outlets.length;
        const nEnds = nOut + inlets.length;
        for (let i = 0; i < nOut; i++) states[i] = outlets[i]!.fillJunctionEnd('outlet', f, i);
        for (let i = 0; i < inlets.length; i++) {
          states[nOut + i] = inlets[i]!.fillJunctionEnd('inlet', f, nOut + i);
        }
        const rel = k.solve(nEnds, nOut);
        for (let i = 0; i < nOut; i++) outlets[i]!.takeJunctionFlux('outlet', f, i);
        for (let i = 0; i < inlets.length; i++) inlets[i]!.takeJunctionFlux('inlet', f, nOut + i);
        if (rel > this.junctionResidual) this.junctionResidual = rel;
        this.updateMergeNoise(node);
      }
      return;
    }
    this.solveJunctionsTs();
  }

  /** The same solve in TypeScript: the reference the wasm is checked against, and the fallback. */
  private solveJunctionsTs(): void {
    for (const node of this.nodes) {
      const { outlets, inlets, states } = node;
      const nOut = outlets.length;
      const nEnds = nOut + inlets.length;

      /**
       * Every end state, once.
       *
       * The node needs each branch's state five separate ways — the common-pressure sum, the pressure
       * bounds, the mixed node temperature, every trial Riemann solve, and the mixing noise.
       * Recomputing it each time would mean five square roots and five short-lived objects per duct
       * per substep, a steady drip of garbage onto the audio thread.
       * `endState` fills a preallocated object, so these are stable references and this only refreshes
       * their values.
       */
      for (let i = 0; i < nOut; i++) states[i] = outlets[i]!.endState('outlet');
      for (let i = 0; i < inlets.length; i++) states[nOut + i] = inlets[i]!.endState('inlet');

      let num = 0;
      let den = 0;
      let pMin = Infinity;
      let pMax = 0;
      let mIn = 0;
      let hIn = 0;
      let scaleGuess = 0;

      for (let i = 0; i < nEnds; i++) {
        const st = states[i]!;
        const w = st.area / st.c;
        num += 2 * w * st.toward;
        den += w;
        if (st.p < pMin) pMin = st.p;
        if (st.p > pMax) pMax = st.p;
        scaleGuess += Math.abs(st.rho * st.area * st.u);
        // Speed *into* the node: along +x at an outlet end, against it at an inlet end.
        const into = i < nOut ? st.u : -st.u;
        if (into > 0) {
          const m = st.rho * st.area * into;
          mIn += m;
          hIn += m * (st.p / (st.rho * GAS.R) + (into * into) / (2 * CP_EXH));
        }
      }

      /**
       * Temperature of the gas actually sitting in the node.
       *
       * A duct cannot build an *inflowing* ghost state from its own interior: that scales the
       * receiving duct's own entropy up to the junction pressure, so a collector inhales its own heat
       * back and amplifies it. Built that way, a V8 with a 0.2 m 42-to-130 mm cone collector at
       * 7000 rpm has the collector reach 42,000 K while every primary feeding it sits at 1568 K, its
       * inlet importing 2.46 MJ/s at 0.109 kg/s — an implied 19,500 K that no branch is supplying. So
       * the node gets a state of its own: the mass-weighted stagnation temperature of whatever is
       * emptying into it.
       */
      const fallback = states[states.length - 1]!;
      const tJunction =
        mIn > 1e-12 ? hIn / mIn : fallback.p / (Math.max(fallback.rho, 1e-7) * GAS.R);

      /**
       * The junction pressure, held inside the range its own branches can justify.
       *
       * The closed form is a *linearisation* of the returning wave, and with several branches carrying
       * large pulses it can solve for a pressure far outside anything any branch actually contains — at
       * which point the ghost state built from it is inadmissible and the duct fills with NaN. Unheld,
       * a four-into-one with a 53 mm collector on 29 mm primaries diverges within a tenth of a second
       * at full throttle yet survives at half throttle, which is the signature of an amplitude limit
       * rather than a coding error.
       */
      let gauge = clamp(GAS.pAmb + (den > 0 ? num / den : 0), 0.3 * pMin, 3 * pMax) - GAS.pAmb;

      /**
       * Then correct it until the branches actually balance.
       *
       * A node has no volume, so what flows in is exactly what flows out — and nothing in the per-duct
       * solves enforces that. Uncorrected, the branches disagree about the mass crossing the node by
       * 18-36% at peak, on every layout including a plain-pipe collector and a healthy V-twin.
       *
       * The residual is nearly linear in the node pressure and its slope is known in closed form,
       * `dR/dp = -sum(A_k / c_k)`, which is exactly `den` — so a Newton step needs no extra derivative,
       * only a trial evaluation of the *nonlinear* fluxes. Two steps bring the peak imbalance to a
       * fraction of a percent. The early exit rarely fires — profiled on a V8's manifolds, junctions run
       * 1.8 of the 2 steps on average — which is part of why this runs in wasm (`kernel/euler.ts`).
       */
      if (den > 0) {
        const tol = JUNCTION_BALANCE_TOL * Math.max(scaleGuess, 1e-9);
        for (let iter = 0; iter < 2; iter++) {
          let r = 0;
          for (let i = 0; i < nOut; i++) {
            r += outlets[i]!.probeJunction('outlet', gauge, tJunction, states[i]);
          }
          for (let i = 0; i < inlets.length; i++) {
            r -= inlets[i]!.probeJunction('inlet', gauge, tJunction, states[nOut + i]);
          }
          if (!Number.isFinite(r) || Math.abs(r) <= tol) break;
          const next = clamp(GAS.pAmb + gauge + r / den, 0.3 * pMin, 3 * pMax) - GAS.pAmb;
          if (next === gauge) break;
          gauge = next;
        }
      }

      let signed = 0;
      let scale = 0;
      for (let i = 0; i < nOut; i++) {
        const f = outlets[i]!.applyJunction('outlet', gauge, tJunction, states[i]);
        signed += f;
        scale += Math.abs(f);
      }
      for (let i = 0; i < inlets.length; i++) {
        const f = inlets[i]!.applyJunction('inlet', gauge, tJunction, states[nOut + i]);
        signed -= f;
        scale += Math.abs(f);
      }
      if (scale > 1e-9) {
        const rel = Math.abs(signed) / scale;
        if (rel > this.junctionResidual) this.junctionResidual = rel;
      }

      this.updateMergeNoise(node);
    }
  }

  /**
   * Broadband mixing noise for one junction.
   *
   * Driven by the *shear* between the branches — the velocity spread about their mean — because that is
   * what actually mixes. It therefore vanishes when the branches are quiescent or flowing equally, and
   * peaks when one cylinder is blowing down into pipes that are not, which is the loud, ragged part of
   * a real collector. A floor proportional to the mean flow is added because a merge separates and
   * mixes even with matched branches.
   *
   * Band-limited at the Strouhal frequency of the merge, the same reasoning as the valve throat: eddies
   * shed at roughly `0.2 u / D`, and white noise here would put a rising hiss across the whole spectrum
   * that no amount of level tuning makes sound right.
   *
   * The result is a mass source into whatever the node feeds, shared out by inlet area when it feeds
   * more than one duct. It is injected through the same path a valve uses, so it trades stagnation
   * enthalpy in both directions and a zero-mean source neither heats nor cools the duct.
   */
  private updateMergeNoise(node: JunctionNode): void {
    const { outlets, states } = node;
    const nOut = outlets.length;

    let areaSum = 0;
    let flowSum = 0;
    for (let i = 0; i < nOut; i++) {
      const st = states[i]!;
      areaSum += st.area;
      flowSum += st.area * st.u;
    }
    const uBar = areaSum > 0 ? flowSum / areaSum : 0;
    let shearSq = 0;
    for (let i = 0; i < nOut; i++) {
      const st = states[i]!;
      const d = st.u - uBar;
      shearSq += st.area * d * d;
    }
    const shear = areaSum > 0 ? Math.sqrt(shearSq / areaSum) : 0;
    // Mixing velocity: branch-to-branch shear, plus a fraction of the through-flow.
    const uMix = hypot2(shear, 0.2 * uBar);

    const inlet = states[nOut] ?? states[states.length - 1]!;
    const rho = Math.max(inlet.rho, 1e-6);
    const sigma = MERGE_TURBULENCE * this.turbulence * rho * inlet.area * uMix;

    const dia = Math.sqrt((4 * inlet.area) / Math.PI);
    const strouhalHz = (0.2 * uMix) / Math.max(dia, 1e-3);
    // See the note in `engineSim`: `1 - exp(-w h)` is the pole mapping, not `w h`. On the substep,
    // because that is how often this runs.
    const k = clamp(1 - Math.exp(-2 * Math.PI * strouhalHz * this.substepDt), 1e-4, 0.9);

    const white = node.noise.next() * sigma * this.substepNoiseScale;
    node.lp1 += k * (white - node.lp1);
    node.lp2 += k * (node.lp1 - node.lp2);

    for (const d of node.downstream) {
      const fed = this.fedByNode[d.duct]!;
      const duct = fed.pipe;
      this.fedFlow[d.duct] = node.lp2 * d.share;
      const valve = fed.valve;
      // The injected mass arrives at the gas temperature already there, so a zero-mean source
      // neither heats nor cools the duct.
      // Read through an array: a float returned from a call that is not inlined is boxed.
      const port = this.portState;
      duct.readPort(port);
      valve.cylTemp = port[1]!;
      valve.cylPressure = port[0]!;
      valve.throatArea = port[2]!;
    }
  }

  /** Gauge pressure along `collector`, or cylinder 0's duct if there is none, for the display. */
  samplePressure(out: Float32Array): void {
    (this.collector ?? this.primaries[0]!).samplePressure(out);
  }

  sampleWallTemperature(out: Float32Array): void {
    (this.collector ?? this.primaries[0]!).sampleWallTemperature(out);
  }

  meanWallTemp(): number {
    let sum = 0;
    const ducts = this.ducts;
    for (const d of ducts) sum += d.meanWallTemp();
    return sum / ducts.length;
  }

  /**
   * First quarter-wave resonance of everything between cylinder 0's valve and open air, Hz.
   *
   * Walked along the graph, so a manifold's links, a downpipe and the collector all count, as they do
   * for the panel's tuning readout (`pathToAir`).
   */
  quarterWaveHz(): number {
    const path = this.airPath;
    if (path.length === 0) return this.primaries[0]!.quarterWaveHz();
    // Ducts in series: acoustic lengths, and so the reciprocals of their quarter-wave frequencies, add.
    let invTotal = 0;
    for (const duct of path) invTotal += 1 / duct.quarterWaveHz();
    return 1 / invTotal;
  }

  exportWall(): Float64Array {
    return (this.collector ?? this.primaries[0]!).exportWall();
  }

  /** The radiation corner of radiating mouth `m`, in the order of `mouthFlows`. */
  mouthCutoffRadOf(m: number): number {
    return this.radiating[m]!.mouthCutoffRad;
  }

  /**
   * Upper band limit for mouth `m`'s radiation: whichever of the two model limits binds first.
   * Plane-wave cut-on rules for a wide mouth; the solver's cell size rules for a narrow one,
   * where cut-on can sit above 10 kHz.
   */
  bandLimitRadOf(m: number): number {
    const d = this.radiating[m]!;
    return Math.min(d.planeWaveCutoffRad, d.resolutionCutoffRad);
  }

  setAirSpeed(v: number): void {
    for (const d of this.ducts) d.setAirSpeed(v);
  }
}

