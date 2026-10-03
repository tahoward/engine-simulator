/**
 * Where the engine makes its sound, as it is drawn: each tailpipe's outlet, each intake, the casing and the
 * turbos, in the scene's frame (x across the crank, y up, z along it, rearwards). The simulation times and
 * levels each source's path to the listener from these, so what is heard is what is on screen.
 */

import * as THREE from 'three';

import { compileExhaust, reversedDucts, solverGraph, type ExhaustGraph } from '../model/exhaustGraph.js';
import { engineShell, exhaustPortOf, type Vec3 } from '../model/geometry.js';
import { type EngineConfig, type EngineSpec, type SoundSources } from '../model/spec.js';
import { turboPortsOf } from '../model/turbo.js';
import { seatEngineTurbos } from './engineTurbos.js';
import { layoutGraph, type ExhaustPlacement, type ExhaustPort } from './exhaustLayout.js';
import { inletLayout } from './inletLayout.js';
import { seatLengthwaysHeaders } from './headerTool.js';
import { layoutPipe } from './PipeMesh.js';
import { refitBends, seatHeaders, seatManifolds, seatTurbos } from './turboPlacement.js';

/** Every cylinder's exhaust port, as the drawn engine has it. */
export function exhaustPorts(spec: EngineSpec): ExhaustPort[] {
  return Array.from({ length: spec.cylinders }, (_, b) => {
    const port = exhaustPortOf(spec, b);
    return { position: new THREE.Vector3(...port.position), direction: new THREE.Vector3(...port.direction) };
  });
}

/**
 * Fit `graph` to the engine before it is laid out: compiled manifolds and headers built to reach their ports,
 * the turbos a layout seats put in their places, and the bends into what pipes join refitted. Seated first,
 * since building a compiled manifold or header can add pipes.
 */
export function seatGraph(graph: ExhaustGraph, ports: ExhaustPort[], spec: EngineSpec): void {
  seatManifolds(graph, ports);
  seatLengthwaysHeaders(graph, ports, spec);
  seatHeaders(graph, ports);
  seatEngineTurbos(graph, ports, spec);
  seatTurbos(graph, ports, spec);
  refitBends(graph, ports, spec);
}

const vec3 = (v: THREE.Vector3): Vec3 => [v.x, v.y, v.z];

/**
 * Where `graph`, laid out as `placement`, makes its sound. Each mouth is named by the duct the solver
 * radiates it from (`solverGraph`): a pipe drawn the other way from how its gas runs has its mouth where it
 * was drawn from, and a turbo with nothing drawn from its outlet breathes out at its outlet flange.
 */
export function soundSources(graph: ExhaustGraph, placement: ExhaustPlacement, spec: EngineSpec): SoundSources {
  const reversed = reversedDucts(graph);
  const mouths: SoundSources['mouths'] = [];
  for (const duct of solverGraph(graph).ducts) {
    if (duct.to.kind !== 'mouth') continue;
    const drawn = graph.ducts.find((d) => d.id === duct.id);
    const place = placement.ducts.get(duct.id);
    if (drawn && place) {
      if (reversed.has(duct.id)) {
        mouths.push({ duct: duct.id, position: vec3(place.origin) });
      } else {
        const joints = layoutPipe(drawn.segments, place.origin, place.heading).joints;
        mouths.push({ duct: duct.id, position: vec3(joints[joints.length - 1] ?? place.origin) });
      }
    } else if (duct.from.kind === 'node') {
      const turbo = placement.turbos.get(duct.from.node);
      if (turbo) mouths.push({ duct: duct.id, position: turbo.outlet.point });
    }
  }

  const shell = engineShell(spec);
  const casing: Vec3 = [0, (shell.top + shell.bottom) / 2, 0];
  const [first, second] = inletLayout(spec).tracts;
  const intake = vec3(first!.mouth);

  const placed = (graph.turbos ?? []).flatMap((t) => (t.position ? [t.position] : []));
  const turbo: Vec3 | undefined =
    placed.length > 0
      ? [0, 1, 2].map((i) => placed.reduce((sum, p) => sum + p[i]!, 0) / placed.length) as Vec3
      : undefined;
  return {
    mouths,
    intake,
    ...(second ? { secondIntake: vec3(second.mouth) } : {}),
    engine: casing,
    ...(turbo ? { turbo } : {}),
  };
}

/**
 * Where the engine of `config` makes its sound, as the app would draw it: for a config with no scene, such as
 * a preset exported for the simulation's own tests.
 */
export function configSources(config: EngineConfig): SoundSources {
  const graph = structuredClone(config.graph ?? compileExhaust(config.engine, config.pipe, config.collector, 0));
  const ports = exhaustPorts(config.engine);
  seatGraph(graph, ports, config.engine);
  const placement = layoutGraph(ports, graph, turboPortsOf(graph, config.engine));
  return soundSources(graph, placement, config.engine);
}
