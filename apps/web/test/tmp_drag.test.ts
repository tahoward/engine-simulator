import { it, expect } from 'vitest';
import * as THREE from 'three';
import { attachToLooseStart, compileLayout, joinDuctEnd, nodeOrder, placeLoosePipe, splitDuctAt, type ExhaustGraph } from '../src/model/exhaustGraph.js';
import { defaultConfig, makeSegment, type EngineSpec } from '../src/model/spec.js';
import { layoutGraph, pipesMeetAt } from '../src/scene/exhaustLayout.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';
import { moveJunction, refitBends } from '../src/scene/turboPlacement.js';
import { layoutPipe } from '../src/scene/PipeMesh.js';
import { fitCurve } from '../src/scene/drawing.js';

const spec = { ...defaultConfig().engine, cylinders: 2, vAngle: 45, exhaustLayout: 'perBank' } as EngineSpec;
const mesh = new EngineMesh(spec);
const ports = [0, 1].map((i) => mesh.exhaustPort(i));
const report = (g: ExhaustGraph, label: string) => {
  const p = layoutGraph(ports, g);
  const bad = nodeOrder(g).filter((n) => !pipesMeetAt(g, p, n));
  console.log(label, bad.length ? `GAP at ${bad}` : 'ok', g.ducts.map((d) => `${d.id}:${d.fitted ? 'F' : ''}${d.segments.length}`).join(' '));
  return bad;
};
const endOf = (g: ExhaustGraph, id: string) => {
  const p = layoutGraph(ports, g).ducts.get(id)!;
  const d = g.ducts.find((x) => x.id === id)!;
  const s = layoutPipe(d.segments, p.origin, p.heading);
  return { point: s.joints.at(-1)!, dir: s.jointDirections.at(-1)! };
};

const fresh = (len: number): ExhaustGraph => ({
  ducts: [0, 1].map((i) => ({ id: `runner${i}`, segments: [makeSegment({ kind: 'pipe', length: len, dIn: 0.042 })], from: { kind: 'valve' as const, cylinder: i }, to: { kind: 'mouth' as const }, headingYaw: 0.3 * (i ? -1 : 1), headingPitch: 0 })),
});
it('scenarios', () => {
  // A: runner1 drawn into runner0's open end (merge), tail drawn from junction, then move junction.
  {
    const g = fresh(0.3);
    const t = endOf(g, 'runner0'), tip = endOf(g, 'runner1');
    const node = joinDuctEnd(g, 'runner0', { position: t.point.toArray() as any, axis: t.dir.toArray() as any })!;
    const r1 = g.ducts.find((d) => d.id === 'runner1')!;
    r1.segments.push(fitCurve(tip.point, tip.dir, t.point, t.dir, { dIn: 0.042, dOut: 0.042 })); r1.fitted = true; r1.to = { kind: 'node', node };
    g.ducts.push({ id: 'tail', segments: [makeSegment({ kind: 'pipe', length: 0.4, dIn: 0.05 })], from: { kind: 'node', node }, to: { kind: 'mouth' } });
    refitBends(g, ports, spec); report(g, 'A before');
    const p = t.point.clone().add(new THREE.Vector3(0.05, 0.08, -0.03));
    moveJunction(g, ports, spec, node, p.toArray() as any, t.dir.toArray() as any);
    report(g, 'A moved');
  }
  // B: tee: runner1 fitted into runner0's side, move runner0's... move the tee junction? (unpinned) then drag runner0 heading
  {
    const g = fresh(0.5);
    const node = splitDuctAt(g, 'runner0', 0.3)!;
    const r1 = g.ducts.find((d) => d.id === 'runner1')!;
    const tip = endOf(g, 'runner1');
    const a = endOf(g, 'runner0');
    r1.segments.push(fitCurve(tip.point, tip.dir, a.point, a.dir, { dIn: 0.042, dOut: 0.042 })); r1.fitted = true; r1.to = { kind: 'node', node };
    refitBends(g, ports, spec); report(g, 'B before');
    const r0 = g.ducts.find((d) => d.id === 'runner0')!;
    r0.headingYaw = (r0.headingYaw ?? 0) + 0.4; r0.headingPitch = (r0.headingPitch ?? 0) + 0.2;
    refitBends(g, ports, spec); report(g, 'B turned runner0');
    moveJunction(g, ports, spec, node, a.point.clone().add(new THREE.Vector3(0.05, 0.05, 0)).toArray() as any, a.dir.toArray() as any);
    report(g, 'B moved tee');
  }
  // C: runner drawn into loose start, then move the loose pipe (free position)
  {
    const g = fresh(0.3);
    const loose = placeLoosePipe(g, [0.5, 0.3, 0.1], 0.042, 0.5);
    const r1 = g.ducts.find((d) => d.id === 'runner1')!;
    const node = attachToLooseStart(g, 'runner1', loose, [0, 0, 1])!;
    r1.segments.push(makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 })); r1.fitted = true;
    refitBends(g, ports, spec); report(g, 'C before');
    moveJunction(g, ports, spec, node, [0.55, 0.35, 0.2], [0, 0, 1]);
    report(g, 'C moved');
    // branch fitted into loose pipe side, then move junction
    const l = g.ducts.find((d) => d.id === loose)!;
    const k = splitDuctAt(g, loose, 0.25)!;
    const r0 = g.ducts.find((d) => d.id === 'runner0')!;
    r0.to = { kind: 'node', node: k }; r0.segments.push(makeSegment({ kind: 'pipe', length: 0.1, dIn: 0.042 })); r0.fitted = true;
    refitBends(g, ports, spec); report(g, 'C teed');
    moveJunction(g, ports, spec, node, [0.6, 0.4, 0.25], [0, 0, 1]);
    report(g, 'C moved teed');
    void l;
  }
});
