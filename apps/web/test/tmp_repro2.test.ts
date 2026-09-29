import { it } from 'vitest';
import { placeLoosePipe, splitDuctAt, removeDuct, type ExhaustGraph } from '../src/model/exhaustGraph.js';
import { defaultConfig, makeSegment, type EngineSpec } from '../src/model/spec.js';
import { layoutGraph } from '../src/scene/exhaustLayout.js';
import { collectSnapTargets } from '../src/scene/drawing.js';
import { EngineMesh } from '../src/scene/EngineMesh.js';

it('targets', () => {
  const spec = { ...defaultConfig().engine, cylinders: 1 } as EngineSpec;
  const ports = [new EngineMesh(spec).exhaustPort(0)];
  const g: ExhaustGraph = { ducts: [] };
  g.ducts.push({ id: 'r', segments: [makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.042 })], from: { kind: 'valve', cylinder: 0 }, to: { kind: 'mouth' } });
  const L = placeLoosePipe(g, [0.3, 0, 0], 0.042, 0.8);
  const node = splitDuctAt(g, L, 0.4)!;
  const b = g.ducts.find((d) => d.id === 'r')!; b.to = { kind: 'node', node };
  let p = layoutGraph(ports, g);
  console.log('before joint', JSON.stringify(p.joints.get(node)?.axis), JSON.stringify(p.joints.get(node)?.centre));
  removeDuct(g, g.ducts.find((d) => d.continues === L)!.id);
  p = layoutGraph(ports, g);
  console.log('after joint', JSON.stringify(p.joints.get(node)?.axis), JSON.stringify(p.joints.get(node)?.centre), p.joints.get(node)?.limbs.length);
  for (const t of collectSnapTargets(g, p, ports)) console.log(t.kind, JSON.stringify(t.point), (t as any).node ?? (t as any).duct ?? '');
});
