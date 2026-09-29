import { it } from 'vitest';
import { compileLayout, placeLoosePipe, splitDuctAt, removeDuct, solverGraph, validateGraph, reversedDucts, type ExhaustGraph } from '../src/model/exhaustGraph.js';
import { defaultConfig, makeSegment, type EngineSpec } from '../src/model/spec.js';

const spec = { ...defaultConfig().engine, cylinders: 1 } as EngineSpec;
const show = (g: ExhaustGraph) => g.ducts.map((d) => `${d.id}: ${JSON.stringify(d.from)} -> ${JSON.stringify(d.to)} cont=${d.continues ?? ''} fitted=${d.fitted ?? ''}`).join('\n');

for (const loose of [false, true]) {
  for (const del of ['A1', 'A2', 'B']) {
    it(`loose=${loose} delete ${del}`, () => {
      const g: ExhaustGraph = loose ? { ducts: [] } : compileLayout({ ...spec, cylinders: 2 } as EngineSpec, [makeSegment({ kind: 'pipe', length: 0.8, dIn: 0.042 })], []);
      if (loose) {
        g.ducts.push({ id: 'r', segments: [makeSegment({ kind: 'pipe', length: 0.3, dIn: 0.042 })], from: { kind: 'valve', cylinder: 0 }, to: { kind: 'mouth' } });
      }
      const through = loose ? placeLoosePipe(g, [0, 0, 0], 0.042, 0.8) : 'runner0';
      const branch = loose ? 'r' : 'runner1';
      const node = splitDuctAt(g, through, 0.4)!;
      const b = g.ducts.find((d) => d.id === branch)!;
      b.to = { kind: 'node', node }; b.fitted = true;
      const names: Record<string, string> = { A1: through, A2: g.ducts.find((d) => d.continues === through)!.id, B: branch };
      console.log(`--- loose=${loose} before\n` + show(g));
      const ok = removeDuct(g, names[del]!);
      console.log(`--- delete ${del}(${names[del]}) ok=${ok}\n` + show(g) + '\nreversed=' + [...reversedDucts(g)] + '\nsolver:\n' + show(solverGraph(g)) + '\nvalid=' + validateGraph(g, loose ? 1 : 2));
    });
  }
}
