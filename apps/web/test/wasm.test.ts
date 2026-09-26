/**
 * The Wasm build the app ships, against the simulation's reference renders.
 *
 * `crates/engine-sim/tests/fixtures/scenarios.json` holds, for a few dozen scenarios, the audio as a
 * hash per 10 ms block and every snapshot, and the Rust crate's own tests check the native build
 * against it bit for bit. This checks the Wasm build, through the same `Sim` the worklet uses,
 * against the same file: so the desktop app and the web app produce the same sound, sample for
 * sample.
 */

import { describe, expect, it } from 'vitest';

import scenarios from '../../../crates/engine-sim/tests/fixtures/scenarios.json';

import { Sim } from '../src/audio/worklet/sim.js';
import type { EngineConfig } from '../src/model/spec.js';

interface Scenario {
  name: string;
  sampleRate: number;
  config: EngineConfig;
  steps: Array<Record<string, unknown>>;
  result: { blocks: string[]; snapshots: unknown[] };
}

const fixture = scenarios as unknown as { blockSize: number; scenarios: Scenario[] };

/** FNV-1a over the raw bytes, as the fixture hashes each block. */
function fnv(samples: Float32Array): string {
  const bytes = new Uint8Array(samples.buffer, samples.byteOffset, samples.byteLength);
  let h = 0x811c9dc5;
  for (let i = 0; i < bytes.length; i++) {
    h ^= bytes[i]!;
    h = Math.imul(h, 0x01000193) >>> 0;
  }
  return h.toString(16).padStart(8, '0');
}

function plain(snapshot: unknown): unknown {
  return JSON.parse(JSON.stringify(snapshot, (_k, v) => (v instanceof Float32Array ? Array.from(v) : v)));
}

describe('the Wasm build renders every reference scenario bit for bit', () => {
  it.each(fixture.scenarios.map((s) => s.name))('%s', (name) => {
    const s = fixture.scenarios.find((q) => q.name === name)!;
    const sim = new Sim(s.sampleRate, s.config);
    const chunks: Float32Array[] = [];
    const snapshots: unknown[] = [];
    for (const step of s.steps) {
      if ('render' in step) chunks.push(sim.render(step.render as number));
      else if ('controls' in step) {
        const [t, l] = step.controls as [number, number];
        sim.setControls(t, l);
      } else if ('engine' in step) sim.setEngine(step.engine as never);
      else if ('graph' in step) sim.setGraph(step.graph as never);
      else if ('dyno' in step) {
        if (step.dyno) sim.startDyno(step.dyno as never);
        else sim.stopDyno();
      } else if ('snapshot' in step) snapshots.push(plain(sim.snapshot()));
    }
    sim.free();
    const audio = new Float32Array(chunks.reduce((a, c) => a + c.length, 0));
    let at = 0;
    for (const c of chunks) {
      audio.set(c, at);
      at += c.length;
    }
    const blocks: string[] = [];
    for (let i = 0; i < audio.length; i += fixture.blockSize) {
      blocks.push(fnv(audio.subarray(i, Math.min(i + fixture.blockSize, audio.length))));
    }
    expect(blocks).toEqual(s.result.blocks);
    // Through the same JSON round trip, which writes -0 as 0 on both sides.
    expect(snapshots).toEqual(plain(s.result.snapshots));
  });
});
