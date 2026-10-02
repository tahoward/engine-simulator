/**
 * The simulation, from the Wasm build of `crates/engine-sim`, with the interface the worklet uses.
 *
 * The module crosses a plain C ABI (see `crates/engine-sim-wasm`): configuration goes in as UTF-8
 * JSON written into the module's memory, audio comes out as `f32` samples there, and a snapshot as
 * JSON. `AudioWorkletGlobalScope` has no `TextEncoder` or `TextDecoder`, so the UTF-8 is done here.
 *
 * Also what the tests and the benchmark run, in Node, so they exercise exactly the module the app
 * ships.
 */

import type { ExhaustGraph } from '../../model/exhaustGraph.js';
import type { LaunchConfig, EngineConfig, EngineSnapshot, EngineSpec, SoundSources } from '../../model/spec.js';
import { SIM_WASM_BASE64 } from './simWasm.js';

interface Exports {
  memory: WebAssembly.Memory;
  alloc(len: number): number;
  sim_new(rate: number, ptr: number, len: number): number;
  sim_free(h: number): void;
  sim_set_engine(h: number, ptr: number, len: number): number;
  sim_set_graph(h: number, ptr: number, len: number): number;
  sim_set_sources(h: number, ptr: number, len: number): number;
  sim_set_listener(h: number, x: number, y: number, z: number): void;
  sim_start_launch(h: number, ptr: number, len: number): number;
  sim_stop_launch(h: number): void;
  sim_set_controls(h: number, throttle: number, load: number): void;
  sim_set_time_scale(h: number, scale: number): void;
  sim_set_ignition(h: number, on: number): void;
  sim_render(h: number, n: number): number;
  sim_snapshot(h: number): number;
  sim_snapshot_len(h: number): number;
  sim_error(h: number): number;
  sim_error_len(h: number): number;
}

/**
 * Base64 to bytes, by hand: `AudioWorkletGlobalScope` has neither `atob` nor `Buffer`.
 */
function decodeBase64(b64: string): Uint8Array {
  const alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';
  const value = new Uint8Array(128);
  for (let i = 0; i < alphabet.length; i++) value[alphabet.charCodeAt(i)] = i;
  let end = b64.length;
  while (end > 0 && b64[end - 1] === '=') end--;
  const out = new Uint8Array(Math.floor((end * 3) / 4));
  let o = 0;
  for (let i = 0; i < end; i += 4) {
    const a = value[b64.charCodeAt(i)]!;
    const b = value[b64.charCodeAt(i + 1)]!;
    const c = i + 2 < end ? value[b64.charCodeAt(i + 2)]! : 0;
    const d = i + 3 < end ? value[b64.charCodeAt(i + 3)]! : 0;
    const n = (a << 18) | (b << 12) | (c << 6) | d;
    out[o++] = (n >> 16) & 0xff;
    if (i + 2 < end) out[o++] = (n >> 8) & 0xff;
    if (i + 3 < end) out[o++] = n & 0xff;
  }
  return out;
}

let compiled: WebAssembly.Module | null = null;

/**
 * Compiled once, synchronously, on first use. A document's main thread may not compile a module
 * this size synchronously; an AudioWorklet and Node may.
 */
function module(): WebAssembly.Module {
  compiled ??= new WebAssembly.Module(decodeBase64(SIM_WASM_BASE64) as BufferSource);
  return compiled;
}

/** UTF-8 bytes of `s`. */
function utf8(s: string): Uint8Array {
  const out = new Uint8Array(s.length * 3);
  let n = 0;
  for (let i = 0; i < s.length; i++) {
    let c = s.charCodeAt(i);
    if (c >= 0xd800 && c < 0xdc00 && i + 1 < s.length) {
      const d = s.charCodeAt(i + 1);
      if (d >= 0xdc00 && d < 0xe000) {
        c = 0x10000 + ((c - 0xd800) << 10) + (d - 0xdc00);
        i++;
      }
    }
    if (c < 0x80) out[n++] = c;
    else if (c < 0x800) {
      out[n++] = 0xc0 | (c >> 6);
      out[n++] = 0x80 | (c & 0x3f);
    } else if (c < 0x10000) {
      out[n++] = 0xe0 | (c >> 12);
      out[n++] = 0x80 | ((c >> 6) & 0x3f);
      out[n++] = 0x80 | (c & 0x3f);
    } else {
      out[n++] = 0xf0 | (c >> 18);
      out[n++] = 0x80 | ((c >> 12) & 0x3f);
      out[n++] = 0x80 | ((c >> 6) & 0x3f);
      out[n++] = 0x80 | (c & 0x3f);
    }
  }
  return out.subarray(0, n);
}

/** `bytes` as a string, decoding UTF-8. */
function fromUtf8(bytes: Uint8Array): string {
  let s = '';
  for (let i = 0; i < bytes.length; ) {
    const b = bytes[i]!;
    let c: number;
    if (b < 0x80) {
      c = b;
      i += 1;
    } else if (b < 0xe0) {
      c = ((b & 0x1f) << 6) | (bytes[i + 1]! & 0x3f);
      i += 2;
    } else if (b < 0xf0) {
      c = ((b & 0x0f) << 12) | ((bytes[i + 1]! & 0x3f) << 6) | (bytes[i + 2]! & 0x3f);
      i += 3;
    } else {
      c = ((b & 0x07) << 18) | ((bytes[i + 1]! & 0x3f) << 12) | ((bytes[i + 2]! & 0x3f) << 6) | (bytes[i + 3]! & 0x3f);
      i += 4;
    }
    s += String.fromCodePoint(c);
  }
  return s;
}

/** The snapshot as the Wasm module writes it, before its sample arrays become typed arrays. */
type RawSnapshot = Omit<EngineSnapshot, 'pipePressure' | 'ductPressure' | 'inletPressure' | 'inletVelocity' | 'runnerPressure' | 'plenumZones' | 'launch'> & {
  pipePressure: number[];
  ductPressure: number[];
  inletPressure: number[];
  inletVelocity: number[];
  runnerPressure: number[];
  plenumZones: number[];
  launch: (Omit<NonNullable<EngineSnapshot['launch']>, 'points'> & { points: number[] }) | null;
};

export class Sim {
  private readonly ex: Exports;
  private readonly handle: number;
  private heapF32: Float32Array;

  constructor(
    readonly sampleRate: number,
    config: EngineConfig,
  ) {
    this.ex = new WebAssembly.Instance(module(), {}).exports as unknown as Exports;
    const [ptr, len] = this.write(JSON.stringify(config));
    this.handle = this.ex.sim_new(sampleRate, ptr, len);
    if (this.handle === 0) throw new Error('engine-sim: the config does not parse');
    this.heapF32 = new Float32Array(this.ex.memory.buffer);
  }

  /** Copy `json` into the module's memory, for a call to take. */
  private write(json: string): [number, number] {
    const bytes = utf8(json);
    const ptr = this.ex.alloc(bytes.length);
    new Uint8Array(this.ex.memory.buffer, ptr, bytes.length).set(bytes);
    return [ptr, bytes.length];
  }

  private check(status: number, what: string): void {
    if (status === 0) return;
    const bytes = new Uint8Array(this.ex.memory.buffer, this.ex.sim_error(this.handle), this.ex.sim_error_len(this.handle));
    throw new Error(`engine-sim: ${what}: ${fromUtf8(bytes)}`);
  }

  /** Change the engine: only the fields given. */
  setEngine(engine: Partial<EngineSpec>): void {
    const [ptr, len] = this.write(JSON.stringify(engine));
    this.check(this.ex.sim_set_engine(this.handle, ptr, len), 'setEngine');
  }

  /** Replace the exhaust graph; `null` compiles one from the layout. */
  setGraph(graph: ExhaustGraph | null): void {
    const [ptr, len] = this.write(JSON.stringify(graph));
    this.check(this.ex.sim_set_graph(this.handle, ptr, len), 'setGraph');
  }

  /** Where the engine makes its sound, as drawn. */
  setSources(sources: SoundSources): void {
    const [ptr, len] = this.write(JSON.stringify(sources));
    this.check(this.ex.sim_set_sources(this.handle, ptr, len), 'setSources');
  }

  /** Put the listener's ear at `position`, m; `null` where it stands by default. Passes numbers, not JSON. */
  setListener(position: [number, number, number] | null): void {
    const [x, y, z] = position ?? [NaN, NaN, NaN];
    this.ex.sim_set_listener(this.handle, x, y, z);
  }

  startLaunch(config: LaunchConfig): void {
    const [ptr, len] = this.write(JSON.stringify(config));
    this.check(this.ex.sim_start_launch(this.handle, ptr, len), 'startLaunch');
  }

  stopLaunch(): void {
    this.ex.sim_stop_launch(this.handle);
  }

  /** The operating point alone. Allocates nothing. */
  setControls(throttle: number, load: number): void {
    this.ex.sim_set_controls(this.handle, throttle, load);
  }

  /** Run at `scale` of real time: 1 is real time, less is slow motion. */
  setTimeScale(scale: number): void {
    this.ex.sim_set_time_scale(this.handle, scale);
  }

  /** Switch the ignition: off, the engine coasts to a standstill. */
  setIgnition(on: boolean): void {
    this.ex.sim_set_ignition(this.handle, on ? 1 : 0);
  }

  /** Render `out.length` samples into `out`. Allocates nothing unless the module's memory grew. */
  renderInto(out: Float32Array): void {
    const n = out.length;
    const ptr = this.ex.sim_render(this.handle, n);
    if (this.heapF32.buffer !== this.ex.memory.buffer) this.heapF32 = new Float32Array(this.ex.memory.buffer);
    const heap = this.heapF32;
    const base = ptr >>> 2;
    for (let i = 0; i < n; i++) out[i] = heap[base + i]!;
  }

  /** Render `n` samples into a new array. */
  render(n: number): Float32Array {
    const out = new Float32Array(n);
    // In blocks, so the module's own buffer stays small.
    const BLOCK = 4096;
    for (let at = 0; at < n; at += BLOCK) this.renderInto(out.subarray(at, Math.min(at + BLOCK, n)));
    return out;
  }

  /** A snapshot for the renderer. Resets the peak meter. */
  snapshot(): EngineSnapshot {
    const ptr = this.ex.sim_snapshot(this.handle);
    const bytes = new Uint8Array(this.ex.memory.buffer, ptr, this.ex.sim_snapshot_len(this.handle));
    const raw = JSON.parse(fromUtf8(bytes)) as RawSnapshot;
    return {
      ...raw,
      pipePressure: Float32Array.from(raw.pipePressure),
      ductPressure: Float32Array.from(raw.ductPressure),
      inletPressure: Float32Array.from(raw.inletPressure),
      inletVelocity: Float32Array.from(raw.inletVelocity),
      runnerPressure: Float32Array.from(raw.runnerPressure),
      plenumZones: Float32Array.from(raw.plenumZones),
      launch: raw.launch && { ...raw.launch, points: Float32Array.from(raw.launch.points) },
    };
  }

  /** Free the module's simulation. The instance is unusable afterwards. */
  free(): void {
    this.ex.sim_free(this.handle);
  }
}
