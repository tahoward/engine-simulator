/**
 * An engine as a file: what Export engine writes and Import engine reads back, and what a shared link
 * carries in its address.
 *
 * Everything that makes the engine what it is: its settings, the pipe and collector its exhaust is compiled
 * from, and its exhaust as drawn. Nothing is trusted as it is read: pipes are rebuilt by `makeSegment`, the
 * exhaust by `graphFromJson`, and an exhaust that does not fit the engine is dropped for one compiled from
 * the pipe and collector instead.
 */

import { graphFromJson, validateGraph } from './exhaustGraph.js';
import { isTurbocharged } from './turbo.js';
import { fullLoadTorque, makeSegment, type EngineConfig, type EngineSpec } from './spec.js';

/** What an exported engine file says it is, so an import can tell one from any other JSON. */
export const ENGINE_FILE_FORMAT = 'engine-simulator/engine';
export const ENGINE_FILE_VERSION = 1;

/** `config` as the text of an engine file. */
export function engineFile(config: EngineConfig): string {
  const file = {
    format: ENGINE_FILE_FORMAT,
    version: ENGINE_FILE_VERSION,
    engine: config.engine,
    pipe: config.pipe,
    collector: config.collector,
    graph: config.graph,
  };
  return JSON.stringify(file, null, 2);
}

/** A file name for an engine, without its extension: its cylinders and layout, "engine-v8-crossplane", say. */
export function engineFileName(spec: EngineSpec): string {
  const kind =
    spec.crankType === 'boxer'
      ? `flat-${spec.cylinders}`
      : spec.cylinders > 2 && spec.vAngle > 0
        ? `v${spec.cylinders}${spec.cylinders === 8 ? `-${spec.crankType}` : ''}`
        : spec.cylinders === 1
          ? 'single'
          : `inline-${spec.cylinders}`;
  return `engine-${kind}`;
}

/**
 * The engine an engine file's text holds, over `base`, or `null` where it is not one: not JSON, JSON with
 * no engine in it, or a file that says it is something else.
 */
export function readEngineFile(text: string, base: EngineConfig): { config: EngineConfig; graphDropped: boolean } | null {
  try {
    const raw = JSON.parse(text) as { format?: unknown };
    if (raw && typeof raw === 'object' && 'format' in raw && raw.format !== ENGINE_FILE_FORMAT) return null;
    return readConfig(raw, base);
  } catch {
    return null;
  }
}

/**
 * An engine read from what a link or an exported file holds, over `base`, which it changes and returns,
 * so anything it leaves out comes out as that does. Its pipes are rebuilt by `makeSegment` and its exhaust by
 * `graphFromJson` rather than trusted as they are, and an exhaust that does not describe the engine is
 * dropped, `graphDropped` saying so, for one to be compiled from its pipe and collector instead. Throws
 * where there is no engine in it at all.
 */
export function readConfig(raw: unknown, base: EngineConfig): { config: EngineConfig; graphDropped: boolean } {
  let graphDropped = false;
  if (!raw || typeof raw !== 'object') throw new Error('not an engine');
  const parsed = raw as Partial<EngineConfig>;
  if (!parsed.engine || typeof parsed.engine !== 'object') throw new Error('not an engine');

  Object.assign(base.engine, parsed.engine);
  // A link may carry the load as a torque in N*m, `loadTorque`, rather than as a fraction.
  const loadTorque = (parsed.engine as { loadTorque?: unknown }).loadTorque;
  if (typeof loadTorque === 'number' && parsed.engine.load === undefined) {
    base.engine.load = Math.min(Math.max(loadTorque / fullLoadTorque(base.engine, isTurbocharged(graphFromJson(parsed.graph) ?? undefined)), 0), 1.5);
  }
  delete (base.engine as { loadTorque?: unknown }).loadTorque;
  base.engine.freeRunning = true;

  if (Array.isArray(parsed.pipe) && parsed.pipe.length > 0) {
    base.pipe = parsed.pipe.map((s) => makeSegment(s));
  }
  // The collector is part of the geometry like anything else: without this a shared link would lose it
  // and come back with the default.
  if (Array.isArray(parsed.collector) && parsed.collector.length > 0) {
    base.collector = parsed.collector.map((s) => makeSegment(s));
  }

  /**
   * A drawn graph, if there is one, rebuilt by `graphFromJson` rather than trusted as-is. A graph that does
   * not describe this engine is dropped in favour of compiling a fresh one, which is the same thing a
   * change of topology does.
   */
  const graph = graphFromJson(parsed.graph);
  if (graph) {
    if (validateGraph(graph, base.engine.cylinders).length === 0) base.graph = graph;
    else graphDropped = true;
  }
  return { config: base, graphDropped };
}
