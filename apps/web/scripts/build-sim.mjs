/**
 * Builds `crates/engine-sim-wasm` for the web and embeds it in `src/audio/worklet/simWasm.ts` as
 * base64: `npm run build:sim`.
 *
 * The generated file is committed, so neither `npm run dev` nor `npm test` needs a Rust toolchain;
 * only changing the simulation does.
 *
 * Base64 rather than an asset URL because `AudioWorkletGlobalScope` has no `fetch` and no
 * `importScripts`: the bytes have to arrive as part of the module graph. The same code path then runs
 * in the worklet and in Node, where the tests and the benchmark load it.
 */

import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = fileURLToPath(new URL('.', import.meta.url));
const web = join(here, '..');
const root = join(web, '..', '..');
const out = join(web, 'src', 'audio', 'worklet', 'simWasm.ts');
const wasmPath = join(root, 'target', 'wasm32-unknown-unknown', 'release', 'engine_sim_wasm.wasm');

// SIMD128 is switched on for this target in the repository's `.cargo/config.toml`.
execFileSync('cargo', ['build', '-p', 'engine-sim-wasm', '--release', '--target', 'wasm32-unknown-unknown'], {
  cwd: root,
  stdio: 'inherit',
});

const bytes = readFileSync(wasmPath);
const b64 = bytes.toString('base64');
writeFileSync(
  out,
  `/**
 * GENERATED FILE — do not edit.
 *
 * Built from \`crates/engine-sim-wasm\` by \`npm run build:sim\`. ${bytes.length} bytes of Wasm,
 * base64-encoded so it can be imported from an AudioWorklet, which has no \`fetch\`.
 */

/** Base64 Wasm for the simulation. */
export const SIM_WASM_BASE64 =
  '${b64}';

/** Uncompressed size, for the record. */
export const SIM_WASM_BYTES = ${bytes.length};
`,
);
console.log(`sim: ${bytes.length} B wasm -> ${b64.length} B base64 -> ${out}`);
