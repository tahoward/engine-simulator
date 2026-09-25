/**
 * Compiles `kernel/euler.ts` (AssemblyScript) and embeds the result in
 * `src/audio/worklet/kernelWasm.ts` as base64.
 *
 * `npm run build:kernel`. The generated file is committed, so neither `npm run dev` nor
 * `npm test` needs AssemblyScript installed — only changing the kernel does.
 *
 * Base64 rather than an asset URL for one reason: `AudioWorkletGlobalScope` has no `fetch`
 * and no `importScripts`, so the bytes have to arrive as part of the module graph. Embedding
 * also means Vite needs no plugin and vitest needs no loader, and the same code path runs in
 * Node and in the browser. The cost is about a third more bytes than the raw wasm, which at
 * this size is a few kilobytes.
 */

import { execFileSync } from 'node:child_process';
import { mkdtempSync, readFileSync, rmSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = fileURLToPath(new URL('.', import.meta.url));
const root = join(here, '..');
const src = join(here, 'euler.ts');
const out = join(root, 'src', 'audio', 'worklet', 'kernelWasm.ts');

const tmp = mkdtempSync(join(tmpdir(), 'euler-kernel-'));
const wasmPath = join(tmp, 'euler.wasm');

/**
 * `--runtime stub` because the kernel allocates nothing: every field lives in one static
 * block. `--initialMemory 1` is one 64 kB page, which holds the 24 x 256 x 8 byte block, the
 * scalar I/O block and the 14 x 16 x 8 byte junction block with room to spare — see `CAP` and
 * `J_CAP` in euler.ts.
 */
const args = [
  src,
  '--outFile',
  wasmPath,
  '--runtime',
  'stub',
  '--enable',
  'simd',
  '-O3',
  '--noAssert',
  '--initialMemory',
  '1',
];

try {
  execFileSync(process.execPath, [join(root, 'node_modules', 'assemblyscript', 'bin', 'asc.js'), ...args], {
    stdio: 'inherit',
  });

  const bytes = readFileSync(wasmPath);
  const b64 = bytes.toString('base64');

  const banner = `/**
 * GENERATED FILE — do not edit.
 *
 * Built from \`kernel/euler.ts\` by \`npm run build:kernel\`. ${bytes.length} bytes of wasm,
 * base64-encoded so it can be imported from an AudioWorklet, which has no \`fetch\`.
 */
`;

  writeFileSync(
    out,
    `${banner}
/** Base64 wasm for the f64x2 Euler kernel. */
export const KERNEL_WASM_BASE64 =
  '${b64}';

/** Uncompressed size, for the record. */
export const KERNEL_WASM_BYTES = ${bytes.length};
`,
  );

  console.log(`kernel: ${bytes.length} B wasm -> ${b64.length} B base64 -> ${out}`);
} finally {
  rmSync(tmp, { recursive: true, force: true });
}
