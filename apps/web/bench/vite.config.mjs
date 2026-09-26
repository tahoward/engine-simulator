// Bundles a benchmark or tool script to plain ESM for bare node, so `node --cpu-prof` sees the same
// code path the app runs. Separate from the app config: different root, different target.
//
// The entry is `bench.ts`, or the script the mode names: `--mode export-presets` bundles
// `export-presets.ts`.
import { defineConfig } from 'vite';
import { fileURLToPath } from 'node:url';

const here = fileURLToPath(new URL('.', import.meta.url));

export default defineConfig(({ mode }) => {
  const entry = mode === 'production' ? 'bench.ts' : `${mode}.ts`;
  return {
    root: fileURLToPath(new URL('..', import.meta.url)),
    build: {
      ssr: true,
      minify: false,
      target: 'node20',
      outDir: here + 'dist',
      emptyOutDir: true,
      rollupOptions: {
        input: here + entry,
        output: { entryFileNames: entry.replace('.ts', '.mjs'), format: 'es' },
      },
    },
  };
});
