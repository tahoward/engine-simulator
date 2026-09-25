// Bundles the benchmark to plain ESM for bare node, so `node --cpu-prof` sees the same code
// path the app runs. Separate from the app config: different root, different target.
import { defineConfig } from 'vite';
import { fileURLToPath } from 'node:url';

const here = fileURLToPath(new URL('.', import.meta.url));

export default defineConfig({
  root: fileURLToPath(new URL('..', import.meta.url)),
  build: {
    ssr: true,
    minify: false,
    target: 'node20',
    outDir: here + 'dist',
    emptyOutDir: true,
    rollupOptions: {
      input: here + (process.env.BENCH_ENTRY ?? 'bench.ts'),
      output: { entryFileNames: (process.env.BENCH_ENTRY ?? 'bench.ts').replace('.ts', '.mjs'), format: 'es' },
    },
  },
});
