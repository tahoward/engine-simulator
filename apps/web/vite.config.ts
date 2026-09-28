import { defineConfig } from 'vitest/config';

export default defineConfig({
  // Relative base so a production build can be dropped on any static host
  // (GitHub Pages project sites included) without path rewriting.
  base: './',
  build: {
    target: 'es2022',
  },
  worker: {
    // The AudioWorklet is loaded via `?worker&url`, which reuses this config.
    // It must be an ES module chunk: AudioWorkletGlobalScope has no importScripts.
    format: 'es',
  },
  test: {
    include: ['test/**/*.test.ts'],
    environment: 'node',
    // Several tests render seconds of audio at 48 kHz through the Euler solver.
    testTimeout: 120_000,
  },
});
