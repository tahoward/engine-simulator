# Getting Started

## Requirements

- **Node.js 22 or newer.** This is the version the deploy workflow uses. Vite 8 and Vitest 5 need
  at least 20.19.
- A browser that supports WebGL (3D graphics) and [AudioWorklet](glossary.md#audioworklet) (the browser's dedicated audio
  thread). Any current Chrome, Edge, Firefox or Safari works.

## Run the simulator

```bash
npm install
npm run dev
```

Open the URL that Vite prints. Then click to start the engine. Browsers only let sound start after
you click or press a key, so you won't hear anything until you do.

## The npm scripts

| Script                | What it does                                                                                                   |
| --------------------- | -------------------------------------------------------------------------------------------------------------- |
| `npm run dev`         | Starts the Vite dev server. The page reloads as you edit.                                                      |
| `npm run build`       | Checks types, then bundles into `dist/`. Uses relative paths, so it can be served from any folder.             |
| `npm run preview`     | Serves the last build.                                                                                         |
| `npm test`            | Runs the physics tests in Node, with no browser. Some tests render seconds of audio, so it takes a few minutes. |
| `npm run typecheck`   | Runs `tsc --noEmit` by itself.                                                                                 |
| `npm run bench`       | Measures how much real time each preset needs. See [Performance](performance.md).                              |
| `npm run build:kernel`| Rebuilds the Wasm SIMD kernel from `kernel/euler.ts` into `src/audio/worklet/kernelWasm.ts`.                   |
| `npm run docs:serve`  | Serves these docs with MkDocs and reloads as you edit.                                                         |
| `npm run docs:build`  | Builds these docs into `site/`. Fails if any link is broken.                                                   |

You only need `build:kernel` after editing `kernel/euler.ts`. The built kernel is already committed.
The kernel is the fast core of the gas solver. It is written in [AssemblyScript](glossary.md#assemblyscript) (a TypeScript-like
language that compiles to [Wasm](glossary.md#webassembly-wasm)). It sits outside `src/` on purpose, so the TypeScript build never
sees it.

## Build for production

```bash
npm run build
```

The output goes to `dist/`. Every path in the build is relative, including the path to the
AudioWorklet code. So the same build works from any folder, including the `app/` folder it is
published in. See [Deployment](deployment.md).

## Work on the documentation

The docs use [MkDocs](https://www.mkdocs.org/) with the
[Material](https://squidfunk.github.io/mkdocs-material/) theme. So you need Python as well as Node.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-docs.txt

npm run docs:serve
```

This serves the docs at <http://localhost:8000> and reloads as you edit. With the virtual
environment active, `npm run docs:serve` just runs `mkdocs serve`, and `npm run docs:build` just
runs `mkdocs build --strict`.

!!! note "The docs and the app build separately"

    The docs build never reads the TypeScript code, and the app build never reads these pages.
    They only come together at deploy time. The docs go to the root of the site and the app goes
    to `app/`.
