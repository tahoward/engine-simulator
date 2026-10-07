# Getting Started

The repository holds the simulation, the web app and the desktop app:

```
crates/engine-sim/        the simulation, in Rust
crates/engine-sim-wasm/   the simulation for the web app's AudioWorklet
apps/web/                 the web app: the interface, and the AudioWorklet that runs the simulation
apps/desktop/             the desktop app: the same interface, with the simulation running natively
docs/                     these pages
```

## Requirements

- **Node.js 22 or newer.** This is the version the deploy workflow uses. Vite 8 and Vitest 5 need
  at least 20.19.
- For the web app, a browser that supports WebGL (3D graphics), [AudioWorklet](glossary.md#audioworklet) (the browser's
  dedicated audio thread) and Wasm [SIMD](glossary.md#simd). Any current Chrome, Edge, Firefox or Safari works.
- **Rust**, for anything that changes the simulation, and for the desktop app. Install it with
  [rustup](https://rustup.rs), then add the Wasm target:

  ```bash
  rustup target add wasm32-unknown-unknown
  ```

  The web app alone needs no Rust: the built simulation is committed, as
  `apps/web/src/audio/worklet/simWasm.ts`.

## Run the web app

```bash
npm install
npm run dev
```

Open the URL that Vite prints. Then click to start the engine. Browsers only let sound start after
you click or press a key, so you won't hear anything until you do.

## Run the desktop app

To use the desktop app without building it, download the macOS `.dmg` or the Windows `.exe` from
the [latest release](https://github.com/tahoward/engine-simulator/releases/latest). To build and
run it from the source:

```bash
npm install
npm run desktop:dev
```

This builds the Rust side, starts the interface's dev server, and opens the app. The engine starts
at once: there is no click to start. The simulation is built optimised even in this development
build, since it has to run in real time.

```bash
npm run desktop:build
```

builds the app for this machine: on macOS, `Engine Simulator.app` in
`target/release/bundle/macos/`; on Windows, an installer in `target/release/bundle/nsis/` and
`target/release/bundle/msi/`.

The desktop app is built on the system it is for. On Windows, that needs the Microsoft C++ Build
Tools (the "Desktop development with C++" workload) and Rust's default MSVC toolchain, as
[Tauri's prerequisites](https://v2.tauri.app/start/prerequisites/) describe. The WebView2 runtime the
interface runs in comes with Windows 10 and 11. Audio goes out through WASAPI, the system's own audio
interface, at the device's sample rate.

## The scripts

From the repository root:

| Script                   | What it does                                                                                  |
| ------------------------ | --------------------------------------------------------------------------------------------- |
| `npm run dev`            | Starts the web app's Vite dev server. The page reloads as you edit.                           |
| `npm run build`          | Checks types, then bundles the web app into `apps/web/dist/`. Every path is relative.         |
| `npm test`               | Runs the web app's tests in Node: the interface, and the Wasm build against the reference renders. |
| `npm run typecheck`      | Runs `tsc --noEmit` by itself.                                                                |
| `npm run bench`          | Measures how much real time each preset needs in the Wasm build. See [Performance](performance.md). |
| `npm run build:sim`      | Rebuilds the Wasm simulation into `apps/web/src/audio/worklet/simWasm.ts`.                    |
| `npm run desktop:dev`    | Runs the desktop app in development.                                                          |
| `npm run desktop:build`  | Builds the desktop app.                                                                       |
| `npm run docs:serve`     | Serves these docs with MkDocs and reloads as you edit.                                        |
| `npm run docs:build`     | Builds these docs into `site/`. Fails if any link is broken.                                  |

And for the simulation:

| Command                                               | What it does                                                        |
| ----------------------------------------------------- | ------------------------------------------------------------------- |
| `cargo test --release -p engine-sim`                  | Runs the physics tests and the reference renders. See [Verification](verification.md). |
| `cargo run --release -p engine-sim --example bench`   | Measures how much real time each preset needs in the native build.  |
| `cargo run --release -p engine-sim --example compare -- --help` | Compares a preset's sound with a recording of the real engine. See [Verification](verification.md#against-a-recording). |

After changing the simulation, run `npm run build:sim`, so the web app runs the change too, and
commit the rebuilt `simWasm.ts` with it.

The physics tests run the web app's engine presets, from `crates/engine-sim/tests/fixtures/presets.json`.
After changing a preset in `apps/web/src/model/spec.ts`, run `npm run export:presets -w apps/web` to
write it there.

## Build the web app for production

```bash
npm run build
```

The output goes to `apps/web/dist/`. Every path in the build is relative, including the path to the
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

    The docs build never reads the code, and the app build never reads these pages. They only come
    together at deploy time. The docs go to the root of the site and the app goes to `app/`.
