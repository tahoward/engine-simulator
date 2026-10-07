# engine-simulator

An engine of one or two banks of up to six cylinders — a single, an inline six, a V4, V8 or V12, a flat
twin or flat twelve — whose sound is **simulated from physics**, in the browser or as a desktop app, with an exhaust system you build
yourself.

**[Live Demo](https://tahoward.github.io/engine-simulator/app/)** · **[Desktop App](https://github.com/tahoward/engine-simulator/releases/latest)** · **[Documentation](https://tahoward.github.io/engine-simulator/)**

Nothing here is sampled, and nothing is an oscillator through a filter bank. A crank-angle
thermodynamic model works out the cylinder pressure; the exhaust valve is a compressible orifice
whose area follows a cam lift curve; and the pipe you assemble is solved as the one-dimensional Euler
equations, with a shock-capturing finite-volume scheme, so pressure waves genuinely propagate,
steepen, reflect off every change in cross-section, and radiate from the open end. Lengthen a header
and the note drops because the wave takes longer to come back, not because a number was mapped to a
filter cutoff.

## Features

- One or two banks of one to six cylinders, each layout with its real firing order and crank — inline
  one to six, V4, 60° V6, crossplane and flatplane V8s, V10, V12, and flat twin to flat twelve — and a
  firing order and intervals of your own on any of them
- An exhaust you build: drag, resize and draw pipes, and snap them into junctions — what you draw is
  what is solved
- Nonlinear gas dynamics with wall heat transfer, friction and radiation to an outdoor listener
- Combustion scatter, crank speed ripple and structure-borne mechanical noise
- Real time on one core: one Rust simulation, run natively in the desktop app and as Wasm in the
  browser's AudioWorklet, sample for sample the same in both

## Quick Start

```bash
npm install
npm run dev              # the web app; click to start audio (browsers require a gesture)
npm run desktop:dev      # the desktop app, with the simulation running natively
npm test                 # the interface, and the Wasm build against the reference renders
npm run build            # the web app, as static output in apps/web/dist/, deployable anywhere

cargo test --release -p engine-sim                    # the physics, validated against closed-form acoustics
cargo run --release -p engine-sim --example bench     # real-time cost of every preset
```

Node 20.19 or newer. Rust (via [rustup](https://rustup.rs)) for the simulation and the desktop app;
the web app alone runs without it. See
[Getting Started](https://tahoward.github.io/engine-simulator/getting-started/).

## Controls

- **Space** starts and stops.
- **Click a pipe** to select it; **drag its triad's arrows and rings** to move and turn it, **drag the
  pale rings** round it to change its diameter, or **right-click** it to type exact sizes into its menu.
- **Draw a pipe**, in the view's toolbar, routes a new pipe from a port, a junction or the open end of a
  pipe, and can join the side of another.
- **Delete** or **Backspace** removes the selected segment or turbo.
- The whole configuration round-trips through the URL hash, so an exhaust you like is a shareable link.

See [the Controls page](https://tahoward.github.io/engine-simulator/controls/) for the rest.

## Documentation

The full documentation is a [MkDocs](https://www.mkdocs.org/) site under `docs/`, covering the
architecture, the gas solver and its heat and friction models, the engine and firing plans, what it
costs to run, what the tests verify, and the model's known limits.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-docs.txt

npm run docs:serve
```

Both the docs and the simulator are published by a single GitHub Pages deployment: the docs at the
site root, the simulator under `/app/`. See `docs/deployment.md`.

## Tech Stack

- The simulation in Rust (`crates/engine-sim`), with SIMD cell loops, compiled natively and to Wasm
- The interface in TypeScript and three.js (`apps/web`), with the Web Audio API's AudioWorklet in the
  browser
- Tauri for the desktop app (`apps/desktop`), with cpal for its audio
- Vite for development and building, Vitest for the interface tests, `cargo test` for the physics
