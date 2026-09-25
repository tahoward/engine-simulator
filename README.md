# engine-simulator

A single, a twin, an inline three, four, five or six, a V6, a V8 or a flat four or six whose sound is
**simulated from physics**, in the browser, with an exhaust system you build yourself.

**[Live Demo](https://tahoward.github.io/engine-simulator/app/)** · **[Documentation](https://tahoward.github.io/engine-simulator/)**

Nothing here is sampled, and nothing is an oscillator through a filter bank. A crank-angle
thermodynamic model works out the cylinder pressure; the exhaust valve is a compressible orifice
whose area follows a cam lift curve; and the pipe you assemble is solved as the one-dimensional Euler
equations, with a shock-capturing finite-volume scheme, so pressure waves genuinely propagate,
steepen, reflect off every change in cross-section, and radiate from the open end. Lengthen a header
and the note drops because the wave takes longer to come back, not because a number was mapped to a
filter cutoff.

## Features

- Every common layout, each with its real firing order and crank: single, parallel and V-twins,
  inline three to six, a 60° V6, crossplane and flatplane V8s, and flat four and six boxers
- An exhaust you build: drag, resize and draw pipes, and snap them into junctions — what you draw is
  what is solved
- Nonlinear gas dynamics with wall heat transfer, friction and radiation to an outdoor listener
- Combustion scatter, crank speed ripple and structure-borne mechanical noise
- Real time on one core, in an AudioWorklet, with the hot loops in a Wasm SIMD kernel

## Quick Start

```bash
npm install
npm run dev      # then click to start audio (browsers require a gesture)
npm test         # physics validated against closed-form acoustics
npm run bench    # real-time cost of every preset
npm run build    # static output in dist/, deployable anywhere
```

Node 20.19 or newer.

## Controls

- **Space** starts and stops.
- **Click a pipe** to select it; **drag the blue spheres** to lengthen and angle a segment, **drag the
  rings** to change its diameter, or type exact sizes into the panel.
- **Draw a pipe** in the panel to route a new runner from a port, a junction or the side of a pipe.
- **Delete** or **Backspace** removes the selected segment or junction.
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

- TypeScript, three.js and the Web Audio API's AudioWorklet
- A Wasm SIMD kernel written in AssemblyScript
- Vite for development and building, Vitest for the physics tests
