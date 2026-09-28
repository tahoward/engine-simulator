# Architecture

## One simulation, two apps

The simulation is one [Rust](glossary.md#rust) crate, `crates/engine-sim`. Two apps run it:

- **The web app**, in the browser. The crate is compiled to [Wasm](glossary.md#webassembly-wasm) and runs inside an
  [AudioWorklet](glossary.md#audioworklet), the browser's dedicated audio thread.
- **The desktop app**, built with [Tauri](glossary.md#tauri). The crate is compiled natively and runs on a
  real-time thread of its own, feeding the system's audio device directly.

Both apps share the same interface, the web app's, and the same physics, to the last bit: the two
builds produce the same samples. See [Verification](verification.md#the-reference-renders).

The gas solver moves forward one small time step for every audio sample. Each step must be short
enough that a wave can't skip past a whole cell of the pipe (the [CFL](glossary.md#cfl-condition) limit). The cells are sized so
that one step per sample is enough. See [Performance](performance.md).

## In the browser

```
main thread                          audio thread (AudioWorklet)
──────────────────────────           ─────────────────────────────────────────
three.js scene, pipe editor          engine-sim, compiled to Wasm:
DOM panel, scope                       Cylinder:  V(θ), ideal gas, Wiebe burn
    │                                  Valve:     lift curve → compressible orifice
    │  engine params / geometry        Euler:     MUSCL-Hancock + HLLC, a graph of
    │                                             ducts and junctions, one CFL
    ▼         (postMessage)                       step per audio sample
  snapshot @ 60 Hz  ◄────────────────  Radiation: open end + far field + ground
  crank angle, pressures, valve lift, pressure along the pipe, solver work
```

The two threads talk with [`postMessage`](glossary.md#postmessage-and-sharedarraybuffer), which sends copies of messages. The simulator doesn't use
`SharedArrayBuffer` (memory both threads can share) on purpose. That would need special
cross-origin-isolation headers from the web server, and plain static hosting can't send them. The
main thread only needs a snapshot 60 times a second. Each snapshot holds 128 pressure readings
along the pipe and is a few kilobytes.

Throttle and load are not messages. They go as AudioParams, which the audio thread reads with
every block it renders, so even an overloaded audio thread, which never gets round to its messages,
still answers the throttle.

Inside the worklet, `Sim` (`src/audio/worklet/sim.ts`) wraps the Wasm module. The module has a
plain C interface: configuration goes in as JSON written into its memory, audio comes out as samples
in its memory, and a snapshot comes out as JSON. There is no generated JavaScript glue, because the
glue tools rely on `TextEncoder`, `TextDecoder` and `fetch`, and an AudioWorklet has none of them. For
the same reason the module is embedded in the code as base64 (`simWasm.ts`) and compiled on the audio
thread when the processor starts.

## On the desktop

```
render thread                 device callback              frame thread
─────────────────             ───────────────              ────────────
owns the simulation           drains the ring,             sends snapshots and
renders blocks ahead ──ring──▸ copies to every channel     lag reports to the UI
into a ring buffer            counts underruns
takes snapshots ─────────────────────────────────────────▸
```

The interface runs in the system's webview. Instead of an AudioWorklet it has `NativeEngine`
(`src/audio/NativeEngine.ts`), which sends every change to the app's Rust side as a Tauri command.

- **The render thread** runs the simulation at real-time priority. It keeps a ring buffer filled
  about two device buffers ahead, so a slow block or a rebuild after an edit is absorbed rather than
  heard. Every change from the interface reaches it over a channel, so anything that allocates, such
  as a new exhaust, happens here.
- **The device callback** only copies samples out of the ring. It takes no lock and allocates
  nothing.
- **The frame thread** sends each snapshot to the interface, with the latest 2048 output samples
  for the scope, packed as bytes on a Tauri channel. The scope works out its spectrum from those
  samples the way the browser's analyser does.

Unlike the worklet, the render thread can time itself, so it reports falling behind from what it
measures. The browser has to infer it from the audio clock.

## Source layout

```
crates/engine-sim/src/          the simulation
  engine_sim.rs                 the per-sample simulation: crank, valves, flows, radiation
  cylinder.rs  valve.rs         thermodynamics, cam lift, compressible orifice flow
  plenum.rs  intake.rs          the finite intake plenum, and the runners from it to each valve
  drivetrain.rs                 the dyno run: clutch, six-speed gearbox and car on the rollers
  euler_pipe.rs                 one duct: MUSCL-Hancock + HLLC, walls, thermal, radiation
  exhaust_system.rs             the exhaust graph: every duct and the junctions between them
  exhaust_graph.rs              compiling a layout into a graph, validating one, walking one
  cross_modes.rs                the cross-wise modes of a chamber
  radiation.rs  listener.rs     open-end radiation, far field, ground reflection
  spec.rs  geometry.rs          the data model, gas properties, firing plans, port positions
  dsp.rs  math.rs  pow.rs       small DSP helpers, and the maths every build computes identically
  simd.rs                       two-lane SIMD for the solver's cell loops
crates/engine-sim/tests/        the physics tests, and the reference renders
crates/engine-sim-wasm/         the C interface the web app's worklet loads

apps/web/src/model/             the interface's data model, presets and graph editing
apps/web/src/audio/
  EngineHost.ts                 what the interface needs from the audio, wherever it runs
  AudioEngine.ts                in the browser: the AudioContext and the worklet's lifecycle
  NativeEngine.ts               on the desktop: Tauri commands to the native simulation
  worklet/processor.ts  sim.ts  the AudioWorkletProcessor, and the Wasm module it runs
  worklet/simWasm.ts            the Wasm module, base64 (npm run build:sim)
apps/web/src/scene/             Viewer, animated EngineMesh, PipeMesh, PipeEditor, junction marks
apps/web/src/ui/                control panel, waveform/spectrum/pressure scope
apps/web/test/                  interface tests, and the Wasm build against the reference renders
apps/web/bench/                 real-time cost of the Wasm build

apps/desktop/src-tauri/         the desktop app: audio.rs is the render thread and the stream
```

`apps/web/src/model/spec.ts` and `crates/engine-sim/src/spec.rs` describe the same data: the
engine, the exhaust segments, the snapshot. They read and write the same JSON, which is how the
interface talks to the simulation in both apps. The interface keeps its own copies of the small
pieces of geometry it draws: the crank, the port positions and the valve lift.
