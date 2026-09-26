# Architecture

## Threads

The whole simulation runs **inside an [AudioWorklet](glossary.md#audioworklet)**, the browser's dedicated audio thread. That
keeps the sound smooth. Memory cleanup and slow graphics frames on the main page can't interrupt it.

The gas solver moves forward one small time step for every audio sample. Each step must be short
enough that a wave can't skip past a whole cell of the pipe (the [CFL](glossary.md#cfl-condition) limit). The cells are sized so
that one step per sample is enough. See [Performance](performance.md).

The diagram shows what each thread does and what passes between them:

```
main thread                          audio thread
──────────────────────────           ─────────────────────────────────────────
three.js scene, pipe editor          Cylinder:  V(θ), ideal gas, Wiebe burn
DOM panel, scope                     Valve:     lift curve → compressible orifice
    │                                Euler:     MUSCL-Hancock + HLLC, a graph of
    │  engine params / geometry                 ducts and junctions, one CFL
    │                                           step per audio sample
    ▼         (postMessage)          Radiation: open end + far field + ground
  snapshot @ 60 Hz  ◄────────────────────────────────┘
  crank angle, pressures, valve lift, pressure along the pipe, solver work
```

The two threads talk with [`postMessage`](glossary.md#postmessage-and-sharedarraybuffer), which sends copies of messages. The simulator doesn't use
`SharedArrayBuffer` (memory both threads can share) on purpose. That would need special
cross-origin-isolation headers from the web server, and plain static hosting can't send them. The
main thread only needs a snapshot 60 times a second. Each snapshot holds 128 pressure readings
along the pipe and is under a kilobyte.

## Source layout

```
src/model/spec.ts              shared types, gas properties, firing plans, engine + pipe presets
src/model/exhaustGraph.ts      the exhaust as a graph of ducts and junctions; compiled layouts, edits
src/model/geometry.ts          exhaust port positions and the pipe turn convention
src/audio/AudioEngine.ts       main-thread AudioContext + worklet lifecycle
src/audio/worklet/
  processor.ts                 thin AudioWorkletProcessor shell
  engineSim.ts                 the per-sample simulation (no audio deps → testable)
  cylinder.ts  valve.ts        thermodynamics, cam lift, compressible orifice flow
  plenum.ts  intake.ts         the finite intake plenum, and the runners from it to each valve
  drivetrain.ts                the dyno run: clutch, six-speed gearbox and car on the rollers
  eulerPipe.ts                 one duct: MUSCL-Hancock + HLLC, walls, thermal, radiation
  exhaustSystem.ts             the exhaust graph: every duct and the junctions between them
  kernel.ts  kernelWasm.ts     loader for the Wasm SIMD kernel, and the kernel itself, base64
  radiation.ts  listener.ts    open-end radiation, far field, ground reflection
  dsp.ts                       noise, resonators, small shared helpers
kernel/euler.ts                the kernel's AssemblyScript source (npm run build:kernel)
src/scene/                     Viewer, animated cutaway EngineMesh, PipeMesh, PipeEditor, fittings
src/ui/                        control panel, waveform/spectrum/pressure scope
test/                          physics tests + a small FFT for them
bench/                         real-time cost harness (also profilable under node --cpu-prof)
```
