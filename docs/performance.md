# Performance

The whole simulation has to keep up with real time on one CPU core: the browser's audio thread in
the web app, a real-time thread of its own in the desktop app. This page covers what it costs, how
the pipe grid is sized to stay within that, and what keeps the hot code fast.

## Cost per preset

Each benchmark measures how much of one CPU core each preset needs to make a second of audio, held
at 6500 rpm at full throttle, the solver's worst case. The simulation is the same in both; only the
build differs:

- `cargo run --release -p engine-sim --example bench` measures the native build, as the desktop app
  runs it.
- `npm run bench` in `apps/web` measures the Wasm build, as the web app runs it, in Node's V8 (the
  JavaScript engine in Chrome), rendered in blocks of 128 as the worklet renders.

Measured on an Apple M3 Max:

```
preset                              cells   native     Wasm
Single, megaphone                      26     5.1%     6.1%
45° V-twin, 2-into-1                   46    10.0%    11.8%
90° V-twin, 2-into-2                   46     9.2%    11.4%
Parallel twin, 360°                    33    10.9%    11.5%
Inline three                           73    17.3%    20.2%
Inline four, Honda F20C               112    20.9%    24.5%
Boxer four                             85    22.3%    25.4%
Inline five                            88    26.3%    30.2%
Inline six                             94    31.6%    35.7%
Boxer six                             150    33.5%    39.0%
V6, Toyota 2GR                        148    34.3%    39.9%
V8, flatplane, manifold per bank      122    39.4%    44.0%
V8, overcammed                        142    41.2%    47.5%
V8, Chevrolet LT6                     216    41.5%    46.7%
V8, crossplane, manifold per bank     134    41.6%    48.6%
V8, Chevrolet LT2                     234    43.8%    51.8%
```

`cells` counts the exhaust's. Each cylinder also has an intake runner, always on the finest grid
the sample rate allows, and the budget charges those cells first: see [The intake](engine.md#the-intake)
for why the runners are not coarsened. So on an engine whose budget is tight, the exhaust gives up
cells instead. The crossplane V8, with eight junctions along its manifolds, has its exhaust at 134
cells where it would otherwise have 182. Engines with headers have two junctions rather than eight,
which leaves room for the long primaries.

Speed barely matters: every preset costs within about a point of the same at its own rpm. What
matters is what the engine is made of — cylinders, junctions and pipe cells.

The desktop app has headroom the web app does not, beyond the table: its render thread keeps two
device buffers of audio ready ahead of the device, so an occasional slow block goes unheard, where
the browser has to finish every 128-sample block before the device needs it.

!!! warning "Measuring small effects"

    Single timed runs on a warm laptop drift by a few points over a few minutes. The benchmarks take
    the best of several runs and report the spread; compare before/after runs back to back.

## One step per audio sample

The gas solver takes exactly **one step per audio sample**, for every engine.

A step can only be as long as the [CFL condition](glossary.md#cfl-condition) allows. (The CFL condition is the stability rule
for this kind of solver: a wave must not cross more than one cell per step.) So one step per sample
sets a minimum cell length:

```
minimum cell = design wave speed / (sample rate × CFL number)
             = 1400 / (48000 × 0.85)  ≈ 34.3 mm at 48 kHz   (37.3 mm at 44.1 kHz)
```

The presets ask for 35 mm cells, just above that. The **Solver resolution** slider goes from 35 to
120 mm; anything finer than the minimum is raised to it.

The step count is fixed, not chosen sample by sample, because it sets the shape of the filter that
brings the solver's output to the audio rate. A count that changed between quiet and loud samples
would change that filter as it went and add broadband noise.

The design wave speed of 1400 m/s is 17% above the fastest `|u| + c` (gas speed plus sound speed)
measured anywhere across 1200–9000 rpm, 0.15 to full throttle, on a megaphone, a long tuned pipe, a
2-into-1 and a 50 mm stub. The fastest was 1192 m/s, in the stub at 9000 rpm. Going over it is not a
stability failure: the solver then splits that sample into extra steps. The `substepBursts` counter
records those samples, and the tests require it to stay at zero.

## The grid budget

A big engine may get coarser cells than it asked for, so it stays in real time. The budget is
counted in pipe cells, with each cylinder and each junction charged as a fixed number of cells:

```
cells available = 1216 − 102 × cylinders − 27 × junctions
```

Those weights come from timing every preset and fitting cost to what each engine is made of: about
6.6% of a core per cylinder, 1.7% per junction and 0.064% per cell. If the pipes need more cells than
the budget leaves, the cell size is increased in 1% steps until they fit. All ducts share one cell
size.

Cells are sized by *length*, not by a fixed count per pipe, so a short pipe costs less.

## Cell size and frequency range

Cell size is the trade between cost and detail. The solver resolves up to about `c/(5 dx)`, where
`c` is the speed of sound at the mouth. With 35 mm cells and a mouth sound speed of 400 m/s, that is
about 2.3 kHz. Above that limit, output is filtered off rather than radiated, because
it would be numerical noise, not sound. For a wide pipe mouth the [plane-wave](glossary.md#plane-wave) assumption (sound moving
as flat fronts down the pipe) breaks down near those frequencies anyway, so the grid is often not
the limiting factor.

## What keeps it fast

- **[SIMD](glossary.md#simd) cell loops.** The solver's cell loops — the reconstruction, the
  [HLLC](glossary.md#hllc) face fluxes, the conservative update and the gas-to-wall heat transfer — work on
  two cells at once (`simd.rs`): NEON on Arm, SSE2 on x86, SIMD128 in the Wasm build. Each lane
  does exactly the arithmetic one cell alone would, so the results are the same bits as a plain
  loop's. Branches become selects that compute both sides and keep the one the plain loop would
  have taken.
- **A table-driven `pow` and `exp`.** The simulation takes a few hundred powers every sample: the
  cylinders' heat transfer, the valves' and throttle's orifice flow, the open ends and the junctions.
  `pow.rs` is Arm's optimized-routines algorithm, as musl ships it: about twice as fast as the classic
  fdlibm one and slightly more accurate.
- **The same maths everywhere.** Every transcendental function is a software implementation in
  the crate rather than the platform's C library, and nothing is fused into multiply-adds, so the
  native and Wasm builds compute identical results on every machine.
- **Each crank angle once.** The cylinder evaluates the crank, one `sin`, `cos` and `sqrt` for
  position, both its derivatives and the volume, at each angle only once: a sub-step ends where the
  next begins, and the result is kept.
- **Nothing allocated per sample.** Every buffer the per-sample path uses is made when the engine or
  exhaust is built.

## Keeping up with real time

In the browser, the audio thread cannot time itself. `performance` is not available in the
[AudioWorklet](glossary.md#audioworklet), and `currentTime` only moves once per block. So the HUD shows cells and steps, which
cost is proportional to, and the app judges whether the audio is keeping up from the audio clock
against the wall clock, over two-second windows.

The desktop app's render thread times every block it renders, and also counts the times the device
found the ring buffer empty. Either one, over the same two-second windows, raises the same notice
the web app does.

## Sources

- CFL condition: [Courant, Friedrichs and Lewy 1928](references.md#cfl1928).
- Resolution of finite-volume schemes: [Toro 2009](references.md#toro2009).
