# Performance

The whole simulation has to keep up with real time on one CPU core, on the browser's audio thread.
This page covers what it costs, how the pipe grid is sized to stay within that, and what keeps the
hot code fast.

## Cost per preset

`npm run bench` measures how much of one CPU core each preset needs, running in Node on V8 (the
JavaScript engine in Chrome and Node). It runs at full throttle and 6500 rpm, then again at each
engine's own preset speed. Measured on an Apple M3 Max:

```
preset                             cells  steps  % of one core @ 6500 rpm
Open header, single                   19      1        5.8%
Megaphone, single                     27      1        6.6%
Tuned expansion chamber, single       32      1        7.0%
Street muffler, single                37      1        7.5%
Long tuned pipe, single               45      1        8.3%
45° V-twin, 2-into-1                  46      1       11.5%
90° V-twin, 2-into-2                  48      1       10.9%
Parallel twin, 360°                   34      1       11.6%
Inline three                          73      1       19.9%
Inline four                           82      1       24.8%
Boxer four                            89      1       25.2%
Inline five                           88      1       30.9%
Inline six                            95      1       36.9%
V6, 60°, manifold per bank           150      1       40.5%
Boxer six                            152      1       40.9%
V8, flatplane, manifold per bank     128      1       53.9%
V8, crossplane, manifold per bank    192      1       56.2%
```

Speed barely matters: every preset costs within about a point of the same at its own rpm. What
matters is what the engine is made of — cylinders, junctions and pipe cells.

!!! warning "Measuring small effects"

    Single timed runs on a warm laptop drift by a few points over a few minutes. The bench takes
    the best of several runs and reports the spread; compare before/after runs back to back.

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

- **A [Wasm SIMD](glossary.md#simd) kernel.** The two cell loops of the solver (reconstruction and update) and the
  junction solve run as WebAssembly with SIMD, which works on two numbers at once. It is written in
  [AssemblyScript](glossary.md#assemblyscript) in `kernel/euler.ts`. The cell loops give bit-identical results to the TypeScript
  path, which the tests check.
- **Almost no garbage on the audio path.** Very little allocates per sample. Numbers that cross a function call
  too large for V8 to inline are passed through preallocated typed arrays or object fields, because
  V8 would otherwise wrap each one in a new heap object. Every numeric class field starts with a
  number, so V8 stores it in place.
- **One trigonometry pass per crank angle.** `crankAt` works out piston position, both its
  derivatives and the cylinder volume from one `sin`, `cos` and `sqrt`. A test checks it against the
  separate functions so the maths cannot drift apart.
- **Multiplies instead of divides.** Constants such as `1/(γ-1)` are precomputed, because V8 does not
  turn a division by a constant into a multiply.

## Why the HUD shows cells, not CPU %

The audio thread cannot time itself. `performance` is not available in the [AudioWorklet](glossary.md#audioworklet) (confirmed
missing in Chrome), and `currentTime` only moves once per block. So the HUD shows cells and steps,
which cost is proportional to, rather than a percentage it would have to make up.

## Sources

- CFL condition: [Courant, Friedrichs and Lewy 1928](references.md#cfl1928).
- Resolution of finite-volume schemes: [Toro 2009](references.md#toro2009).
