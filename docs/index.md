# Engine Simulator

Hear engines from a single-cylinder to a V8, right in your browser. The sound is **worked out from
physics**, and you build the exhaust yourself.

<!-- These links are absolute, not relative. The app/ folder is only added to the site at deploy
time, so MkDocs can't see it at build time and a relative link would fail the strict check. This
means the button opens the live app even from a local preview. -->

[Launch the simulator :material-engine:](https://tahoward.github.io/engine-simulator/app/){ .md-button .md-button--primary }
[Source on GitHub :fontawesome-brands-github:](https://github.com/tahoward/engine-simulator){ .md-button }

There are no recordings here, and no synthesizer tones run through filters. Instead, the simulator
models what really happens:

- It works out the pressure inside each cylinder as the crank turns.
- It lets gas out through the exhaust valve as the cam opens it.
- It follows pressure waves down the pipe you built. The waves travel, sharpen, bounce back where
  the pipe gets wider or narrower, and leave as sound from the open end.

For the pipe, it solves the **[Euler equations](glossary.md#euler-equations)** (the basic equations of how gas flows) along the
pipe's length. So if you make a header longer, the note drops because the wave really takes longer
to come back. No setting is just mapped to a tone.

## What it does

- **Every common engine layout**: a single, a parallel twin or V-twin, inline three, four, five and
  six, a 60° V6, [crossplane](glossary.md#crossplane-and-flatplane-cranks) and flatplane V8s, and flat-four and flat-six [boxers](glossary.md#boxer). Each one has its
  real [firing order](glossary.md#firing-order) and crankshaft.
- **An exhaust you build.** Drag pipes longer. Make them wider or narrower. Draw new pipes from an
  exhaust port or off the side of another pipe, and join them together. The simulator solves exactly
  what you draw.
- **Realistic gas flow** in every pipe. It includes heat loss to the pipe walls, friction, and sound
  leaving every open end toward a listener standing outside.
- **Natural variation.** Each combustion cycle is slightly different, the crank speed wobbles, and
  there is mechanical noise from the engine itself. So it doesn't sound like a loop.
- **Runs in real time on one CPU core.** It runs in an [AudioWorklet](glossary.md#audioworklet) (the browser's dedicated audio
  thread). The busiest code runs as [Wasm SIMD](glossary.md#simd) (fast compiled code that works on several numbers at
  once).

## Where to start

<!-- A table instead of Material's `grid cards`, because Prettier reformats list indentation and
that quietly breaks the grid markup. -->

| Page                                                       | What it covers                                                                  |
| ---------------------------------------------------------- | ------------------------------------------------------------------------------- |
| :material-play: **[Getting Started](getting-started.md)**  | Install it, run it, test it, benchmark it and build it.                         |
| :material-gesture-tap: **[Controls](controls.md)**         | Start the engine, edit and draw pipes, select and delete.                       |
| :material-sitemap: **[Architecture](architecture.md)**     | The two threads, how they talk, and where the code lives.                       |
| :material-waves: **[Acoustics](acoustics.md)**             | How the gas and sound are simulated, with heat, friction and flow noise.        |
| :material-engine-outline: **[The Engine](engine.md)**      | The cylinder model, firing orders, what more cylinders do, and pipe joins.      |
| :material-speedometer: **[Performance](performance.md)**   | How much work it takes, and how it keeps up in real time.                       |
| :material-check-all: **[Verification](verification.md)**   | What the tests check, and what they compare against.                            |
| :material-alert-outline: **[Known Limits](limits.md)**     | Where the model is a rough guess, and by how much.                              |
| :material-book-alphabet: **[Glossary](glossary.md)**           | Plain definitions of every technical term, with links to fuller ones.           |
| :material-bookshelf: **[References](references.md)**           | The published papers and books the models come from.                            |
| :material-rocket-launch: **[Deployment](deployment.md)**   | How the docs and the simulator are published to GitHub Pages.                   |
