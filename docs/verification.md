# Verification

`cargo test --release -p engine-sim` runs the physics tests, in `crates/engine-sim/tests/`. The
simulation is pure code with no audio device, so they run headless. They run the web app's own
engine presets, from `tests/fixtures/presets.json`. Each item below says what is checked and why it
matters.

- **Shock capturing.** [Sod's shock tube](glossary.md#sod-shock-tube) (a standard test: a tube with high pressure on one side
  and low on the other, suddenly opened) is compared to its exact solution. The result must
  converge as cells get finer, with **[zero overshoot](glossary.md#slope-limiter)** for all three limiters. This proves the
  gradient limiting works.
- **Nonlinear steepening.** A loud travelling wave must steepen (1.9x a sine wave after 0.8 m),
  while a quiet one stays a sine wave. Steepening must grow with both loudness and distance.
  This is measured in the middle half of a long duct so no reflection can reach it. A short
  burst in a closed tube does not work, because wall reflections create harmonics of their own
  at any loudness.
- **Cavity stability and [well-balancedness](glossary.md#well-balanced).** A duct with still air must stay still whatever
  its shape (maximum velocity under 1e-6 m/s). A cavity between two changes in pipe width must
  not start pumping by itself. Both guard the [predictor](glossary.md#muscl-hancock) details described in
  [Acoustics](acoustics.md#stability).
- **Wave speed.** A duct closed at one end and open at the other must resonate at odd multiples
  of `c/4L` (the [quarter-wave resonance](glossary.md#quarter-wave-resonance)). Also: halving the length doubles the lowest note, a chamber lowers the tuning,
  and hot gas raises it.
- **Conservation.** Mass and energy are conserved to 1e-9 in a sealed duct.
- **Open-end reflection.** High resonances must fade much faster than low ones. The implied
  reflection strength |R| must land within 0.1 of [Levine-Schwinger](glossary.md#levineschwinger) (the standard result for
  sound leaving an open pipe). A wider mouth must lose treble sooner. As a regression guard,
  the fade rate must not depend on the number of [CFL](glossary.md#cfl-condition) substeps (smaller solver steps inside one
  audio sample).
- **Cylinder integration.** Emptying the cylinder at constant outside pressure, where the exact
  answer is known, must converge as the timestep shrinks. No operating point from full throttle
  down to 0.1 may clamp the gas temperature. Exhaust gas flowing back in must arrive at port
  temperature. A free wind-down from full throttle to closed must stay clean. Tracking
  temperature instead of energy as the cylinder's state fails three of these.
- **Combustion.** The laminar flame speed must match the correlation at room conditions, rise
  with temperature, fall with pressure and leftover exhaust, and be zero outside the flammability
  limits. The burn duration must equal the reference at the reference state, lengthen with rpm by
  much less than a burn taking a fixed time would, and come out longer at part throttle and lean
  in the running engine. The advance map must retard the spark at low rpm and advance it at high
  rpm and part throttle. An inline four bogged down under load must pull away on full throttle.
  Lean mixtures must release less heat, rich ones no more than stoichiometric, and λ 2 must
  misfire. With the fuel cut, a shut throttle at 3200 rpm must leave no fuel in the manifold and
  fire no cycles, and opening the throttle must bring every cycle back.
- **Afterfire.**
  - Pockets: air alone, or fuel with no air, must never light, and neither must a pocket leaner
    than the lean limit. A charge in a cold header must wait there and light within milliseconds of
    hot gas reaching it. A pocket must wash out in proportion to the gas pushed in behind it. A burn
    must release its fuel's heating value, including heat the pipe takes late. Ignition must come
    at irregular moments, the same ones every run.
  - The pipe: heat released in the leading cells must all appear in the gas's energy, and no cell
    may take more than a quarter of its energy at once or pass 2600 K.
  - The running engine: the LT6's launch control and the LT2's rev limiter must pop. With the
    crackle map, a lift above 2500 rpm must pop, and more at full intensity. Without it, the fuel
    cut must leave the pipe silent. The map must fire the spark after top dead centre, skip between
    a fifth and a half of the cycles at full intensity, end within its window and re-arm when the
    throttle opens. Every preset held steady at part and full throttle, and rich, must never
    afterfire.
- **Intake runners.** Left on auto, a runner's bore must follow the valves and its length the rev
  limit, and set values must be used as given. On a 6.2 litre V8, the tuned runner must fill the
  cylinder to over 95% at its tuned speed. That must be more than 10 points above an 80 mm stub, and
  more than it fills either side of that speed. An 800 mm runner must make more torque than a 250 mm
  one at 3500 rpm and less at 6450.
- **A smooth torque curve.** Held at each 400 rpm from 4400 to 8400, the LT6 at full throttle must
  fall by under 3% from one speed to the next on its way up to its peak, and rise by under 4% on its
  way down, so its runners' resonance does not build from one cycle to the next into dips and humps.
- **Variable valve timing.** On the LT6 its cam map must lift the torque at 4500 rpm by more than
  10%, and change nothing at 8400, where the map has the cams at rest. The cams must sit at rest at
  idle, move under load, and return to rest when the map is set to nothing.
- **Cam profile switching.** On the F20C at 3000 rpm, switching must make more than 20% more torque
  than its high-speed cam alone, and the same at 8000, where it is on that cam. It must switch at its
  switch speed, stay switched inside the 150 rpm below it, and switch back below that, and the valves
  must open to each lobe's own lift.
- **Two-stage intake.** On the LT6 at 7800 rpm, below its switch speed, the two-stage intake must
  make more than 2% more torque than its short runners alone, and the same at 8400, where it is on them. It must switch at its switch
  speed, stay switched inside the 150 rpm below it, switch back below that, and change the torque by
  less than 10% across the switch, with no solver recoveries.
- **Turbocharger.** On the RB26 preset, the boost must stay under 0.5 bar at 1500 rpm and hold within
  0.06 bar of its 0.7 bar target at 4000 and 6500 rpm, with the wastegate open at both. Opened
  from part throttle at 3500 rpm, it must take between 0.2 and 2.5 s to reach 90% of its boost. It must
  make within 10% of the real engine's 368 N·m at 4400 rpm, less friction, and between 280 and 350 PS
  at 6800. The 3S-GTE, four cylinders on one turbo, on its 0.7 bar must make within 10% of the Japanese
  engine's rated 304 N·m at 5000 rpm, once on boost, and 225 PS at 6000, the 1.5 EcoBoost Dragon, three
  on one turbo, on its 1.15 bar within 10% of the real engine's rated 290 N·m at 3000 rpm, once on boost,
  and 200 PS at 6000, and the EA855 EVO, five on one
  turbo, on its 1.35 bar within 10% of the real engine's rated 480 N·m at 4500 rpm and 400 PS at 5850
  and 7000; with compressors too small for it, passing 0.12 kg/s each, it must make less power at 7900
  than at 7000, where they are at their choke. With
  the throttle shut on boost, an atmospheric blow-off valve must open and let the boost go without the
  compressor flow ever reversing; with none, the compressor must surge at between 5 and 60 cycles a
  second. A
  naturally aspirated engine must have no turbo, and taking the turbos out must leave no boost behind. The
  six must have two turbines, in its exhaust, passing the valves' flow within 3%, with the pressure at
  their inlets more than 0.2 bar above their outlets; the pressure past them must swing by less than 70%
  of what arrives; and there must be no solver recoveries. A single with its pipe drawn into a turbo must
  make boost, and a turbo with nothing attached must do nothing.
  The strongest tone of what the compressor radiates must be within 3% of its blade-pass frequency.
  On the six, left on the engine's settings, the two turbos must turn at one speed with one wastegate
  opening, without surging; its second given a bigger compressor, 0.22 kg/s, on the same boost, it must
  turn more than 10% slower than the first, their wastegates within 5% of each other, without surging;
  given 0.1 bar more boost, its wastegate must be at least 20% less open than the first's; with its
  blow-off valve taken off, lifting off on boost must open the first's past 90% and leave it shut. Giving a
  turbo its own settings must leave the gas in the pipes and the turbos' speed as they were.
- **Placing turbos.** A turbo put down on an open pipe end must attach it and add nothing at its outlet,
  in a graph the solver accepts, the solver alone given a vent to the air at its outlet flange; the pipe
  must meet its inlet flange and a pipe drawn from its outlet start at its outlet flange, with no junction
  fitting drawn; its inlet must be offered to draw to, its outlet to draw from until a pipe is drawn from
  it, and its node not as a junction; tidying must leave its pipes as they are; taking it out must leave the
  pipe open again; moved, the pipe must follow it and still meet its inlet; and it must survive a link, with its own settings if it has them.
- **A T into a placed pipe.** A runner drawn into the side of a loose pipe must give the solver the half
  before the junction turned round, from the junction to its open end with its taper reversed, in a graph
  it accepts with two mouths, and the gas must reach both halves in the Wasm build.
- **A square T.** A runner drawn into the side of a wider loose pipe with Shift held must, its bend fitted,
  end on the pipe's axis arriving at right angles to it, at the pipe's bore, and keep that through a link;
  with the pipe's bore changed and the bend fitted again, it must end at the new bore, still square. Without
  Shift it must arrive along the pipe's flow, at the pipe's bore.
  Heard through the Wasm build it must make boost, with a pipe drawn from its outlet or
  without. The RB26, its exhaust compiled, must have one turbo with every cylinder through it. Turned a
  quarter turn about an axis, a turbo's outlet must turn by that, with its pipe still meeting its inlet.
- **Turbos a layout seats.** Switched to any entry of the Cylinders menu, with headers or manifolds and
  one turbo or two, the engine must have one turbo for each bank, its outlet facing rearwards, halfway
  along the engine and out from its bank's ports the way they point, with every port of the bank piped
  straight into its inlet and meeting it. Each must be clear of the engine's outline, of every pipe but its
  own where they meet its flanges, and of the other turbo; and a turbo that has been put somewhere must be
  left there.
- **Pipes into a turbo.** A pipe fitted to a turbo's inlet must run straight in when the inlet is on
  its own line. Off to one side but facing the same way, it must be one smooth bend, an S, ending on the
  flange, leaving the way the pipe was going and meeting the flange square, with no two stations along
  it turning more than 3 degrees, and as long as the bend is. It must curve round to meet an inlet facing
  another way, and survive a link. Moving the turbo must fit only the bend again, leaving the pipe as
  drawn and the bend marked as not editable; moving it back must put the pipe back as it was; changing
  the pipe drawn up to it must bring the bend along; and taking the turbo out must take the bend with it.
- **Pipes drawn to join another.** A pipe joined onto another's open end must finish in one smooth bend,
  marked as not editable, that meets the junction and arrives along the pipe it joins; lengthening either
  pipe must bring the bend along; and taking it off again must take the bend with it.
- **Deleting.** A pipe in the middle must be deleted, what carried on from it left loose where it lay, or
  still fed where another pipe feeds its junction; a turbo whose outlet pipe has pipes after it must not be
  taken out, and a refused delete must change nothing; a pipe
  with nothing after it must still be deleted. Across every preset, deleting any junction or segment that
  is allowed must leave a graph the solver accepts, with no untouched pipe moving.
- **Diameters.** A pipe must taper in a straight line between its two ends, and one given only its inlet
  must be the same all the way along. A bend fitted to a pipe it joins must start at the bore the pipe
  before it ends at and end at the bore of the pipe it joins; a bend into a turbo must end at its inlet's
  bore, and the pipe leaving it start at its outlet's.
- **Loose pipes.** A loose pipe must be where it was put down, in a graph the app accepts, without the
  solver being given it, and must survive a link. Drawn into, it must stay where it was, be fed and given
  to the solver, and the pipe drawn into it must meet its start in a fitted bend.
- **Editing the exhaust while it runs.** Idling free, the F20C, the LT2 and the RB26, their tailpipe made
  5 mm longer, must rev no more than 30 rpm higher over the next second than the same engine left alone:
  the intake keeps its gas when only the exhaust is rebuilt. An edit the solver would build the same, the
  pipes' headings and a turbo's place changed as placing a loose pipe changes them, must leave the gas in
  the pipes as it was.
- **Turning a loose pipe round.** Drawn into at its far end, a loose pipe must lie exactly where it did,
  corner for corner, running the other way, with the same lengths and each bore swapped end for end; and
  attached there it must meet the pipe drawn into it with no pipe added.
- **Joining a pipe's end.** It must make a junction and nothing after it, fixed where the pipe ends; with
  a second pipe into it the app must accept it, and the solver must be given both pipes ending in open air
  until a pipe is drawn on from it, which must carry on the way the joined pipe was going.
- **Moving a junction.** Moved, a junction must be where it was put, the pipe leaving it must start
  there, and both pipes into it must meet it in fitted bends arriving along the pipe leaving; and it must
  survive a link.
- **Swinging a pipe.** Turned about where it starts, a pipe of three bent segments must end up exactly
  where turning it as one piece puts it, every segment keeping its length.
- **The triad.** It must read how far along an arrow the pointer is from any view but straight down
  the axis, land in a square's plane, accumulate a turn past half a revolution without wrapping, and
  still read a ring seen edge on. Its axes must turn with the part, a pipe segment's x running along it
  with its y as near up as it can be. A snapped turn must land on 15-degree steps from the engine's axes,
  squaring up a part set at an odd angle, and a snapped move on 5 mm steps.
- **Where the sound comes from.** On every preset, the app must place every mouth the solver radiates
  from, by its duct, with the intake and casing; the default single's tailpipe must be at the end of
  its megaphone, out to the side, and the LT6's two within 0.25 m of each other in the middle. Through
  the Wasm build, a listener twice as far away must hear the engine half as loud.
- **Headers.** On the LT6 at 8400 rpm, equal-length headers must fill the cylinder more than 1.5
  points past a manifold along the ports.
- **Port injection.** With the fuel cut, neither the manifold nor a runner may hold more than a
  hundredth of a stoichiometric charge's fuel.
- **Valves per cylinder.** The boxer four's torque at full throttle must hold from 3600 to 6200
  rpm on its four-valve head, and fall well away with one valve of each. Each valve must open along
  its own stem in the 3D view, with two per side on a four-valve head.
- **Thermal.** The pipe wall must warm steadily and slowly from cold: two seconds must not get
  close to equilibrium. It must be cooler downstream than at the flange, sit between gas and
  outside air temperature, cool when airflow rises, and warm more slowly when thicker. Editing
  the geometry must keep the wall temperature rather than reset it.
- **Friction split.** Steady flow through the pipe must change by under 15% whether the
  acoustic damping coefficient is 0 or 150. A sound wave must still fade by more than 1 dB per
  pass, and at least 3x faster than the solver's own numerical loss. This checks that damping
  affects sound but not the steady flow.
- **Firing frequency.** At 1800, 3200 and 4800 rpm, every low spectral peak must be a multiple
  of `rpm/120`, and the half-order component must be missing. This confirms a four-stroke
  cycle.
- **Crank algebra.** `crank_at` computes piston position, both its derivatives and the
  cylinder volume from one sin/cos/sqrt. It must match the separate formulas to 12 decimal
  places over the whole cycle, including a connecting rod only just longer than the crank throw,
  and the piston the 3D view draws must sit where it says. Without this, the fast version (used by
  the physics) and the readable one (used by the renderer) could drift apart unnoticed.
- **Thermodynamics.** Mass is conserved with the valves shut. Peak compression pressure is below
  the ideal no-heat-loss ([isentropic](glossary.md#adiabatic-and-isentropic)) limit but within 15% of it. The [polytropic exponent](glossary.md#polytropic-exponent) (how
  pressure scales with volume during compression) is 1.28–1.36. [Choked flow](glossary.md#choked-flow) through a valve
  stops responding to the pressure downstream.
- **Robustness.** No pipe at all, a 10 mm stub, a 400 mm chamber, closed throttle, zero valve
  lift, valves that never close, 12,000 rpm and cold exhaust must all stay finite and within
  full scale.
- **Timbre.** The loudest [octave](glossary.md#octave) must be between 125 and 500 Hz. Energy must fall steadily from
  1 kHz upward. 4 kHz must be at least 14 dB below the peak. Turning throat noise from 0 to full
  must change 4 kHz by under 6 dB and leave 250 Hz alone. These catch an engine whose firing
  frequency and resonances are all correct but whose tonal balance is wrong.
- **Realism.** Consecutive cycles must differ by more than 24% [RMS](glossary.md#rms), and by 1.4x more with
  combustion scatter on than off. Scatter must grow as load falls. Crank speed must ripple
  within each cycle *and*, held at a set speed as a dynamometer holds it, keep that average
  speed to 1% at every throttle.
  Reciprocating inertia must add up to zero work over a cycle. The ground reflection must
  cancel at the frequencies the path-length difference predicts. Twice as far from the engine, the
  ear must hear it half as loud, to within 7.5%. Moving the ear mid-run must not step the output by
  more than 1.5 times as much from one sample to the next as the steady note does, so a turn of the
  view does not click. A mouth the sources do not place must still be heard. Cycle-to-cycle *correlation*
  is deliberately not used: it ignores amplitude, so turning scatter on only moves it
  0.991 → 0.967, where the RMS difference goes 14% → 28%.
- **The twin.** The firing interval must follow `360 + vAngle`, and the override must break
  that link. Both cylinders must fire, stay a fixed number of degrees apart over thousands of
  cycles, and report one snapshot entry each. An evenly firing twin must have almost no half
  order, an uneven one must have some, and 90° must have more than 45°. Through a shared
  collector, moving bank 1's timing must change bank 0's port pressure. Through separate pipes
  with a rigid crank, it must not, to 1e-6. The junction must pass a pulse from one primary
  into the other and into the collector, and conserve mass and energy to 1e-3. Every engine
  preset must run clean, and switching layout or cylinder count mid-run must stay finite. A three with a
  manifold drawn by hand, three junctions a few centimetres apart along a pipe of one runner's bore, must
  rev to its limiter at 32 kHz with nothing diverging, no junction clamping, and no pipe left above three
  atmospheres: gas racing back into a junction faster than sound, from below the junction's pressure, must
  meet a shock rather than pour in without end, and a junction must fill the pipe after it no faster than
  sound, choked. Its note must be within 25% of the level at 48 kHz as it revs through the top half of its
  range.
- **The launch.** The Honda F20C in the S2000, with its own gears and weight, must slip the clutch off
  the line, pull through all six gears, and run 0–60 mph in 4 to 8 s and the quarter mile in 12 to 17 s,
  around the real car's 5.5 s and 14 s; it runs 5.6 s and 14.1 s at 100 mph. The clock must not start
  until the car has moved. A final drive 30% shorter must go into second at a road speed 30% lower, a
  three-speed must stop in third, and a gearbox without gears must not start. A 900 kg car with an LT2
  has far more torque than its tyres can take: traction control must get it to 60 quicker than spinning
  them, and no quicker than 2.4 s, what their grip allows. The engines from real cars must launch through
  those cars' gearboxes, and the Skyline through all four wheels must be quicker to 60 than through the
  rear. The RS 3 must run 0–60 mph in 3.1 to 4.1 s, around the 3.6 s road tests time it at; it runs
  3.5 s and the quarter mile in 11.7 s at 120 mph. The Fiesta ST must run 0–60 mph in 5.7 to 6.9 s,
  around the 6.5 s Ford gives it to 62; it runs 6.1 s and the quarter mile in 14.5 s at 99 mph. Through
  the rear wheels rather than the front it must be quicker to 60 by more than a tenth.

## The reference renders

The tests above check the physics. `tests/parity.rs` checks that the simulation computes exactly what
it computed before, down to the last bit of every sample. `tests/fixtures/scenarios.json` holds a
few dozen scenarios: every preset held at one operating point and idling free, every exhaust preset,
a throttle blip into the rev limiter and back onto the fuel cut, a launch through a cam switch,
edits to a running engine including a change of layout, a drawn exhaust, and another sample rate.
For each it holds the audio as a hash per 10 ms block, and every snapshot taken.

So a change meant to be a refactor or an optimisation is proven to be one, and a change meant to
alter the sound shows exactly where it does. When it is meant to, `PARITY_BLESS=1` writes the new
results into the fixture.

In the web app, `npm test` replays the same file through the Wasm build, through the same `Sim` the
AudioWorklet uses. That is what keeps the web app and the desktop app sounding the same: both builds
must produce these samples exactly. The rest of `npm test` covers the interface: the pipe editor, the
exhaust layout drawn in 3D, and the drawn engine.

## Against a recording

The tests check the physics piece by piece. To check the sound as a whole against a real engine, the
`compare` example holds a preset at one speed and throttle and sets its sound against a recording:

```
cargo run --release -p engine-sim --example compare -- \
    --preset F20C --rpm 6000 --throttle 1 \
    --reference s2000_6000.wav --ref-start 1.5 --ref-seconds 4
```

Both sounds are reduced to two views, each normalised so the recording's gain and distance drop out:

- **Engine orders.** The level of every half order up to 3 kHz, in dB against the firing order.
  This shows whether the right harmonics are there, and in the right balance.
- **Third-octave bands.** Band levels from 25 Hz to 16 kHz, in dB against their total. This shows
  the tonal balance, including broadband noise between the orders.

It also prints each sound's spectral tilt, in dB per octave from 100 Hz to 8 kHz, and its spectral
centroid, along with the RMS and mean absolute difference over each view. `--csv` writes the tables
and `--wav` writes the simulation's render.

The recording's speed is refined from `--rpm` by finding where the firing order's first eight
harmonics are strongest, within 7%, so a tachometer reading a few percent out does not misplace every
order. `--ref-rpm-fixed` turns this off.

### A dyno pull

With `--sweep`, the recording is a pull rather than a hold, and `--rpm` is its speed where the part
used starts:

```
cargo run --release -p engine-sim --example compare -- \
    --preset LT6 --sweep --rpm 2100 \
    --reference z06_dyno.wav --ref-start 36.3 --ref-seconds 10.6
```

The recording is cut into 0.17 s frames, and its speed is tracked from frame to frame: found within 7%
of `--rpm` in the first, then within 3% of the frame before, and median-filtered over five frames.
The speed it found is printed once a second, so it can be checked against a spectrogram. Seeded at the
wrong speed, it follows the wrong lines. The simulation is then pulled on its dyno over the same range
at the same average rate.

Each frame's orders are taken at its own speed, so frames pool by order. The two sounds are compared
over each band of speed, 500 rpm wide by default (`--bin`): each band's overall level against the
pull's mean, the RMS difference of its orders and of its third-octave bands, and its tilt and
centroid. Then they are compared over the whole pull, with the same tables as a hold.

Fed its own pull, rendered with `--wav`, as the recording, the simulation matches itself to 0.3 dB in
its orders and 0.1 dB in its bands. The tracked speed is within 0.2% of the crank's through the
middle of the pull and within 2% at its ends.

The recording must be WAV (PCM 16, 24 or 32-bit, or float) at any sample rate. Stereo is mixed to
mono. A useful recording is steady: a dyno or a held throttle at one speed for a few seconds, out of
the wind, with the speed and throttle written down. By default the simulated listener stands 1.5 m
from the middle of the tailpipes, 45 degrees off the car's rear axis, with the ear 1.2 m above the
ground, and the ground reflection puts notches in the spectrum that depend on those distances. So
place the microphone the same way, or put the simulation's ear where the microphone was with
`--listener x,y,z`, in metres in the drawn engine's frame: x across the crank, y up from it, z along
it, rearwards. A recording from inside the car has no counterpart in the simulation.

## Sources

- Shock tube: [Sod 1978](references.md#sod1978). Open-end reflection:
  [Levine and Schwinger 1948](references.md#levine1948). Pipe resonance:
  [Kinsler et al. 2000](references.md#kinsler2000).
- Thermodynamics, compression and choked valve flow: [Heywood 2018](references.md#heywood2018).
