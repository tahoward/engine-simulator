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
  at 6800. The 3S-GTE, four cylinders on one turbo, on its 0.5 bar must make within 10% of the North
  American engine's rated 271 N·m at 3200 rpm and 200 hp at 6000; with compressors too small for it, passing 0.12 kg/s each, it must make less power at 7900
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
- **Placing turbos.** A turbo put down on an open pipe end must attach it and add nothing at its outlet,
  in a graph the solver accepts, the solver alone given a vent to the air at its outlet flange; the pipe
  must meet its inlet flange and a pipe drawn from its outlet start at its outlet flange, with no junction
  fitting drawn; its inlet must be offered to draw to, its outlet to draw from until a pipe is drawn from
  it, and its node not as a junction; tidying must leave its pipes as they are; taking it out must leave the
  pipe open again; moved, the pipe must follow it and still meet its inlet; and it must survive a link.
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
  cancel at the frequencies the path-length difference predicts. Cycle-to-cycle *correlation*
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
  rear.

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

## Sources

- Shock tube: [Sod 1978](references.md#sod1978). Open-end reflection:
  [Levine and Schwinger 1948](references.md#levine1948). Pipe resonance:
  [Kinsler et al. 2000](references.md#kinsler2000).
- Thermodynamics, compression and choked valve flow: [Heywood 2018](references.md#heywood2018).
