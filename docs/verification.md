# Verification

`npm test` runs the test suite against the physics core. The core is pure code with no
`AudioContext`, so every test runs headless in Node. Each item below says what is checked and
why it matters.

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
- **Crank algebra.** `crankState` computes piston position, both its derivatives and the
  cylinder volume from one sin/cos/sqrt. It must match the four separate functions to 12
  decimal places over the whole cycle, including a connecting rod only just longer than the
  crank throw. Without this, the fast version (used by the physics) and the readable version
  (used by the renderer) could drift apart unnoticed.
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
  within each cycle *and* still hold the set average speed to 1% at every throttle.
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
  preset must run clean, and switching layout or cylinder count mid-run must stay finite.

## Sources

- Shock tube: [Sod 1978](references.md#sod1978). Open-end reflection:
  [Levine and Schwinger 1948](references.md#levine1948). Pipe resonance:
  [Kinsler et al. 2000](references.md#kinsler2000).
- Thermodynamics, compression and choked valve flow: [Heywood 2018](references.md#heywood2018).
