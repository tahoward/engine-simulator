# Known Limits

Where the model is simplified, and by how much.

- **Shock fronts are blurred.** The solver is a [TVD](glossary.md#tvd) scheme (a method that stops false wiggles
  near sharp jumps). It spreads a shock over two or three cells, about 70-105 mm at the
  default 35 mm cells. A real shock is microns thick. The steepening and the harmonics it
  creates are real, but the sharpest edge of an exhaust pulse is blunter than in reality.
- **The intake is a single volume, not a pipe.** It holds gas that flows back during valve
  overlap and returns it later, so leftover exhaust in the cylinder is tracked correctly.
  Intake runners and an airbox could use the same duct solver as the exhaust. Until they do,
  there is no intake roar.
- **A sealed cavity has a tiny built-in growth.** In a nearly lossless *sealed* cavity the
  solver has a small [second-order](glossary.md#order-of-accuracy) error that grows at about 1.4 /s. Normal damping is 150 /s,
  about a hundred times larger, so it stays suppressed. It is not zero.
- **Engine body noise is simplified.** It uses four lumped modes, not a vibration model of a
  real casting. It also comes from the same point as the exhaust, not from its own position
  with its own direction pattern.
- **Tailpipe positions are partly simplified.** Mouths are spaced along a line, each with its
  own delay and distance loss. But they share one ground reflection and one air-absorption
  path, and are assumed to be at the same height. That is exact for the differentiator (it is
  linear, and the mouths are added together before it) and close for the rest, since tailpipes
  usually sit at about one height. The listener's angle is fixed at 45 degrees and is not a
  control.
- **A few constants are tuned, not derived.** The linear acoustic damping coefficient is fitted
  to measured duct decay rates. It is the one openly empirical constant in the acoustics. The
  only others chosen by ear are the two structure-borne noise levels and the turbulence
  intensity. All are labelled as such in the source.
- **Cylinder wall temperature is fixed at 450 K.** The *pipe* wall temperature is simulated, but
  the cylinder's is not. So a cold engine does not burn or lose heat differently.
- **Peak flame temperature is too high.** [Specific heat](glossary.md#specific-heat) is constant and there is no
  [dissociation](glossary.md#dissociation) (hot gas molecules splitting and absorbing energy). So peak flame temperature
  comes out at about 3150-3260 K, where real engines measure 2500-2900 K. Real `cv` rises from
  about 820 J/(kg K) cold to over 1200 hot, and dissociation absorbs energy above roughly
  2200 K. Peak *pressures* are in the right range (66-88 bar at full load) because the trapped
  mass is right. So the error shows in temperature, not in the sound.
- **Junctions are balanced approximately.** Each junction is balanced by two [Newton](glossary.md#newtons-method)
  corrections (refinement steps), not solved fully. Its mass flow balance is close but not
  exact, and the remaining imbalance is reported as a diagnostic.
- **A closed throttle still leaks.** It is modelled as 22 kPa manifold pressure, not a perfect
  seal. That is realistic for throttle-plate clearance plus idle bypass. But there is no fuel
  cut when coasting, so the engine keeps firing weakly instead of being turned over with no
  combustion.

## Sources

- Shock smearing in TVD schemes: [Harten 1983](references.md#harten1983),
  [Toro 2009](references.md#toro2009).
- Specific heat, dissociation and flame temperature: [Heywood 2018](references.md#heywood2018).
