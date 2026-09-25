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
  own delay, distance loss and far-field radiation. But they share one ground reflection and one
  air-absorption path, and are assumed to be at the same height. That is close, since tailpipes
  usually sit at about one height. The listener's angle is fixed at 45 degrees and is not a
  control.
- **A few constants are tuned, not derived.** The linear acoustic damping coefficient is fitted
  to measured duct decay rates. It is the one openly empirical constant in the acoustics. The
  only others chosen by ear are the two structure-borne noise levels and the turbulence
  intensity. All are labelled as such in the source.
- **Cylinder wall temperature is fixed at 450 K.** The *pipe* wall temperature is simulated, but
  the cylinder's is not. So a cold engine does not burn or lose heat differently.
- **Gas properties are approximate.** [Specific heat](glossary.md#specific-heat) rises linearly with temperature, fitted
  to gamma 1.35 at 500 K and 1.28 at 1800 K, and is the same for fresh charge and burned gas. There
  is no [dissociation](glossary.md#dissociation) (hot gas molecules splitting and absorbing energy above roughly 2200 K).
  Peak flame temperature comes out at about 2700 K at full throttle, inside the 2500-2900 K real
  engines measure, but toward the top of it. The exhaust pipe keeps a single gamma of 1.33.
- **A chamber's cross-wise modes are linear and partly simplified.** They're added to the 1D
  solver as linear oscillators, so a very loud pulse doesn't steepen across the can as it does
  along it. The pipes can only be offset along the can's width, which is where they drive the
  lowest modes, not along its height. Mode damping is one fixed ratio (ζ = 0.02, a Q of 25)
  for wall and visco-thermal loss, chosen from the range measured muffler cavities show, not
  derived. Loss into the pipes is not part of it; that comes out of the coupling itself. The
  panels of a flat can are rigid, so it doesn't drone the way thin sheet metal does.
- **Junctions are balanced approximately.** Each junction is balanced by two [Newton](glossary.md#newtons-method)
  corrections (refinement steps), not solved fully. Its mass flow balance is close but not
  exact, and the remaining imbalance is reported as a diagnostic.
- **A closed throttle still leaks.** It is modelled as 22 kPa manifold pressure, not a perfect
  seal. That is realistic for throttle-plate clearance plus idle bypass.
- **The burn duration's model has two calibrated constants.** How the burn splits between the
  flame front's travel and the burn-out behind it (0.35 burn-out at the reference state) and the
  mixture speed below which the spark starts to fail (half a stoichiometric one's) are chosen, not
  derived. The laminar speed's correction for leftover exhaust is measured only up to about 30%,
  and is held at a floor above that.
- **Rich mixtures make no extra power.** A real engine peaks near λ 0.85-0.9, from fuel evaporation
  cooling the charge and the extra gas molecules rich combustion produces. Neither is modelled,
  so here torque is flat, or falls slightly, rich of stoichiometric.
- **The exhaust carries no chemistry.** Unburned fuel from a rich mixture, a misfire or the rev
  limiter goes down the pipe as ordinary exhaust gas. So there is no afterfire or popping on the
  overrun.
- **The fuel is metered at the throttle, not in each port.** It reaches the cylinders through
  the manifold, a cycle or so later. A port injector's fuel film on the port walls delays it about
  as much. There is no knock model, so an over-advanced spark just loses power.

## Sources

- Shock smearing in TVD schemes: [Harten 1983](references.md#harten1983),
  [Toro 2009](references.md#toro2009).
- Specific heat, dissociation and flame temperature: [Heywood 2018](references.md#heywood2018).
