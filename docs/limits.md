# Known Limits

Where the model is simplified, and by how much.

- **Shock fronts are blurred.** The solver is a [TVD](glossary.md#tvd) scheme (a method that stops false wiggles
  near sharp jumps). It spreads a shock over two or three cells, about 70-105 mm at the
  default 35 mm cells. A real shock is microns thick. The steepening and the harmonics it
  creates are real, but the sharpest edge of an exhaust pulse is blunter than in reality.
- **The intake runners are simplified.** Each is one straight duct, solved like the exhaust but
  with some things left out:
  - The runners' pressure is not radiated, so there is no intake roar.
  - Their walls exchange no heat, so the charge is not warmed on its way in, as a hot port warms a
    real one.
  - The solver carries a single gamma, 1.33, where cool air's is 1.40, so the air in them carries
    sound about 3% slower and their tuning sits about 3% low.
  - Their spent gas and fuel are each tracked as one well-mixed fraction per runner, so gas pushed
    back up a runner comes back spread through it rather than as a slug at the valve.
  - They are solved on the finest grid the sample rate allows. At the lower sample rates that grid is
    coarser, and they ram the charge in less.
  - Each has one length. A two-stage manifold, which switches between long runners for mid-range torque
    and short ones for power, is not modelled.
- **Some effects that make high-output engines strong are missing.** Direct injection cools the
  charge as the fuel evaporates, variable cam timing moves the cam for each speed, and a rich
  mixture at full throttle adds a few percent of power. None is modelled. A 6.2 litre V8 in the
  proportions of a Chevrolet LT2 comes out at about 600 N·m and 460 hp, against the real engine’s
  637 N·m and 495 hp. The LT6, the 8600 rpm flat-plane V8, comes out at 650 hp at 8400 rpm, against 670,
  but at 535 N·m at 6300 against 624: its runners are tuned for the top end, where the real engine's
  manifold switches to its long runners below it. Its cam, rods, runners and headers are estimates,
  since they are not published.
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
- **Port injection only, and no fuel film.** The fuel is injected in each runner as vapour, straight
  into the air it draws. A real port injector wets the port walls, and that film takes a few cycles
  to follow a change of throttle. Direct injection, into the cylinder, is not modelled either. There
  is no knock model, so an over-advanced spark just loses power.

## Sources

- Shock smearing in TVD schemes: [Harten 1983](references.md#harten1983),
  [Toro 2009](references.md#toro2009).
- Specific heat, dissociation and flame temperature: [Heywood 2018](references.md#heywood2018).
