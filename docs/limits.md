# Known Limits

Where the model is simplified, and by how much.

- **Shock fronts are blurred.** The solver is a [TVD](glossary.md#tvd) scheme (a method that stops false wiggles
  near sharp jumps). It spreads a shock over two or three cells, about 70-105 mm at the
  default 35 mm cells. A real shock is microns thick. The steepening and the harmonics it
  creates are real, but the sharpest edge of an exhaust pulse is blunter than in reality.
- **The intake runners are simplified.** Each is one straight duct, solved like the exhaust but
  with some things left out:
  - The runners' pressure is not radiated, so there is no intake roar.
  - What their waves lose to turbulence, the bend into the port and the valve seat is not worked out
    one loss at a time: it is one factor, five times the boundary-layer loss of a smooth tube, chosen
    so a wave dies away over a few cycles as it does in a real runner.
  - Their walls exchange no heat, so the charge is not warmed on its way in, as a hot port warms a
    real one.
  - The solver carries a single gamma, 1.33, where cool air's is 1.40, so the air in them carries
    sound about 3% slower and their tuning sits about 3% low.
  - Their spent gas and fuel are each tracked as one well-mixed fraction per runner, so gas pushed
    back up a runner comes back spread through it rather than as a slug at the valve.
  - They are solved on the finest grid the sample rate allows. At the lower sample rates that grid is
    coarser, and they ram the charge in less.
  - A two-stage intake switches between its two sets of runners at once. A real flap takes a few tens
    of milliseconds to move, and is part open on the way.
- **Some effects that make high-output engines strong are missing.** Direct injection cools the
  charge as the fuel evaporates, and a rich mixture at full throttle adds a few percent of power.
  Neither is modelled. A naturally aspirated engine copied from a real one comes out within about 5%
  of its published torque and power, but its cams, cam maps, rods, runners and headers are estimates
  wherever they are not published.
- **Cam profile switching is on speed alone.** A real engine's control unit also checks the load, the
  oil pressure and the road speed before it engages the high-speed lobes; here it switches on engine
  speed only, and at once.
- **The variable valve timing map is set by hand.** A production map is calibrated on a dyno, point by
  point, for torque, economy and emissions together; this one is four cam positions at two speeds,
  blended in a straight line between them, and moves the cams under load only. What cam timing does
  to part-load economy and emissions, much of what a real engine uses it for, is not modelled.
- **The turbine is a restriction without volume.** It is solved in the exhaust, between the
  junction's two sides, but as a nozzle obeying Stodola's law at every instant: it has no housing
  volume, and its efficiency does not vary with its speed or its pressure ratio. Several turbos share one
  lumped shaft, so they cannot spool apart from each other.
- **The rest of the turbocharger is lumped.** The compressor map is one generic shape scaled to the
  turbo's size, not a real turbo's map. There is no knock, so no boost is too much and the spark is not
  retarded on boost, as a real engine's would be. The shaft has no speed limit, so a turbo too small
  for the engine, at its choke, spins as fast as its turbine can drive it, well past the speed a real
  wheel would survive. Where a real engine's own compressor map is not published, its turbos are sized
  from a similar turbo's map and the flow the engine is reckoned to need, so a turbocharged engine can
  come out stronger than the one it copies.
- **Some of the turbocharger's sounds have chosen levels.** The whine is the blades modulating the
  inlet's flow by a depth chosen for it, not solved from the flow through the blades; the turbine's
  whistle is likewise a chosen pulsation of its flow; a stalled compressor's turbulence has a chosen
  intensity; and the wastegate's rattle is an impact on two modes. The blow-off valve's and the reverse
  flow's jet noise follow Lighthill's law from the simulated jets, but its constant is taken from
  measured jets, not derived, and a recirculating valve's share that gets out through the ducting is
  chosen.
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
- **The launch car is a point mass on a flat strip.** Its tyres' grip is one friction coefficient,
  whatever the speed, with no suspension, tyre heat or surface to change it. Where a real car's grip is
  known it is from a skidpad, which measures it cornering, raised by a fixed 7% for grip driving in a
  straight line; the true difference depends on the tyre. The weight moving onto the rear wheels is one
  fixed share per g, 0.18, the same for every car. Its drag area is fixed at 0.6 m², and its wheels,
  tyres and half-shafts are one 3 kg·m² body. The gearbox and clutch have no inertia of their own, and
  the engine's rotating inertia is an estimate where it is not published: through the low gears it
  weighs as much as a few hundred kilograms more car. Traction control is idealised: it knows the tyres'
  slip exactly, and cuts the spark with no delay beyond the next firing. Makers' times are often set on
  prepared surfaces, and real timeslips also depend on the driver and the air, so the times compare one
  engine or gearing with another better than they predict a real car's.

## Sources

- Shock smearing in TVD schemes: [Harten 1983](references.md#harten1983),
  [Toro 2009](references.md#toro2009).
- Specific heat, dissociation and flame temperature: [Heywood 2018](references.md#heywood2018).
