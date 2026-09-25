# The Engine

How the cylinder model works, why it doesn't sound like a loop, and what adding cylinders does.

## The cylinder

The cylinder is modelled as one pocket of gas. Its walls move with the piston, and gas enters
and leaves through the valves. The model tracks the gas's **mass and internal energy**.
Temperature and pressure are worked out from those two.

Tracking energy rather than temperature keeps the model stable. During the exhaust stroke, two
large effects almost cancel: the piston doing work on the gas, and the energy carried out with
the escaping gas. That is why the temperature stays steady while gas is pushed out. Updating
temperature directly would divide the tiny leftover by a mass that is itself shrinking toward
zero, and the result would blow up near exhaust valve closing at part throttle. With energy as
the state, the division by mass only happens when temperature is read, and by then mass and
energy have shrunk together.

Other details of the model:

- **Flow direction counts.** Gas pushed back into the cylinder through the exhaust valve (during
  [valve overlap](glossary.md#valve-overlap), when both valves are briefly open) arrives at the exhaust port's temperature,
  not the cylinder's. Each valve's energy flow is handled with this in mind.
- **Combustion uses a [Wiebe function](glossary.md#wiebe-function)**, a standard curve for how fast fuel burns. The total heat
  released depends on how much fuel, and air to burn it, is trapped when the intake valve closes.
  So the throttle works the way a real one does, by limiting how much air gets in. Engine load
  comes out of the mass bookkeeping rather than being applied as a volume knob. How long the burn
  takes is worked out for each charge; see [The flame](#the-flame).
- **A head has two or four valves.** With four, there are two intake and two exhaust valves. The
  flow through them is the flow through one, twice over. Two valves open more of the cylinder than
  one of the same total area: at the same lift, √2 as much of the gap around the valve head. That
  is what lets a four-valve engine breathe at high rpm, and the presets with overhead cams have
  four-valve heads. Their valves are sized as a typical four-valve head's for the bore, not taken
  from each real engine: each intake valve is 0.40 of the bore and each exhaust valve 0.34. Given
  one valve of each instead, the boxer four's torque falls by more than a third from 3600 to 6200 rpm. With
  two of each it holds 90-99% volumetric efficiency right up to its rev limit, as do the other
  four-valve presets.
- **Heat loss to the walls** uses the [Woschni](glossary.md#woschni-model) model. This puts the compression curve at a realistic
  [polytropic exponent](glossary.md#polytropic-exponent) near 1.33 (a measure of how pressure rises as gas is squeezed), instead of
  the 1.35 you'd get with no heat loss.
- **Reverse flow is kept.** Gas can flow from the pipe back into the cylinder during valve overlap.
  That is exactly the effect a tuned exhaust relies on.


## The flame

### How long the burn takes

The **Burn duration** control is the burn at a reference state: a [stoichiometric](glossary.md#air-fuel-ratio-and-lambda)
charge at 13 bar and 650 K when the spark fires, with 4% leftover exhaust, at 10 m/s
[mean piston speed](glossary.md#mean-piston-speed). That is roughly any naturally aspirated engine at
full throttle. Each cycle's burn is worked out from there, from how fast its own charge burns.

The flame starts from the [laminar burning velocity](glossary.md#laminar-burning-velocity): how fast
the mixture burns in still gas. It rises steeply with temperature, falls a little with pressure,
peaks slightly rich, and drops sharply with leftover exhaust in the charge. In the cylinder the
flame is carried by turbulence, which scales with piston speed. The burn has two parts:

- **The flame front sweeps across the chamber** at the turbulence speed plus the laminar speed.
  Turbulence rises with rpm, so in crank degrees this part hardly changes with speed.
- **Each pocket of charge the front has passed then burns out** at the laminar speed. This part
  grows when the laminar speed falls: at part throttle, with leftover exhaust, and lean.

On the default single this gives the burns below, with the spark the advance map (below) picks
for each. The single's set advance is 25°.

```
                              burn    spark
full throttle, 1000 rpm        39°    17° BTDC
full throttle, 3200 rpm        54°    25°
full throttle, 6000 rpm        66°    30°     six times the speed, not six times the degrees
manifold at 0.4 bar, 3200      69°    32°
λ 1.3, full throttle, 3200     66°    31°
V8 idling at 900 rpm           84°    40°     a quarter of the charge is leftover exhaust
```

The burn is capped at 150°. A charge that slow is still burning when the exhaust valve opens.

### The advance map

Because the burn no longer takes a fixed number of degrees, a fixed spark would be wrong almost
everywhere. At low rpm the burn is quick, so the pressure would peak before top dead centre and
push against the piston. A four stuck at 450 rpm under load would then make no power whatever the
throttle did. At part throttle the burn is slow and would peak too late to do much work.

So the spark follows an advance map, as a real ignition system does. The spark moves so that each
charge reaches half burned where a reference burn would. The **Ignition advance** control is the
timing for that reference burn. The map retards the spark at low rpm and advances it at part
throttle and high rpm, and keeps it between top dead centre and 50° before it. The spec's
`advanceCurve` switch turns the map off and fixes the spark at the set angle.

### The mixture

Fuel and air are tracked separately, through the manifold and the cylinder. The fuel is metered in
with the air at the throttle, in proportion to it, at the **Mixture** control's
[λ](glossary.md#air-fuel-ratio-and-lambda). Gas pushed back up the intake carries its fuel with it,
and brings it back next cycle.

- **Lean**, each charge carries less fuel, so it releases less heat, and it burns slower.
- **Rich**, the oxygen runs out first. The extra fuel goes out unburned, so torque stays level with
  stoichiometric rather than rising.
- **The spark can fail to light the charge.** Excess air dilutes the charge the way leftover exhaust
  does, and the two are counted together, so a lean idle misfires sooner than a lean charge at full
  throttle. Misfires also start once the mixture itself burns at under half the speed of a
  stoichiometric one, near λ 1.5. They become certain at the flammability limit, near λ 2.15.

**Overrun fuel cut.** With the throttle shut above 1500 rpm the fuel stops, as it does on a
fuel-injected engine. The engine is then turned over by its load, pumping air. The fuel comes back
below 1200 rpm or as soon as the throttle opens. The manifold empties of fuel within a cycle or so.
Turn **Overrun fuel cut** off for a carburettor's behaviour, which keeps feeding fuel with the air
that leaks past the throttle, so the engine keeps firing weakly.

## Why it does not sound looped

A perfectly repeating engine sounds synthetic. Real engines vary from cycle to cycle, their cranks
never turn at an exact rate, and you hear them outdoors. Four effects model this.

**Combustion varies from cycle to cycle.** How the flame starts depends on the random turbulence
at the spark plug when it fires, so no two cycles burn the same. Burn duration, heat release and
ignition delay are picked fresh for each cycle when the intake valve closes. Burn rate and heat
release are linked, because a slow-starting flame also burns less completely. The variation grows
as the charge gets thinner. Cycle-to-cycle variation in work output is about 1.9% at full load and
rises to about 9.9% near idle, which matches published single-cylinder data. Consecutive cycles'
waveforms differ by about 28%, and by 117% at idle, where some cycles nearly misfire.

**The crank speeds up and slows down within each cycle.** Gas pressure and the moving parts push
the crank unevenly. On a big single, the speed swings by a few percent even on a [dynamometer](glossary.md#dynamometer)
(a test rig that holds the engine at a set speed). Only the *fluctuating* part of the torque
should do this, because whatever holds the speed absorbs the average. So the model tracks the
average torque and subtracts it first, which keeps the engine at the speed you set.

**The piston's weight shakes the crank.** The piston and the small end of the rod push on the crank
as hard as the gas does, at twice crank frequency. Over a full cycle this averages to zero, so it
doesn't change how fast the engine runs, only how unevenly. That unevenness is most of a big
single's character. The piston motion uses exact formulas for its first and second derivatives,
checked against numerical estimates.

**Engine noise comes from combustion, not a timer.** The block and head ring because the pressure
rise hits them. So the model drives four structural vibration modes from `dp/dt`, the rate of
pressure rise. Advance the ignition or shorten the burn, and the engine sounds harsher with no
extra rule: peak `dp/dt` goes from 1.6 GPa/s with a 90° burn to 14 GPa/s with a 20° burn.
[Piston slap](glossary.md#piston-slap) (the piston rocking against the bore) scales with cylinder pressure at [TDC](glossary.md#tdc) (top dead
centre, the piston's highest point). So it is loud under load and almost gone when coasting. All
this mechanical noise sits about 18 dB below an open header. That is why you only hear it once a
muffler has quietened the exhaust.

**The listener is outdoors.** Sound reaches you directly and also bounces off the ground. The
bounce arrives later and duller, and mixing it with the direct sound cancels some frequencies.
At 1.5 m, the first cancelled frequency is near 400 Hz, right in the middle of the engine note.
Air also absorbs high frequencies, so a distant engine sounds muffled, not just quieter. All of
this changes when you change ear height, exhaust height or ground surface.


## More cylinders

Every cylinder uses the same model on the same crank. So, for sound purposes, an engine layout is
just two lists: **when each cylinder fires**, and **which bank it belongs to**.

For more than two cylinders, both lists are taken from the real engines, not calculated. Real
[firing orders](glossary.md#firing-order) are chosen for crankshaft balance and bearing loads, and no formula recovers them.
So each engine has its own entry. The twin is the exception: there, the shared crankpin really
does set the firing interval, and you can hear the V angle in it.

```
              fires at                          banks         each bank fires
single        0                                  A            every 720
V-twin 45°    0, 405                             A B          405 / 315 apart
inline three  0, 240, 480                         A A A        every 240
inline four   0, 180, 360, 540                   A A A A      every 180
inline five   0, 144, 288, 432, 576              A A A A A    every 144
inline six    0, 120, 240, 360, 480, 600         A A A A A A  every 120
V6 60°        0, 120, 240, 360, 480, 600         A B A B A B  every 240
boxer four    0, 180, 360, 540                   A A B B      180 / 540 apart
boxer six     0, 120, 240, 360, 480, 600         A B A B A B  every 240
V8 flatplane  0, 90, 180, 270, 360, 450, 540, 630  A B A B A B A B    every 180, even
V8 crossplane 0, 90, 180, 270, 360, 450, 540, 630  A B A A B A B B    180-90-180-270, uneven
```

The two V8s are the most interesting case. **Both fire at exactly the same eight crank angles.**
Through one shared collector, they sound almost the same. Give each bank its own collector, and
they sound completely different. A flatplane bank gets four evenly spaced pulses. A [crossplane](glossary.md#crossplane-and-flatplane-cranks)
bank gets an uneven 180-90-180-270. That is the difference between the muscle-car burble and the
Ferrari shriek. Measured at each bank's collector inlet, the crossplane has over five times the
flatplane's second-order energy. The flatplane concentrates its energy on multiples of its fourth
order: its fourth-to-third-order ratio is more than twenty times the crossplane's. No burble was
added by hand. It comes purely from the pattern of pulses arriving at a piece of pipe.

### What more cylinders actually do to the sound

**More cylinders raise the pitch, a lot.** The firing frequency is `cylinders x rpm/120`. At 3000
rpm, a single fires at 25 Hz, a twin at 50, a four at 100 and a V8 at 200. That is three octaves
from one end to the other, and it is the main thing you hear.

**Even firing also removes the lower notes.** When cylinders fire at evenly spaced intervals, their
pulses cancel every order that is not a multiple of the cylinder count. (An ["order"](glossary.md#order) here is a
multiple of `rpm/120`.) Adding copies of a pulse train shifted by 1/N of a cycle acts as a [comb
filter](glossary.md#comb-filter). Measured through one collector:

```
             order  1     2     3     4     5     6     7     8
single             2e3   2e3   5e3   1e4   1e4   4e3   3e3   3e2     all of them
twin, even         1e-2  8e3   4e-1  2e5   1e0   3e4   5e-1  8e3     even orders only
inline four        9e-2  6e-1  1e0   3e5   2e0   9e-1  1e0   5e3     4th and 8th only
V8                 2e-1  2e0   4e0   9e-1  5e0   4e0   1e0   3e2     8th only
```

So a multi-cylinder engine doesn't just add a higher note. It *removes* everything below it, by a
factor of 100,000 to 1,000,000. No low-frequency content is left to make it sound deep. That is why
a four sounds smooth and buzzy and a single sounds lumpy. Real engines do exactly this.

Two effects follow from this:

- **The crossplane V8 is the exception, and that is its whole appeal.** Its uneven bank firing
  breaks the cancellation, so energy survives *below* the firing [order](glossary.md#order): 7.3% of the total for the
  crossplane, against 0.5% for the flatplane. The burble is exactly that low-order content.
- **With only one strong order, pipe tuning matters much more.** A primary pipe with a dead spot
  at the firing frequency only dulls a single, but it can make a V8 nearly vanish, because the V8's
  one surviving order lands in it. In the table, the 0.4 m primary used has such a dead spot at
  200 Hz: the single's 8th order is ten times lower than its neighbours. Re-check the sound after
  editing a multi-cylinder pipe.

More cylinders do not add brightness. Compare engines at the same *firing frequency* instead of
the same rpm (a single at 12,000 rpm against a V8 at 1500), and more cylinders sounds *darker*: the
[spectral centroid](glossary.md#spectral-centroid) (the "average" frequency) falls from 735 Hz to 119 Hz.

In a real engine the cancellation is strong but never perfect, because nothing is perfectly
symmetrical. Two parts of the model keep it that way:

- **Each tailpipe is heard from its own place.** `mouthSpacing` spaces the pipe outlets apart, and
  each gets its own travel delay and distance loss to the listener. Real tailpipes are about a metre
  apart, which at 187 Hz is most of a wavelength. Without this, a flatplane V8's two banks, which
  fire exactly out of step, would cancel each other's firing order and the engine would jump up an
  octave. The listener stands off to one side, at 45 degrees. Outlets placed symmetrically in front
  of the listener would all be the same distance away, and the spacing would change nothing.
- **No two cylinders breathe quite alike.** `cylinderSpread` varies the runner pressure each
  cylinder sees by a few percent, standing in for unequal runner lengths, valve seats and fuelling.
  The low orders sit at about −35 dB for a four and −23 dB for a V8, within the 20 to 35 dB range of
  real engines, so there is a rumble under the firing note. Pressure is varied rather than intake
  valve area because it changes how much gas is trapped; valve area only changes how fast the
  cylinder fills.

**The crank layout can be worked back out from the firing order**, which is a useful check on the
data. Two cylinders share a crankpin when the second fires one bank angle after the first, or one
bank angle plus a full revolution. Asking `crankPins` to pair them up gives four pins 90° apart for
the crossplane, and four pins in one plane for the flatplane. That is exactly what those cranks are
named for. A 270° parallel twin correctly comes out with two pins, because no shared pin can give
that interval. That is what the firing-offset override is for.

The gap is not always `vAngle + 360`, as it is on a V-twin: the camshaft picks, for each pin, which
of the two TDCs is the firing one. That is what allows even 90° firing with uneven banks.

## The twin

A second cylinder is just another copy of the same cylinder model on the same crank. The
interesting part is that **the two cylinders affect each other in two different ways**: through
the exhaust and through the crank.

The firing interval is not a free setting. Both rods share one crankpin, so the second cylinder
reaches firing TDC `vAngle` degrees of crank rotation after the first. Over a 720° four-stroke
cycle, that puts the two firings `360 + vAngle` and `360 - vAngle` apart:

```
vAngle      fires at        character
  0°        360 / 360       parallel twin, evenly spaced
 45°        405 / 315       Harley: the lopsided potato-potato idle
 90°        450 / 270       Ducati L-twin, more uneven still
```

This interval sets the sound, and it comes from the geometry, not a setting. An even-firing twin
has almost **no half-order component**. Both firings land in the same place each revolution, so
the firing frequency doubles and the `rpm/120` line nearly disappears (measured at four orders of
magnitude below the full order). An uneven twin puts energy back at the half order, more so the
more uneven it is: 90° has about 4x the half-order content of 45°. That one frequency line is most
of what makes a Harley sound like a Harley, and nobody added it by hand.

A separate **firing offset** override covers intervals a shared pin can't produce, such as a 270°
parallel twin, which needs a crank with two pins 90° apart. Changing it shifts the timing of the
running cylinders instead of rebuilding them, so the engine doesn't restart while you drag the
slider.

### The junction

**Wherever pipes meet, they share one pressure at the junction.** This covers a 2-into-1's two
primaries and its collector, and each link of a manifold. Each pipe end supplies the wave heading
*toward* the junction, `a_k`. Writing the returning wave as `b_k = p_J - a_k` makes the mass flow
into the junction from that pipe `A_k (2 a_k - p_J) / c_k`. Conservation of mass then gives a first
estimate of the shared pressure directly:

```
p_J = sum(2 A_k a_k / c_k) / sum(A_k / c_k)
```

That estimate is only approximate when big pulses hit the collector from both sides, so it is just
the starting point. Two [Newton steps](glossary.md#newtons-method) (a standard way to refine an estimate) then balance the mass
flows, using an [HLLC](glossary.md#hllc) Riemann solve (a fast calculation of how two gas states meet) for each branch.
Any imbalance left over is reported as a diagnostic rather than corrected. The solve runs in the
Wasm kernel, with a TypeScript fallback.

It behaves correctly in simple cases: two pipes reduce to a plain change in pipe width, and one pipe
to a closed end. What it adds is cross-talk. Each cylinder's exhaust pulse reaches the junction, and
part of it travels **up the other primary**. Depending on the firing interval, that either helps
clear the other cylinder or blocks it. Real exhaust tuning uses this, and here it emerges on its own.
Measured with the crank held perfectly steady, so the exhaust is the only link: moving the second
cylinder's firing from 360° to 450° changes the first cylinder's *own* port pressure by 127% [RMS](glossary.md#rms)
through a shared collector, and by nothing (1e-8) through separate pipes.

The crankshaft is the second link, and it is easy to forget. Torque pulses from one cylinder speed
up and slow down the shared crank, so the other cylinder's timing shifts even when the exhausts are
completely separate. With separate pipes and a standard flywheel, that link alone changes the first
cylinder by 4%. A twin is never two independent singles, whatever exhaust you fit.

## Sources

- Cylinder model, Wiebe burn, valve flow and cycle-to-cycle variation:
  [Heywood 2018](references.md#heywood2018).
- In-cylinder heat loss: [Woschni 1967](references.md#woschni1967).
- Laminar burning velocity: [Rhodes and Keck 1985](references.md#rhodes1985), with Heywood's
  gasoline constants; turbulent entrainment and burn-up: [Blizard and Keck 1974](references.md#blizard1974).
- Junction solve: [Toro 2009](references.md#toro2009).
- Outdoor listener, ground reflection and air absorption: [Kinsler et al. 2000](references.md#kinsler2000).
