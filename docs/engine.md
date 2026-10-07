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
  one valve of each instead, the Subaru FA20D's torque falls by more than a third from 3600 to 6200 rpm. With
  two of each it holds 90-99% volumetric efficiency right up to its rev limit, as do the other
  four-valve presets.
- **The intake has runners.** Each cylinder draws through its own runner, a duct from the plenum
  to the intake valve, solved with the same gas dynamics as the exhaust. See [The intake](#the-intake).
- **Heat loss to the walls** uses the [Woschni](glossary.md#woschni-model) model. This puts the compression curve at a realistic
  [polytropic exponent](glossary.md#polytropic-exponent) near 1.33 (a measure of how pressure rises as gas is squeezed), instead of
  the 1.35 you'd get with no heat loss.
- **Reverse flow is kept.** Gas can flow from the pipe back into the cylinder during valve overlap.
  That is exactly the effect a tuned exhaust relies on.
- **A wide-open valve flows steadily.** The flow through a valve goes as the square root of the
  pressure across it, so when the pressures either side are nearly equal (through the exhaust
  stroke, or while the intake valve is open) a small change in either swings the flow a long way.
  Each step takes the flow at the port pressure that flow will leave behind at the end of the step,
  not the one before it; taken at the one before, it overshoots, flows back the next sample and out
  again the one after.


## The intake

Each cylinder breathes through a runner: a duct from the intake plenum to its intake valve, solved
with the same gas dynamics as the exhaust. The plenum end is open onto the plenum's gas, the way an
exhaust mouth is open onto the air outside.

The air in a runner has mass. The falling piston sets it moving, and its momentum keeps it flowing
into the cylinder after bottom dead centre, even once the cylinder's pressure is above the
plenum's. Its pressure waves travel up to the plenum and back, arriving in step with the next intake
stroke at some speeds and out of step at others. Together these *ram* the charge in: at the speed a
runner is tuned for, the cylinder fills to about 100% of its volume at ambient density. Without them,
an engine that closes its intake 70° after bottom dead centre just pushes the charge back out.

On a 6.2 litre V8 with that late intake closing, at full throttle:

```
                          4000   4500   4950   5400   5800 rpm
tuned 450 mm runner        95%   101%    99%    96%    94%     volumetric efficiency
80 mm stub                 83%    83%    86%    84%    83%
```

A runner's length sets where it is tuned. Long ones make torque low down and short ones make power
at the top, and on that V8 an 800 mm runner makes more torque than a 250 mm one at 3500 rpm and less
at 6450. **Intake runner length** and **Intake runner bore** set them. Left on auto, the bore passes
the intake valves' area, a little narrowed, and the length is tuned to three quarters of the rev
limit: its quarter-wave resonance is 2.3 times the crank speed there.

**A two-stage intake has both.** With **Short runner length** set, each cylinder has a second, shorter
path to the plenum, and the manifold switches to it at **Switch to short runners at**, as a real one's
flap opens. It switches back 150 rpm lower, so it does not flap back and forth at the switch speed.
Both sets are solved, on the same grid; the gas in the one being left, measured from the valve, is
laid onto the other, so the charge in the ports and its flow carry on through the switch. On the LT6,
with 390 mm runners switching to 310 mm ones at 7600 rpm, on a dyno pull:

```
                          3000   4500   5500   6300   7000   7800   8400 rpm      crank torque, N·m
310 mm runners only        425    496    578    556    569    565    568
two-stage                  432    550    570    619    611    568    567
```

The runners are solved on the finest grid the sample rate allows, 35 mm at 48 kHz, even where the
cost budget coarsens the exhaust. A runner is only a few hundred millimetres long, and its ends are a
good part of it, so its ramming depends on resolution far more than the exhaust's sound does: on 70
mm cells, at the speed they are tuned for, the LT2 fills three points less and the LT6 one.

**Dual plenums split the intake by bank.** With **Dual plenums** on a V or a boxer, the plenum's
casting is divided down its middle, as the LT6's is: each bank's runners draw from their own half, and
each half has its own throttle body of the **Throttle bore** and its own inlet tract, airbox and
snorkel, mirrored either side of the engine, each heard from its own snorkel's mouth. Apart, each
half carries only its own bank's pulses, evenly spaced on an engine whose banks each fire evenly, and
rings with them alone, which tunes the intake differently from one shared box. Two balance valves
through the wall between them, each a throttle bore across, at a third and two thirds of its length,
open at **Open balance valves at**, joining the halves, shut again at **Shut balance valves at**,
parting them, and open again for the top end at **Reopen balance valves at**; coming down, each happens
150 rpm below its speed. They swing over a tenth of a second, and the air through them is carried by
its momentum, as along the plenum. The LT6 has them, with two 87 mm throttles; on a dyno pull, crank
torque:

```
                          3500   4000   4500   5000   6500   7500   8400 rpm      N·m
valves always shut         406    435    504    542    628    587    560
valves always open         409    463    550    571    592    595    567
open 3500-5700, 7500 up    412    463    550    571    628    590    567
```

Joined, they fill it better from 4000 to 5500 and above 7500; apart, each bank's plenum ringing with
its own pulses does from 5700 to 7500, by up to 40 N·m, so its valves are shut across that band alone.
Open, the halves are still joined only through the valves, so they are not quite one box.

**Headers scavenge.** With equal-length headers, each cylinder's primary runs all the way to one
merge per collector, instead of joining a manifold along the ports. The **Equal-length header** tool
builds them ([Controls](controls.md#equal-length-headers)), and the presets that have them come with them. The wave each exhaust pulse sends
back from the merge reaches the next cylinder's port as a suction during the overlap, and pulls fresh
charge through the cylinder after its exhaust. On the LT6, whose cam holds both valves open for 70°,
410 mm primaries tuned for 8400 rpm fill the cylinder to 99% there, against 94% on a manifold.

A runner's sound dies away through the viscous and thermal boundary layer at its walls, and through
the turbulence, the bend into the port and the valve seat. The first is worked out from the runner's
own bore and air, using Kirchhoff's formula for a tube: about 10 /s for the LT6's 60 mm runners.
Kirchhoff's formula is for a small wave in a smooth, straight tube of still air, and the rest are not
modelled one by one, so the damping is five times that. A wave then loses more than half its strength
over one cycle at 8400 rpm and is gone within a few, as the pressure measured in real runners is.
On Kirchhoff's figure alone it rings on long enough to build from one cycle to the next, and the
full-throttle torque rises and falls by 5-8% every 1500 rpm or so as the runner's resonance comes
in and out of step with the cycle. The exhaust's damping is fitted to hot gas in steel pipes and is
two and a half times the runners'.

**What a cylinder pushes back stays by its valve.** The solver carries no composition, so each runner's
spent gas and fuel are carried alongside it, cell by cell, by the mass flows the solver finds across each
face. The exhaust a cylinder pushes back up its runner at overlap is drawn back in first on the next
intake stroke, as in a real port, and only what is pushed further than the runner is long reaches the
plenum. At idle the presets trap 21 to 34% spent gas, their manifolds 22 to 33 kPa absolute, as a real
engine's idle does, and the plenum stays clean. The injector meters on the fresh air drawn, net of what
was pushed back, so the mixture each cylinder traps is the one asked for.

**The idle air valve holds the idle speed.** With the throttle shut, a PI controller opens a valve round
the plate. The valve is sized for the engine rather than for the throttle body: it passes what the
throttle body the engine would be given left to itself would at that much more opening, so an engine
given a smaller one, as the LT2's 87 mm is, still has the air to idle on. The air it lets in takes the plenum's time to reach the cylinders, so on its own the
controller would carry the speed past the idle and back, round and round, worse the bigger the plenum.
A dashpot answers the speed's rate of change: it opens the valve further as the engine falls, so one
dropping off a lift catches itself at the idle, and closes it as the engine rises. On the LT6, whose
dual plenums hold 19 litres, the idle holds within about 680 to 850 rpm. The valve starts from about the
opening an engine needs to idle at 800 rpm, and more for one that idles higher, with the square root of
its idle speed. The Superquadro Mono idles at 1700 rpm, as a big single has to: started with the 800
rpm opening, it falls to 1250 rpm before the controller has learned the rest.

**Switched off, the vacuum bleeds away.** With the throttle shut, an idle air valve round the plate
holds the idle speed. When the ignition goes off it stays where it was, as a stepper motor or a
drive-by-wire throttle's own motor does without power, so air goes on leaking into the plenum through
it while the engine coasts to rest. On the F20C, the manifold is back within 1.5 kPa of the atmosphere
about two seconds after the crank stops, where through the plate's clearance alone it would take ten.

**The air comes in through an airbox.** Without a turbo, the throttle draws from an inlet tract: a
snorkel open to the air and an airbox, solved like the exhaust. The runners' pulses reach the air
through it, as the intake's note, and the jet past the throttle plate hisses through it. See
[Acoustics](acoustics.md#the-intakes-sound).

**The air in the throttle body has momentum.** The pressure across the throttle accelerates the column
of air in its bore, 6 cm long, and the throttle's own loss holds it back, so in steady flow it passes just
what the plate's opening does, choked when nearly shut. Wide open, a throttle would otherwise pass in one
sample many times the air that evens out the inlet tract's end and the plenum's front, and swing that air
back and forth through itself every sample: at full throttle, and with the plate left open after the
engine stops, where it would hiss at the snorkel's mouth on and on.

## Cam profile switching

A cam's lobe is a compromise. A wild one, open long and far, fills the cylinder at the top end, where
the charge is moving fast enough to keep coming in long after bottom dead centre. Low down, the same
lobe lets the charge flow back out before the valve shuts. A mild one does the opposite. Honda's VTEC
gives each valve both: below **Switch to high cam at** it follows the lobe the valve timing sets, and
from there a second, high-speed lobe with its own lift and timing. It switches back 150 rpm lower, so
it does not flap back and forth at the switch speed.

That is not the same as variable valve timing, below, which turns the whole cam. Phasing slides a
valve's opening and closing together; a second lobe changes how long the valve is open and how far.

On the F20C, with a 9 mm lobe closing the intake 30° after bottom dead centre and a 13 mm one closing
it 85° after, switching at 5500 rpm:

```
                          2000   3000   4000   5000   6500   7000   8000   8300 rpm   full throttle, N·m
high cam only              133    141    148    190    199    200    198    196
low cam only               184    191    199    196    166    153
switching at 5500          184    191    199    196    199    200    198    196
```

## Variable valve timing

The cams can be turned against the crank as the engine runs, as an oil-pressure phaser turns them. The
valve timing set in the panel is each cam's rest position. The ECU's map, which you set, advances the
intake cam and retards the exhaust cam from there: one amount up to a low speed, another from a high
speed, and a straight line between the two. A phaser turns at up to 250 crank degrees a second, so the
cams follow the map smoothly.

The map applies under load only. With the throttle nearly shut the cams sit at rest, which is least
overlap: at a near-vacuum in the manifold, overlap only pushes exhaust back up the intake and roughens
the idle. Load is read from the throttle rather than from the manifold pressure, which is what a real
ECU weighs the air by, because the overlap itself raises that pressure at idle. A map reading that as
load would advance the cam further and stall an engine with a big cam.

What it buys depends on the cam. On the LT6, whose cam is tuned for 8400 rpm, the phasers under load
retard the exhaust cam 25° and advance the intake 40° up to 4550 rpm, easing back to 15° by 7750. At
rest, as it idles, its cams have 15° less overlap than that: with all 70° of it at idle, the exhaust
it pushes back up the runners dilutes the charge until the idle hunts and stalls. With the cams held at
their top-end setting instead:

```
                          3000   4500   5500   6300   8400 rpm      full throttle, N·m
cams at top-end setting    406    465    541    600    568
cam map                    432    550    570    619    567
```

On the 2GR, the cam rests late, so its runners ram the charge in at the top end, and the phaser
advances it 40° at low speed. With the cam fixed at either end of that range, a band of the curve falls
away:

```
                          2000   3000   4000   5200   6400 rpm      full throttle, N·m
cam fixed early            334    332    349    311    264
cam fixed late             297    311    349    379    350
intake 40° at low speed    334    339    365    378    350
```

## The turbocharger

A turbocharger is a turbine in the exhaust and a compressor in the intake on one shaft. The throttle
draws from the air between the compressor and itself, the charge air, rather than from the atmosphere,
so on boost the cylinders fill from above atmospheric pressure. The turbine and the charge pipes are
part of the gas dynamics; the rest is lumped, one state each, and stepped every audio sample
([`turbo.rs`](https://github.com/tahoward/engine-simulator/blob/main/crates/engine-sim/src/turbo.rs)):

- **The turbine** sits in the exhaust, wherever the turbo was placed: the pipes drawn into its inlet
  meet at one pressure and its outlet pipe leaves at another
  ([Controls](controls.md#turbocharger)). The ducts arriving at it meet
  at one pressure and the tailpipe leaves at another, and between them the turbine and its wastegate
  pass the flow Stodola's ellipse law gives for the two: `m = K sqrt(p_in^2 - p_out^2) / sqrt(T)`. It
  is solved every substep with the junctions, against the waves arriving from both sides, so every
  exhaust pulse passes through it
  ([`exhaust_system.rs`](https://github.com/tahoward/engine-simulator/blob/main/crates/engine-sim/src/exhaust_system.rs)).
  Its power is the isentropic expansion across it at 68% efficiency, and the gas leaves it cooler by
  the work it did. The cylinders push out against its inlet pressure, which costs pumping work and
  leaves more spent gas in the cylinder.
- **Two or more turbos** blow into one throttle body, each down its own charge pipe and through its own
  intercooler, each with its own blow-off valve. Turbos on the same settings share one lumped shaft and
  one charge pipe, with the airflow split evenly between them; one set differently
  ([Controls](controls.md#turbocharger)) has a shaft, compressor duct, charge pipe, wastegate,
  intercooler and blow-off valve of its own, turned by its own turbine. Each turbine is solved on its own, on the pulses of the cylinders
  feeding it. Each wastegate opens on the one boost, so a turbo on a higher target keeps its gate shut
  while the other's opens, and the other, slowing, can be pushed into a surge by the charge air it can
  no longer hold.
- **The wastegate** opens a bypass around the turbine as the boost reaches its target, so the turbine
  takes less of the exhaust. It is worked by a boost controller, as a modern engine's is: it opens in
  proportion to how far the boost is over the target, going from shut to wide open over a quarter of the
  target, and a trim that follows the error over about a second brings the mean boost onto the target.
  The band grows with the target because the turbine's spare power does: a band a fixed tenth of a bar
  wide would, at 2 bar, swing the gate from shut to wide open and back a few times a second, and the
  boost and torque with it. The actuator follows with a 40 ms lag. Its hose and diaphragm feel the boost
  smoothed over 20 ms: the boost's mean, not the pulses the runners and
  the plenum's waves ride on it, which at 6500 rpm on the RB26 swing it by about 0.15 bar either way.
- **The compressor** is Moore and Greitzer's model. Its characteristic is a cubic in the flow, scaled
  with the square of the shaft speed, with its peak pressure rise at 44% of the choke flow at full
  speed: its surge line. To the left of that the pressure it makes falls as the flow falls, which is
  what drives a surge. As on a real map, the lower speed lines are wider and flatter: the surge line
  moves toward less flow as the speed falls, and below 80% of full speed the fall to the left of the
  peak shrinks, to nothing at 40%. Backwards through the wheel the characteristic falls a little further
  past no flow, then rises steeply as the spinning blades fight the reversed flow, as measured
  ones do, so a wheel shut in behind the throttle never comes to rest with no flow through it. The pressure the wheel makes follows its characteristic two
  revolutions behind. A rotating stall grows to the left of the surge line within a few revolutions,
  lowering the pressure the wheel makes by up to 6%, and dies away once the flow recovers; on the
  flatter speed lines it grows to less, and on a flat one, at idle, to nothing. The air in
  the 0.6 m duct the compressor draws through, from the air filter, has inertia. The efficiency
  is best in the middle of the map and falls toward the choke, so a turbo too small for the engine
  does ever more work for the same boost at the top end. Up to full speed, the speed its sizing gives
  it, the choke flow rises in proportion to the shaft speed. Above it the choke flow levels off at 10%
  more, as the speed lines crowd together near the choke on a real map: a wheel spun faster still makes
  more pressure at low flow, but passes little more air.
- **The charge pipe** carries each compressor's air to the throttle body, laid out as for an
  intercooler at the front of the car: 1.2 m of pipe to the intercooler, the intercooler, and 1.5 m of
  pipe back, at one and a half times the inducer's area,
  solved like the exhaust on cells no shorter than 8 cm. The charge air's volume, twice the swept
  volume, is mostly the intercooler's; the throttle body, a fifth of the swept volume, is lumped. The
  charge air's pressure waves travel along it and reflect off its ends: when the throttle shuts, the
  compressor feels it about 10 ms later, as the wave arrives. No sample's flow through the throttle moves more air
  than would bring the throttle body and the front of the plenum to the same pressure. A wide-open
  throttle between them, as a diesel's always is, would otherwise pass many times that each sample and
  swing the air back and forth between the two without ever settling.
- **The shaft** is accelerated by the turbine's power less the compressor's and its bearings'. Its
  inertia grows as the wheel's diameter to the fifth, which is why a big turbo lags.
- **The intercooler** takes a share of the compressor's heating back out of the air it delivers.
- **The blow-off valve** opens on the pressure across a shut throttle and vents the throttle body, to
  the atmosphere or back to the compressor inlet. Each turbo's is as big as its compressor's inducer, so
  with one turbo's taken off, the others' vent less of the charge, and more slowly.

Left on auto, the turbo is sized from the engine's airflow at 80% of its rev limit on full boost.
Whatever its size, its turbine's nozzle is sized for the boost target: narrow enough that at that
airflow, wastegate shut, the turbine makes 1.76 times the power the compressor takes to reach the
target. The higher the target, the higher the pressure the exhaust backs up to behind it: at 0.7 bar,
twice the atmosphere's at that flow; at 2 bar, near six times.

The lag is not a filter on a boost map; it is the shaft spinning up. At 3500 rpm, opened from part
throttle, the RB26's twin turbos take 1.2 s to reach 90% of their boost.

With the throttle shut on boost and no blow-off valve, the charge air has nowhere to go. The throttle
sends a pressure wave back up the charge pipe as it shuts, and when it reaches the compressor the flow
falls past the surge line, where the characteristic that is stable to the right of it is unstable. The
flow collapses and runs backwards through the wheel, the charge pipe empties back out of the inlet,
and the compressor recovers, over and over: on the RB26 at 4000 rpm, about 15 times a second. That is a
surge, and it is where the flutter comes from. It is not scripted: it comes out of the compressor's
characteristic, the inertia of the air and the charge pipe's gas dynamics, as Greitzer's model of it
does. As the shaft slows, its speed line flattens and has less to drive a surge with: revved free and
lifted, the RB26's surge has died away within a second and a half.

A blow-off valve opens as the plenum's pressure falls behind the shut throttle. The wave from the
throttle reaches the compressor before the valve has lifted, and pushes the flow back through it once;
from then on the valve lets the charge go, and the compressor does not surge.

The RB26 preset's exhaust has the real engine's layout: the front three cylinders' pipes feed one
turbo, the rear three's another, both on the exhaust side of the head, and the two turbos' outlets meet
behind them. At 0.7 bar, with its naturally aspirated self on the same cams, runners and pipes for
comparison:

```
                          2000   3000   4000   5000   6000   7500 rpm   full throttle, N·m
naturally aspirated        215    211    218    235    209    174
twin turbos, 0.7 bar       277    374    384    399    350    279
boost, bar                0.26   0.67   0.69   0.70   0.69   0.69
```

Its turbos are sized to what is known of the real engine's T28s (see the preset), and each is fed the
pulses of three cylinders: full boost from 3000 rpm, and at their full speed at the top, about 125k
rpm, where the exhaust ahead of them is about 0.7 bar above the atmosphere. The power, less friction,
reaches 299 PS at 6000 rpm; past that the torque falls about as fast as the speed rises, so it holds
near 297 PS to the limit, as a stock engine's does on the dyno. Compressors too small for it reach
their choke instead: with ones passing 0.12 kg/s each, the shafts run a third faster than these by
6400 rpm, and the power falls from 7000 to 7900. At 4400 rpm each pulse arriving at a turbine swings the pressure there
by about 15 kPa; past it, by 8.

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

Because the burn does not take a fixed number of degrees, a fixed spark would be wrong almost
everywhere. At low rpm the burn is quick, so the pressure would peak before top dead centre and
push against the piston. A four stuck at 450 rpm under load would then make no power whatever the
throttle did. At part throttle the burn is slow and would peak too late to do much work.

So the spark follows an advance map, as a real ignition system does. The spark moves so that each
charge reaches half burned where a reference burn would. The **Ignition advance** control is the
timing for that reference burn. The map retards the spark at low rpm and advances it at part
throttle and high rpm, and keeps it between top dead centre and 50° before it. The spec's
`advanceCurve` switch turns the map off and fixes the spark at the set angle.

### The mixture

Fuel and air are tracked separately, through the manifold, the runners and the cylinder. Each
cylinder has a port injector in its runner, which meters fuel into the air the runner draws from the
manifold, in proportion to it, at the **Mixture** control's
[λ](glossary.md#air-fuel-ratio-and-lambda). So the manifold holds air, and each cylinder's fuel
arrives with its own charge. Gas pushed back up a runner carries its fuel with it, and brings it back
next cycle.

- **Lean**, each charge carries less fuel, so it releases less heat, and it burns slower.
- **Rich**, the oxygen runs out first. The extra fuel goes out unburned, so torque stays level with
  stoichiometric rather than rising. With no air left to burn it, it does not light in the pipe.
- **The spark can fail to light the charge.** Excess air dilutes the charge the way leftover exhaust
  does, and the two are counted together, so a lean idle misfires sooner than a lean charge at full
  throttle. Misfires also start once the mixture itself burns at under half the speed of a
  stoichiometric one, near λ 1.5. They become certain at the flammability limit, near λ 2.15.

**Overrun fuel cut.** With the throttle shut above 1500 rpm the fuel stops, as it does on a
fuel-injected engine. The engine is then turned over by its load, pumping air. The fuel comes back
below 1200 rpm or as soon as the throttle opens. On an engine that idles above 800 rpm both move up
with its idle, to 700 and 400 rpm above it, so the fuel never stops at the idle itself. The injectors stop at once; what fuel is left in
the runners is drawn in over the next cycle or two.
Turn **Overrun fuel cut** off for a carburettor's behaviour, which keeps feeding fuel with the air
that leaks past the throttle, so the engine keeps firing weakly.

## Afterfire

Fuel that leaves the cylinder unburned can light in the exhaust and pop. A spark cut sends out whole
charges, fuel and air together: the rev limiter, the launch control holding the engine at the launch
speed, and the traction control through a shift. So does a misfire, and a burn still going when the
exhaust valve opens sends out what it has not yet burned. That fuel burns in the pipe, if at all,
not in the cylinder.

Each cylinder keeps a pocket of this mixture in the first 0.4 m of its primary: the port and the
start of the pipe, where the next blowdown is still hot. The pocket fills with the fuel and air its
valve sends out. The gas the valve sends in after it pushes it on down the pipe in proportion, so a
pocket that does not light soon washes out. With the valve shut it waits in the header.

The mixture lights once its ignition delay runs out in hot gas: the hottest gas in those cells, as a
blowdown from a cylinder that fired, or a pulse coming back from the collector. The delay follows an
Arrhenius law, about 5 ms at 1000 K and 38 ms at 900 K, integrated over the gas's temperature as it
changes. Nothing reacts below 850 K, where hydrocarbons stop oxidising in an exhaust port. A pocket
leaner than the lean limit never lights. That limit falls with temperature, from about λ 1.8 at
room temperature to about three times leaner at 1300 K. The point where each pocket lights is drawn
at random, so pops come at irregular moments, and the same ones every run.

Hot and already mixed, a pocket burns nearly all at once, in about 1.5 ms, and burns between half
and all of the fuel its air can burn. The heat goes into the gas of those cells. So the pop is a
pressure pulse that travels down the exhaust, through its junctions and mufflers, like any other.
Rich fuel with no air to burn it goes out unburned, and a lean charge that fired leaves nothing to
burn. A steady engine burns its fuel in the cylinder, so it never afterfires.

**Overrun crackle.** This is a performance car's "pops and bangs" map. Switched on, for up to 3 s
after the throttle shuts above 2500 rpm, it holds off the fuel cut. It cracks the throttle open to
feed the charge and fires the spark long after top dead centre, and it skips the spark on some
cycles. The late burns fill the header with hot gas, and the charges it skips light in it.
**Crackle** sets how hard it works:
- the spark from 15° to 45° after top dead centre;
- from 10% to 35% of the sparks skipped;
- the throttle open from 0.05 to 0.15.

The map stops below 2000 rpm, and the fuel cut takes over. Opening the throttle arms it for the next
lift.

## The diesel

A diesel is set by **Fuel**. It draws its air through an intake with no throttle body: its tube opens
straight into the plenum at its full bore, whatever the pedal does, so its plenum sits at the atmosphere, or at the boost, and never in vacuum. No fuel
comes with the air. The pedal sets how much fuel the pump injects into each cylinder, as a share of
its full delivery, and the injection starts at a fixed angle before top dead centre, as a mechanical
pump's static timing does. The charge it lights is always lean overall: λ 14 at idle, and no richer
than about 1.5 at full pedal.

**How much fuel.** The fuel each cycle gets is committed when the intake valve closes and the air is
trapped. At full pedal it is the less of two limits:
- the **smoke limit**: the fuel the trapped air can take at λ `smokeLambda`. Richer than that the
  fuel finds too little air to burn clean, so a real pump's stop, or its boost compensator, holds it
  there. Off boost this is what limits the torque.
- the **full delivery**, `maxFuel`: the most the pump injects a stroke. Once the turbo gives the air
  for it, this is what sets the torque.

**When it lights.** The fuel lights itself once the hot air has heated and vaporised it, after an
ignition delay. The delay comes from Hardenberg and Hase's correlation, from the temperature and
pressure compression leaves the air at by top dead centre, and the mean piston speed:

    delay (deg) = (0.36 + 0.22 Sp) exp[ E (1/(R T) - 1/17190) + (21.2 / (p - 12.4))^0.63 ]

with `Sp` in m/s, `T` in K, `p` in bar and `E = 618840 / (CN + 25)` J/mol for a fuel of cetane number
45. Hotter, denser air lights it sooner, so a higher compression ratio or more boost shortens it. At
a running diesel's top dead centre it is a few degrees, about 0.5 to 1 ms.

**How it burns.** The burn is two Wiebe burns added together, as Watson's correlation has it:
- the **premixed** burn: the fuel that has mixed with the air during the delay burns all at once,
  in about 0.5 ms, taking off at once rather than easing in as a spark's flame does (a Wiebe form
  factor of 1, against a spark's 2), as Watson's premixed burn does. Its share is
  `1 - 0.926 phi^0.37 / delay^0.26`, with the delay in ms and `phi` the overall equivalence ratio. A long
  delay and a little fuel give a big share, so the sharp pressure rise that is the diesel's clatter is
  loudest at idle and light load.
- the **diffusion** burn: the rest burns as it is injected and finds its air, over **Burn
  duration** at full fuel and 10 m/s mean piston speed. It is shorter on less fuel, and longer the
  faster the engine turns.

The burn flickers as the turbulence it burns in does: from one step to the next its heat release
wanders by 80% RMS about the steady rate the Wiebe curves give, the same heat on average, which is
what makes it roar rather than thud while it burns.

No two cylinders burn alike either. A mechanical pump's elements and the injectors they feed are
calibrated to a few percent and a fraction of a degree at full load, and drift further apart at idle,
where each delivers its least. At **Cylinder spread** 1 each cylinder's delivery is up to 70% more or
less than the mean and its injection up to 7° early or late, so on the 6CT, at 0.3, up to 21% and 2.1°.
Those differences, fixed for each cylinder, are what puts the half orders, at multiples of half the
crank's speed, into a real diesel's sound: at the 6CT's idle they stand 8 dB above the noise between them
from 150 to 460 Hz, as a 6CTA's do.

No two cycles light alike: the spray and the swirl it meets never are. The ignition delay wanders by
8% RMS from one cycle to the next, about half a degree at idle, and the premixed share by 20%, at
**Cycle-to-cycle scatter** 1. That is what keeps the clatter from ringing as a tone at the firing
frequency's harmonics.

The fuel joins the cylinder's gas as it burns, so none of it is ever in the charge unburned, and a
diesel never afterfires. Nor can a lean charge fail to light, as a spark's does: the misfire limits
are a spark's, and a diesel does not have them.

On the Cummins 6CT these come out, on average, as:

```
                              delay   premixed   diffusion   λ
idle, 800 rpm                  5.8°     67%        18°       14
1500 rpm, 10% pedal            7.3°     64%        25°       15
1500 rpm, full pedal           5.0°     10%        46°       1.7     on boost: the full delivery
2200 rpm, full pedal           6.5°      3%        56°       1.5     at the smoke limit
```

**The governor.** A diesel's speed is held by its fuel. With the pedal up, a governor holds the idle
speed, giving it more fuel against a load, the same controller that is a petrol engine's idle air
valve. Above its idle with the pedal up it gives none at all, so there is no overrun fuel cut to
switch. At the top, over the 300 rpm below its **Governed speed** (the rev limit), it takes the fuel
away, none left at the speed itself, so the engine runs up to it smoothly instead of bouncing off a
spark cut. Launch control cuts its fuel where it would a spark.

### The clatter

A diesel's pressure rises ten times as fast as a petrol engine's: 5.6 bar per degree at the 6CT's
idle against 0.5 on the RB26. Its block carries that to the ear as a broad, hard clatter. Three sources
make it, all heard from the engine's casing (below), all only on a diesel:

- **The block.** The pressure rise drives 168 block modes, log-spaced from about 200 Hz to 12 kHz on
  the 6CT (lower on a bigger engine), each of Q 50, as lightly damped as cast iron is. They overlap
  into one broad ring that carries on for 10 ms and more, where a petrol engine's four modes would ring
  as four tones under a diesel's drive. They pass the combustion alike from about 700 Hz to 3 kHz on
  the 6CT. Below that they pass ever less, 20 dB less at 200 Hz, as a stiff casting radiates its
  slow bending poorly; and less above, as the casting, its covers and the manifolds over it muffle the highs. A
  knock is not felt all through the casting at once: bending waves carry it at a few hundred metres a
  second, and the walls that radiate it lie at their own distances from each cylinder. So each mode is
  struck at its own moment, spread over 3 ms after the blow. Struck all together, the many modes
  would add to one hard spike every firing, three times as peaky as a real block's clatter. Below 214 Hz on the 6CT (300 Hz on the reference engine),
  two poles of high-pass take the rise ever less to heart: the slow swing of compression and expansion
  is far the largest part of it and the most regular, and through the low skirts of the modes it would
  buzz at the firing frequency's harmonics. A piston's slap on its liner is a knock the same block
  rings to, so on a diesel it drives these modes too, rather than one narrow mode of its own, which
  every firing would ring as a note.
- **The chamber.** The premixed burn lights all at once, unevenly, and sets the gas in the chamber
  ringing at its own acoustic modes, `c alpha / (pi bore)`. With Draper's numbers for the first two
  circumferential modes and the first radial (1.841, 3.054 and 3.832), the 6CT's 114 mm bore rings
  near 3.2, 5.3 and 6.7 kHz in its 1000 K gas, for a millisecond or two. The ring is part of the
  cylinder's pressure, swinging it by 15% of the rise the premixed burn makes, about a bar at idle,
  as measured direct-injection diesels ring, and varies by a third either way from one firing to the
  next. It shakes the block with the rest of the pressure rise, through the same modes.
- **The injectors.** Each needle ticks as it lifts at the start of injection, and harder as it
  slams shut at the end, 3° to 22° later as the fuel grows.

Set against a recording of a Cummins 6CTA idling at 662 rpm, made standing beside it, the 6CT at the
same speed, heard from where a standing person's head would be, has as much clatter over its low rumble
as the real engine: its 0.5-4 kHz band 12 dB over its 25-200 Hz, against 12.8. Each firing's clatter
peaks 7 dB over its mean level, against the real engine's 4.6. Its third-octave bands come within 9 dB
RMS of the recording's, with the same tilt; the clatter is still too strong around 1.6 kHz and short
around 2.5 kHz and above 8 kHz.

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
the crank unevenly. In the app the crank is driven by the torque itself, against friction, the
load and the flywheel, so the speed follows the throttle and the ripple comes with it. On a big
single, the speed swings by a few percent within a cycle, and it does so even on a
[dynamometer](glossary.md#dynamometer) (a test rig that holds the engine at a set speed). The tests
measure the engine held like that, at exact operating points. There only the *fluctuating* part of
the torque may move the crank, because whatever holds the speed absorbs the average, so the model
tracks the average torque and subtracts it first.

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
centre, the piston's highest point). So it is loud under load and almost gone when coasting.

**The moving parts make their own running noise.** Each one follows the loads the simulation
already computes, so nothing runs on a timer:

- **Piston and rings sliding.** The rings and skirt rub the bore. The rub is noise, as loud as the
  piston's speed times the load pressing it on the bore: the rings' own tension, the gas behind the
  top ring, and the side thrust from the rod's lean. It is quiet at each dead centre and loudest
  mid-stroke, and under load loudest early in the firing stroke, where the gas load and the lean meet.
- **Valvetrain.** Each valve ticks as it leaves its seat and its lash closes, then clacks shut, each
  an impact as loud as the momentum it brings. A cam sets its valves down on a closing ramp and lifts
  them on an opening one, ground for 0.009 mm of travel per crank degree, so they land faster in
  proportion to the engine's speed: 0.3 m/s at 6000 rpm, 0.04 m/s at a diesel's idle. What lands is
  the valve, weighing as the cube of its head's diameter (about 43 g at 34 mm, 107 g at 46 mm), with its
  retainer, keepers and spring, half as much again; what closes the lash is a rocker and pushrod, half as
  much again as the valve, or a bucket, half the valve. So a heavy pushrod diesel's valvetrain knocks far
  louder at a given speed than a light overhead-cam engine's, and a head with two valves of a kind
  knocks twice as hard as one with one. While it is open, its cam follower rubs the lobe, as loud as the spring it pushes against.
  The timing drive whines at 21 meshes a crank turn, louder as the speed rises, and its tension
  wavers with the valve springs it turns and with its chain's slack.
- **Bearings and crank.** A rod's bearings knock across their oil clearance whenever the force down
  the rod changes sign. At speed with a closed throttle, the piston's inertia flips it four times a
  cycle. Under load, the gas holds the rod in compression through the firing top, so it flips only
  twice. The crank twists on its first torsional mode under the torque it carries. A longer crank
  twists lower, as the square root of its throws.

All this mechanical noise sits about 18 dB below an open header. That is why you only hear it once a
muffler has quietened the exhaust.

**You hear the engine from where the camera is.** Each place the engine makes its sound is heard
from where it is drawn: every tailpipe's outlet, the intake above the front of the engine, each of the
casing's surfaces (below), and the turbos from where they sit. Each has its own path to the camera,
so its own delay and its own loss with distance, and moving the view moves your ear. The paths glide
to their new lengths over 50 ms as the camera moves, so a turn of the view does not click. Nearer
than 0.25 m, a source gets no louder.

**The casing is several surfaces.** Each part's noise comes out of the part of the casing it shakes:

- **Each block side**, halfway up the casting from the crankcase to the deck: its bank's piston slap
  and ring scuff, and three quarters of the block's combustion ring, shared between the sides. An
  inline's one casting has both its sides; a vee's or a boxer's each casting has its outer side.
- **Each head**, at the top of its valvetrain, facing out along its bank: its valves' clack and lash
  ticks and its cam followers, and on a diesel its injectors' ticks.
- **The oil pan**, under the crankcase, facing down: the rods' bearing knock, the crank's twist and
  the last quarter of the block's ring.
- **The front cover**, on the block's front end, facing forward: the timing drive.

The cam covers and the oil pan are thin stamped panels and ring at three low modes of their own, at
about 450, 950 and 1800 Hz and 300, 650 and 1300 Hz on a casing half a metre long, lower on a longer
one. At its own frequency each mode adds as much again as the surface carries. The block's sides and
the front cover are stiff castings and ring only as the block does.

Each surface is loudest the way it faces, as a baffled panel is, and quietest behind, a fifth of that,
where the engine is in the way. Each still radiates the power a source radiating alike every way
would, so the casing is about as loud from any side as before, within a few dB. What changes is what
it sounds like: on the LT2 from the side its 1-2.5 kHz band, the block's and the pistons', is 5.5 dB
more of its sound than from the front, where the pan and the timing drive have more of it. From one
place that difference was 1.2 dB. The ground and walls' reflections leave each surface the way they
would reach the ear's image in them, so they are directional too.

Without the app to place them, as in the tests' reference renders, the casing radiates from one
place, the engine's middle, alike every way.

The ground lies `exhaustHeight` below the lowest tailpipe, and each source's sound also bounces off
it. The bounce arrives later and duller, and mixing it with the direct sound cancels some
frequencies: 1.5 m from a tailpipe, the first cancelled frequency is near 400 Hz, right in the
middle of the engine note. Air also absorbs high frequencies, so a distant engine sounds muffled,
not just quieter.


## More cylinders

Every cylinder uses the same model on the same crank. So, for sound purposes, an engine layout is
just two lists: **when each cylinder fires**, and **which bank it belongs to**.

An engine is one bank of 1 to 6 cylinders, or two banks of 1 to 6 each: a V, or at a 180-degree bank
angle on a boxer crank a flat engine. Each layout fires the way the real engines of its kind do. Real
[firing orders](glossary.md#firing-order) are chosen for crankshaft balance and bearing loads, and no formula recovers them,
so the inline engines, the V4, V6, V8s and the flat four and six each have their own crank, taken from the
real engines. The twin is the exception: there, the shared crankpin really does set the firing interval,
and you can hear the V angle in it.

The other V engines are two banks of an inline crank, each throw shared by both banks, the second bank's
cylinder firing the bank angle after its partner. On an inline five's crank that gives the even 72-degree
firing of a 72-degree V10 and the 54-90 of a 90-degree one, the Viper's; on an inline six's, a 60-degree
V12's even 60. The flat eight, ten and twelve fire evenly, each opposed pair a revolution apart.

The firing order and the gap before each firing can also be set by hand (**Layout → Firing order** and
**Firing intervals**), over any layout: an odd-fire or big-bang engine is the same crank with uneven gaps.
Cylinders are numbered front to back along the crank, alternating between the banks on two. Pins are
shared by two cylinders wherever their firings let them, and each other cylinder has a pin of its own.

```
              fires at                          banks         each bank fires
single        0                                  A            every 720
V-twin 45°    0, 405                             A B          405 / 315 apart
inline three  0, 240, 480                         A A A        every 240
inline four   0, 180, 360, 540                   A A A A      every 180
inline five   0, 144, 288, 432, 576              A A A A A    every 144
inline six    0, 120, 240, 360, 480, 600         A A A A A A  every 120
V4 90°        0, 180, 450, 630                   A B A B      180 / 540 apart
V6 60°        0, 120, 240, 360, 480, 600         A B A B A B  every 240
boxer four    0, 180, 360, 540                   A A B B      180 / 540 apart
boxer six     0, 120, 240, 360, 480, 600         A B A B A B  every 240
V8 flatplane  0, 90, 180, 270, 360, 450, 540, 630  A B A B A B A B    every 180, even
V8 crossplane 0, 90, 180, 270, 360, 450, 540, 630  A B A A B A B B    180-90-180-270, uneven
V10 90°       0, 90, 144, 234, ... every 54 and 90  A B A B ...         every 144
V12 60°       every 60                             A B A B ...         every 120
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

- **Each tailpipe is heard from its own place.** Every outlet is heard from where it is drawn, with
  its own travel delay and distance loss to the listener. Tailpipes exiting either side of a car are
  about a metre apart, which at 187 Hz is most of a wavelength. If they were heard from one point, a
  flatplane V8's two banks, which fire exactly out of step, would cancel each other's firing order and
  the engine would jump up an octave. Heard from straight behind, outlets placed symmetrically are the
  same distance away and cancel like that anyway; heard from off to one side, they do not. A car whose
  tailpipes exit together in the middle, as the Corvette Z06's four do, has its outlets close enough
  that the two banks' firing orders mostly cancel from anywhere behind it, and the LT6 preset draws its
  collectors turned in to a centre exit, 0.21 m apart. A dyno recording of the Z06 shows the same: its
  firing order dominates, and the banks' own firing order sits about 10 dB below it.
- **No two cylinders breathe quite alike.** `cylinderSpread` varies the runner pressure each
  cylinder sees by a few percent, standing in for unequal runner lengths, valve seats and fuelling.
  At the default, 0.3, it varies the pressure by up to 1.2% and the cam timing by up to 0.7°. The
  LT6's odd orders then sit 17 to 29 dB below its firing order, close to the 20 to 26 dB of a dyno
  recording of the Z06 (see [Verification](verification.md#against-a-recording)), so there is a
  rumble under the firing note. An inline four on equal-length runners comes out a little cleaner,
  at about −37 dB, and the full spread of 1 puts it at −25 dB. Pressure is varied rather than intake
  valve area because it changes how much gas is trapped; valve area only changes how fast the
  cylinder fills.

**The crank layout can be worked back out from the firing order**, which is a useful check on the
data. Two cylinders share a crankpin when the second fires one bank angle after the first, or one
bank angle plus a full revolution. Asking `crankPins` to pair them up gives four pins 90° apart for
the crossplane, and four pins in one plane for the flatplane. That is exactly what those cranks are
named for. A 270° parallel twin correctly comes out with two pins, because no shared pin can give
that interval. That is what the firing-offset override is for.

**Cylinders sharing a pin sit a rod's width apart along the crank** (`ROD_STAGGER`, 16 mm), so their
rods run side by side on it, and so one bank of a V sits a little ahead of the other, as on a real
one. A split pin puts each of the two on its own pin, with a thin web between the two pins, so they sit
that much further apart (`SPLIT_WEB`, 5 mm). In a V too narrow, or
with bores too big for their stroke, for the two banks' pistons to pass each other at the bottom of
their strokes, the two sit further apart, as far as keeps the pistons as drawn 4 mm clear through the
whole cycle (`rodStagger`), as a VR engine's banks are staggered, and the throws far enough apart for
the next pair to clear too, and for a main journal between their pins. The Milwaukee-Eight's narrow vee and big bores make it one of these.

**A V too narrow for an intake in its valley shares one head between its banks** (`sharedHead`), as a
VR engine does: below the angle where the two banks' intake ports would meet in the valley even on top
of the heads, or where their overhead intake cams leave no room for a runner between them, about 30–40°
for the presets. Every cylinder's exhaust valves and port are then on bank 0's outer side and its intake
valves and port on bank 1's, so the exhaust comes off one side of the engine and the plenum sits beside
the other, as an inline engine's do, and the two banks' cams in the middle of the head take turns along
it between the staggered cylinders. Everything placed
along the crank follows `cylinderZ`: the cylinders, and the exhaust ports, so an 8-into-1's two
downpipes differ in length by a few millimetres.

**Main bearings sit between the throws** (`mainBearingsAfter`), and at each end: five for an inline
four or a V8, seven for an inline six. A flat engine is the exception. Each opposed pair's pins sit
side by side, half a turn apart, joined by one web, and the mains go between the pairs: three for a
flat four, four for a six.

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
Any imbalance left over is reported as a diagnostic rather than corrected.

**Near the speed of sound, a pipe end at a junction behaves as gas does.** Filling a pipe, the
junction's gas cannot come in faster than sound: it accelerates from the junction's pressure and
temperature as through a nozzle and chokes at the pipe's end, at 0.54 of the junction's pressure, however
far below that the pipe's end falls. Leaving a pipe faster than sound, gas cannot feel a junction lower
than itself, and passes into it as it is; into a junction higher than itself it meets a shock, which the
Riemann solve resolves. A manifold no wider than one runner, which runs its flow close to sonic, needs
both: without them its junctions lock or clamp.

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
- Runner boundary-layer damping: [Pierce 2019](references.md#pierce2019).
- Laminar burning velocity: [Rhodes and Keck 1985](references.md#rhodes1985), with Heywood's
  gasoline constants; turbulent entrainment and burn-up: [Blizard and Keck 1974](references.md#blizard1974).
- A diesel's chamber ringing: [Draper 1938](references.md#draper1938).
- Diesel ignition delay: [Hardenberg and Hase 1979](references.md#hardenberg1979); its premixed and
  diffusion burns: [Watson et al. 1980](references.md#watson1980).
- Junction solve: [Toro 2009](references.md#toro2009).
- Outdoor listener, ground reflection and air absorption: [Kinsler et al. 2000](references.md#kinsler2000).
