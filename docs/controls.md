# Controls

## Starting and choosing an engine

- **Click anywhere on the intro screen**, or click the start button in the panel. Browsers only let
  sound start after you click or press a key. The desktop app starts the engine as it opens.
- <kbd>Space</kbd> starts and stops the engine.
- **Engine preset** loads a complete engine: its layout, firing order, sizes and a matching exhaust.
  It starts idling in neutral, at 800 rpm or the Superquadro Mono's 1700, on a throttle opening found
  for that engine. Open the
  throttle and add load to go from there.
- **Layout → Banks** and **Cylinders per bank** change only the layout: one bank of 1 to 6 is an inline
  engine, two banks of 1 to 6 each a V. Two banks have a **Bank angle**, from 15 to 180 degrees; a second
  bank starts at the angle its real engines have, 60 for a V6 or V12, 72 for a V10 and 90 otherwise. At
  180 degrees **Crank** offers a boxer, opposed pins making it a flat engine; on a V8 it is crossplane or
  flatplane. Changing the layout also builds an exhaust to suit: a single pipe on a single, one collector
  per bank on two banks, and one collector for all cylinders otherwise.
- **Layout → Firing order** and **Firing intervals** set how the engine fires, as "1-5-3-6-2-4" and
  "180-270-180-90": the cylinders, numbered front to back alternating between the banks, and the crank
  degrees from each firing to the next, adding up to 720. Left empty, the layout's own, shown faintly;
  **Layout's own firing** clears both. An order or intervals the engine cannot fire are pointed out, and
  the layout's own is used until they are fixed.
- **Operating point** has no speed control: the engine turns as fast as its torque drives it
  against friction and the **Load**, so the **Throttle**, the load and the exhaust tuning all set
  it. **Flywheel inertia** sets how quickly it responds.
- **Operating point → Rev limiter** sets the highest speed the engine will run. Each preset sets
  the limit of the real engine it copies. Above the limit the spark is cut, and it comes back
  200 rpm lower, so the engine bounces off the limit the way a real one does.

## Launch

**Start launch** runs a standing start at full throttle. The engine revs to the **Launch at** speed, and
the clutch slips to hold it there until the car catches up. Then each gear is pulled to the **Shift at**
speed. The run works the throttle itself, and it ends at the shift point in top gear, or when the car
stops gaining speed. As the car accelerates, its weight moves back, onto the rear wheels and off the
front, so driven rear wheels grip harder and driven front ones less.
Too much torque for the tyres spins them, and a spinning tyre grips about a fifth less than one at its
peak. The car is slowed by rolling resistance and by air drag, with a drag area of 0.6 m².

- **Car mass** is the car's weight with the driver in it. Auto sizes the car to the engine, about 9 kg
  per kW.
- **Shift time** is how long each shift takes, from lifting off to full throttle in the next gear. Auto
  is 0.4 s, a quick manual shift.
- **Tyre grip** is the tyres' friction coefficient at their peak, driving: 1.1 for a road tyre on auto,
  and 1.31 for the Corvettes. That is the 1.22 g the Z06 pulls on a skidpad on its Cup 2 Rs, raised 7%,
  because a tyre grips a little harder driving in a straight line than cornering.
- **Dual-clutch gearbox** shifts with no gap in the drive: the next gear's clutch takes the drive as the
  last one lets it go, and the throttle stays open. Off, each shift lifts off and takes the clutch out.
- **Driven wheels** are the rear, the front or all four. Auto is the rear, and a real car's stock is its
  own. The rear wheels of a car fitted to the engine carry half its weight at rest, and front ones three
  fifths; a real car's carry its own share. All four take the car's whole weight, so it launches
  hardest before the tyres spin.
- **Traction control**, on by default, works as a launch control and a dual-clutch gearbox's torque
  management do. Off the line it cuts the spark to hold the engine at the launch speed. While the clutch
  slips, it passes no more torque than the tyres can take. In gear it cuts the spark whenever the tyres
  slip past their peak. Off, an engine with more torque than the tyres can take spins them, and reaches
  the shift point before the car has the speed for it.
- **Gearbox** starts as a close-ratio six-speed. Type a ratio into any gear to change it. **×** takes a
  gear out and **Add gear** adds one above the top gear. A gearbox can have from one to eight gears.
  Next to each gear is the road speed it reaches at the shift point.
- **Final drive** is geared on auto so that the shift point in top gear comes where the car would run
  out of power against its drag. Type a value to set it yourself. **Reset gearing** puts back the
  six-speed and the auto final drive.

The presets of real engines launch in the real cars they come from, marked **(stock)**. Each uses its car's
gear ratios, final drive, tyres, kerb weight with a 75 kg driver, weight on the driven wheels and gearbox:

| Preset | Car | Gearbox | Tyres |
|---|---|---|---|
| Ducati Superquadro Mono | Ducati Hypermotard 698 Mono, 235 kg | six-speed, primary gears and chain | road |
| Harley-Davidson Milwaukee-Eight 121 | Harley-Davidson CVO Road Glide, 466 kg | six-speed Cruise Drive, primary chain and belt | road |
| Honda RC51 | Honda RC51 (SP-1), 298 kg | six-speed, primary gears and chain | road |
| Triumph 1200 HT | Triumph Speed Twin 1200, 291 kg | six-speed, primary gears and chain | road |
| Honda F20C | Honda S2000 (AP1), 1349 kg | six-speed manual | road |
| Subaru FA20D | Toyota 86 (ZN6), 1325 kg | six-speed manual | road |
| Porsche Mezger 4.0 | Porsche 911 GT3 RS 4.0 (997), 1445 kg | six-speed manual | road |
| Ford 1.5 EcoBoost Dragon | Ford Fiesta ST (Mk8), 1265 kg, front-wheel drive | six-speed manual | road |
| Toyota 3S-GTE | Toyota MR2 GT-S (SW20, 1992–93), 1325 kg | Toyota E153 five-speed manual | Yokohama A022 |
| Audi EA855 EVO | Audi RS 3 Sportback (8V, 2017–20), 1585 kg, all-wheel drive | Audi seven-speed S tronic dual clutch | road |
| Nissan RB26DETT | Nissan Skyline GT-R V-Spec (R34), 1635 kg, all-wheel drive | Getrag six-speed manual | road |
| Toyota 2GR | Lotus Evora (2012), 1457 kg | Toyota six-speed manual, close ratios | road |
| Chevrolet LT2 | Chevrolet Corvette Stingray Z51 (C8), 1729 kg | Tremec eight-speed dual clutch | Pilot Sport 4S |
| Chevrolet LT6 | Chevrolet Corvette Z06 with the Z07 package (C8), 1711 kg | Tremec eight-speed dual clutch | Pilot Sport Cup 2 R |

The S2000's primary reduction, the Road Glide's primary chain and belt, the RC51's, the Speed Twin's and the Hypermotard's primary gears and chain, and the Evora's and RS 3's second final drives
are folded into the ratios or final drive shown, so
the overall gearing is the real car's. **Reset gearing** goes back to the car's own gearbox. The other
presets get a car and a six-speed fitted to the engine.

Loading an engine preset or importing an engine puts every launch setting back on auto, or on stock.

The sheet shows a timeslip: 0–60 mph, and the quarter and half mile with the speed at each. Each is timed
from the moment the car has rolled a foot, where a drag strip's clock starts, and where American road
tests, and the makers' figures from them, start theirs. Below that it plots crank power, torque, volumetric efficiency and intake
pressure against rpm, one colour per gear.

## Dyno

**Start dyno pull** runs one pull at full throttle on an engine dyno. The crank drives the dyno's absorber
directly, in one gear at 1:1, so there is no car, tyres or gearbox. The absorber is a brake under a speed
controller, as on an eddy-current or water-brake dyno. It holds the engine at the **Pull from** speed, each
cycle's mean within 100 rpm of it, for a second, then lets it speed up at the **Sweep rate** to the **Pull to** speed. All the way it brakes with
whatever torque holds the engine to that sweep, so its resistance rises and falls with the engine's torque.
It only ever brakes, and never drives the engine.

- **Pull from** is where the sweep starts. Auto is a quarter of the rev limiter, and at least 2000 rpm.
- **Pull to** is where it ends. Auto is just under the rev limiter.
- **Sweep rate** is how fast the engine speeds up through the pull. Auto is 500 rpm/s, an engine dyno's
  steady sweep. A slower sweep gives a turbo longer to spool at each speed, so it reads more boost low down.

The sheet plots the same crank power, torque, volumetric efficiency and intake pressure as a launch's, from
one engine cycle at a time. Because the sweep is slow and steady, little of the engine's torque goes into
speeding up its own flywheel, and the curve comes out close to what the engine makes held at each speed.

## Valves

- **Valves per cylinder** chooses a two-valve head (one intake, one exhaust) or a four-valve head
  (two of each). The **Exhaust valve** and **Intake valve** sizes are for each valve.

- **Cam profile switching (VTEC)** gives each valve a second, high-speed lobe. **Switch to high cam at**
  sets where the valves change over to it, and back from 150 rpm lower; at 0 there is one profile. The
  **High cam** sliders set its lift and timing, and the valve timing above is then the low-speed lobe's.
  The line above them says which lobes the valves are on. See
  [Cam profile switching](engine.md#cam-profile-switching).
- **Variable valve timing** is the ECU's cam map. The timing above it is the cams' rest position.
  **Intake cam** advances the intake cam and **Exhaust cam** retards the exhaust cam, each set for
  low speed and for high speed. **Low speed is** and **High speed is** say where those hold: up to the
  first the cams take the low-speed settings, from the second the high-speed ones, and in between the
  map blends in a straight line. It applies under load only: with the throttle under 10% open the cams sit
  at rest, for a steady idle, and it applies in full from 50%. **One phaser for both cams** moves the
  whole cam by the intake's advance, as on a pushrod engine. The line above the sliders says where
  the cams are now. See [Variable valve timing](engine.md#variable-valve-timing).
- **Pushrods** draws the valvetrain as a pushrod engine has it: one camshaft in the block, in the valley
  of a V, its lobes lifting tappets, pushrods straight up the banks from them, parallel to the cylinders,
  and rocker arms on the heads reaching across from the pushrods to the valves. Pushrods work a two-valve
  head, its valves upright in a row along it, so turning them on sets two valves a cylinder, and choosing
  four valves turns them off. Off, each row of valves has a cam over it, pressing on a bucket on each valve. Either
  way every lobe is cut to its valve's lift and turns at half the crank's speed, set round by the cam's
  phaser. With cam profile switching each pair of valves has three lobes and three finger rockers under
  them, on a rocker shaft, as Honda's VTEC has: the outer two on the low-speed lobes press the valves, and
  the middle one swings on its own under the high-speed lobe until, at the switch speed, the pin slides
  out through all three and they move together, the valves then on the middle lobe. It changes only the
  drawing: the simulation follows the valves' lift and does not model what drives it.

## Intake

- **Intake runner length** and **Intake runner bore** size each cylinder's runner from the plenum
  to its valve. At 0 they are sized for the engine: the bore from the valves, the length tuned to
  three quarters of the rev limit. Longer makes more low-rpm torque, shorter more high-rpm power.
  See [The intake](engine.md#the-intake).
- **Short runner length** makes it a two-stage intake: a second, shorter set of runners, which the
  manifold switches to at **Switch to short runners at**, and back from 150 rpm lower. The long runners
  are then the ones **Intake runner length** sets. At 0 there is one set.
- The line at the top of the section gives the runners' size and the speed they are tuned for,
  where they ram the charge in hardest and the torque peaks near, and on a two-stage intake which set
  it is on.
- **Plenum volume** is the manifold the runners draw from. At 0 it is one and a half times the
  engine's displacement.
- **Dual plenums**, on a V or a boxer whose banks have heads of their own, divides the plenum down its
  middle: each bank's runners draw from their own half, each with its own throttle body. **Open
  balance valves at** joins the halves through two valves in the wall between them at that speed,
  **Shut balance valves at** parts them again at that one, and **Reopen balance valves at** joins them
  again from there to the rev limit; coming down, each happens 150 rpm lower. At 0 the first keeps them
  shut, the second leaves them open to the rev limit, and the third leaves them as the other two do. See
  [The intake](engine.md#the-intake).
- **Throttle bore** sets the throttle's size: each throttle body's, with dual plenums. At 0 it is sized
  for the engine's airflow at full throttle and 7000 rpm, through both together with dual plenums. The
  idle air valve is sized for the engine whatever this is, so a small throttle body still idles.
- **Airbox volume**, **Snorkel length** and **Snorkel bore** shape the inlet tract the throttle draws
  its air through, and so the intake's note and its hiss. The view draws it, with the plenum, the
  throttle body and its butterfly, on top of the engine. At 0
  the airbox is four times the engine's displacement and the snorkel a little wider than the throttle. With
  dual plenums each throttle body has a tract of its own, mirrored either side of the engine: the airbox
  volume is the two airboxes' together, and the snorkel's length and bore each one's. A turbocharged
  engine has none.

## Turbocharger

Turbos are part of the exhaust: you put them in the view and pipe them up.

- **Place a turbo**, in the view's toolbar, puts one down where you click, level with the exhaust ports.
  Near the open end of a pipe it snaps onto it, its inlet flange on the pipe's end, turned to take it, and
  the pipe is attached as it goes down. Right-click, press <kbd>Escape</kbd> or click the button again to
  stop.
- **Attach pipes** by drawing into its inlet, the flange on the side of its turbine: see
  [Drawing pipes](#drawing-pipes). A pipe drawn to the inlet curves round to meet the flange square, in
  one smooth bend: an S where the flange is off to one side of the pipe but facing the same way, and
  straight only where it is on the pipe's own line. Joined at both ends, the bend is not edited: it shows
  in its menu, greyed out, and follows whenever the pipe before it or the turbo moves. Taking the
  turbo out takes the bend with it. Nothing is added at its outlet: until a pipe is drawn from it, the turbine
  exhausts straight to the air at its outlet flange, and still makes boost. Start drawing on the outlet
  flange to make the downpipe. Several pipes can feed one turbo.
- **Click a turbo** to select it and move or turn it with its
  [triad](#the-triad), whose axes are the turbo's own: red along its shaft. The pipes feeding it follow,
  their curves into the inlet fitted again, and the pipe drawn up to each curve left as it was.
  <kbd>Delete</kbd> takes it out, leaving the pipes that fed it open.
- **Right-click a turbo** for its menu: how it is running, its boost, its shaft speed, the pressure the
  exhaust works against at its inlet, and whether its wastegate is open, its blow-off valve is venting or a
  compressor is surging; what feeds it; its settings; and **Take this turbo out**. See
  [The turbocharger](engine.md#the-turbocharger).
- **Changing the engine's layout** keeps it turbocharged, with a turbo for each bank: one on an inline
  engine or a single, one under each bank of a V or a boxer. Each sits halfway along the engine, out from
  its bank's ports the way they point, as close in as leaves it clear of the engine, with its shaft along
  the engine, its outlet facing rearwards and its inlet facing the ports. Every port of the bank is piped
  straight into the inlet. From the outlets, each bank has its own collector, or with the banks merged,
  the two downpipes meet behind the engine.

A turbo's menu has its settings:

- **Boost** is what its wastegate holds.
- **Turbo size** is its compressor's airflow at full speed. Small spools early and runs out of breath
  at the top; big lags and holds its boost to the limiter. At 0 it is sized for the engine.
- **Intercooler** is how much of its compressor's heating it takes back out of the air it delivers.
- **Blow-off valve** vents the charge when the throttle shuts on boost: to the atmosphere, back to the
  compressor inlet, or, with none, not at all, so the compressor surges.

**Keep turbos in sync**, in the turbos' menu, opened from the small arrow in the corner of the Place a turbo
button or by right-clicking it, decides whose settings they are. On, as it starts, every turbo runs on the
same settings, and setting one in its menu sets them all. Off, each keeps its own, starting from the ones
they shared, and a turbo placed while it is off starts from them too. Syncing again puts every turbo on the
first one's. A turbo set to a higher boost than another holds its wastegate shut while the other's opens, and
can push the other into a surge.

**Turbo sound**, in the Listener section, sets the level of the turbos' own sounds: the whine, the blow-off
valves, the flutter and the wastegates. See [The turbocharger's sounds](acoustics.md#the-turbochargers-sounds).

## Combustion

- **Ignition advance** is the spark timing for a charge that burns over the reference **Burn
  duration**. An advance map moves it for each charge, so the spark fires later at low rpm and
  earlier at part throttle and high rpm. See [The flame](engine.md#the-advance-map).
- **Burn duration** is how long the charge takes to burn at full throttle, stoichiometric, at
  10 m/s mean piston speed. Each cycle burns faster or slower than this from its own flame speed.
- **Mixture** sets λ, the air-fuel ratio as a multiple of stoichiometric: below 1 rich, above 1
  lean.
- **Overrun fuel cut** stops the fuel with the throttle shut above 1500 rpm, until the engine
  drops below 1200 or the throttle opens. Above an 800 rpm idle, both are 700 and 400 rpm above the
  idle speed instead. Off, it behaves like a carburettor and keeps firing weakly.
- **Overrun crackle** is a "pops and bangs" map. For up to 3 s after the throttle shuts above
  2500 rpm, it keeps the fuel on and fires the spark late, skipping it on some cycles, so the
  exhaust pops. **Crackle** sets how hard it works. See [Afterfire](engine.md#afterfire).
  The readout shows **crackle** while the map runs and **pop** as the exhaust afterfires, from any
  cause.

## The camera

- **Left drag** to orbit, **right drag** to pan, **scroll** to zoom.

## Editing the exhaust

Each pipe is made of segments. There are two kinds:

- `pipe`: a tube with a diameter at each end, tapering in a straight line between them where they
  differ, as in headers, megaphones and reverse cones, or the same all the way along where they don't.
- `chamber`: a sudden widening into a can, then back down, like a muffler. Its **Shape** can
  be round, oval or rectangular. A non-round can is sized by **Width** and **Height**.
  **Inlet offset** and **Outlet offset** move its pipes off the centreline along the width.
  Offset pipes into a wide can make it ring across its width, which a centred pipe can't do.
  See [Chamber shapes](acoustics.md#chamber-shapes).

The exhaust's tools are in the toolbar over the view: **Draw a pipe**, **Place a pipe**, **Bend a pipe**,
**Equal-length header** and **Place a turbo**. The tool that is on shows a card beside the toolbar with its
hint and settings. Right-click or press <kbd>Escape</kbd> to leave a tool.

How to edit:

- **Click a pipe** to select the segment you clicked. The handles move onto that pipe.
- **Right-click a segment** to open its menu: the segment's settings, and for the pipe it is in, what else
  applies to it. Right-clicking a junction opens the junction's menu. <kbd>Escape</kbd> or a click outside
  closes it.
- **Place a pipe** puts down a straight pipe attached to nothing: click where it should start. It runs
  along the crank, level with the ports, as long as the engine is, at the ports' bore. Select it
  and its triad moves it with the arrows and squares and turns it with the rings, as a whole. It carries
  no gas, and the simulation does not hear it, until a pipe is drawn into one of its ends: that attaches
  it, the end becoming a junction fixed where it was, and from then on it is a pipe like any other. Drawn
  into its far end, it is turned round where it lies to carry on from the pipe drawn into it, with no
  extra pipe added. A pipe fed by a cylinder, finished on at its end, is a merge instead: the pipe drawn
  into it curves in beside it at a junction fixed where the pipe ended. Nothing is added after the
  junction: draw the pipe that carries the merged flow on from it (right-click it, then **Draw a pipe from
  here**). Until you do, both pipes end in open air there.
- **Turn or move the selected pipe** with its [triad](#the-triad), which sits where the pipe starts.
  - A **loose pipe** is free: its three rings swing it every way about where it starts, turned with the
    pipe, red along the way it sets off.
  - A pipe that **starts at a connection** is held there: it turns only about the way its opening
    points, by one red ring round it, so where it sets off from, and the way, stay put and the rest of it
    swings round. The ring always sits on the pipe's opening: for a pipe leaving square, that is the axis
    of the exhaust port, the turbo outlet, the pipe it carries on from or its junction; for one leaving at
    an angle, the pipe's own. A straight pipe along that axis would turn to no effect, so it has no ring.
  - Either way the whole pipe turns as one piece: every segment keeps its length and the corners and bends
    between them stay as they are.
  - With a **bend further along** selected, the triad sits where that bend starts instead, with one red
    ring round the way it sets off: it rolls the bend, and the rest of the pipe after it, so it bends
    another way, while the pipe before it stays put.
  - Its **arrows and squares** move the junction the pipe starts from, and the pipe with it, and every
    pipe into the junction bends in to follow; from then on it stays where it was put. A pipe from an
    exhaust port has no arrows, since the port is part of the engine.
  - A bend the pipe finishes in, fitted to what it joins, re-forms. Change a single segment's length or
    bend in its menu.
- **Drag a diameter ring**, the pale blue one hugging the pipe, to make the pipe wider or narrower
  there: the ring at a pipe's start sets its inlet, and the one at each segment's end sets that end. Zoom
  in for finer control. The ring moves exactly as far as your pointer does in the scene.
- **Pipes match where they meet.** Each segment starts at the diameter the one before it ends at, whichever
  end you change. A bend fitted to what a pipe joins tapers from the pipe's diameter to the diameter of
  what it joins, a turbo's inlet or the pipe it merges into, and a pipe leaving a turbo starts at the
  turbo's outlet bore. At a junction, resizing one pipe's end there, by its ring or in its menu, resizes
  every other pipe end there that was the same bore, so pipes joined at a bore stay joined at it. Ends that
  were a different bore, as a header's primaries are from their collector, are left as they are.
- **Or type exact sizes** into the segment's menu. A typed value takes effect when you press
  <kbd>Enter</kbd>, leave the field, or use the spinner arrows. The handles and the menu both edit
  the same segments of the selected pipe, so they always agree. The buttons in the segment's head move
  it earlier or later in the pipe, duplicate it or delete it.
- **Apply to every cylinder**, in the menu of a cylinder's pipe, keeps all the cylinders' pipes the same
  while you edit one. Turn it off to build headers of different lengths.
- **+ pipe / + chamber** add a segment: in the menu of the segment a pipe ends in open air with, to carry
  the pipe on, and in a junction's menu, to carry on out of it.

### The triad

A selected pipe or turbo has a triad: handles along its own three axes, red, green and blue. A pipe held
to a connection has only the ring round the connection's axis, and arrows only where it leaves a junction
of its own.

- **Drag an arrow** to move along that axis.
- **Drag a square** to move in the plane of the two arrows beside it.
- **Drag a ring** to turn about its axis.

Hold <kbd>Shift</kbd> to move in 5 mm steps and turn in 15-degree steps. A snapped turn lands on
15-degree steps measured from the engine's axes, not from where the part started, so a part set at an
odd angle squares up to the engine, parallel or perpendicular, as it snaps. Hold <kbd>Ctrl</kbd> to line
the handles up with the engine's axes rather than the part's.

Bends don't change the sound. The model only cares about each pipe's length and how wide it is
along that length. So folding a 1.4 m pipe to fit on screen doesn't change the note. This isn't a
shortcut: real sound waves in a pipe behave the same way.

## Drawing pipes

Click **Draw a pipe** to switch to draw mode.

1. **Start** from an exhaust port, a junction or the open end of a pipe (to extend it). A turbo's outlet
   flange, with no pipe on it yet, is a place to start too. A pipe cannot start from the side of another,
   only join it there.
2. **Click** to add each corner. Out of an exhaust port, the first segment runs straight on out of the
   port, as long as how far along it you click, and the corners start after it. Carrying on from the open
   end of a pipe, every segment runs straight on until the pipe joins something; bend it afterwards with
   **Bend a pipe**. Each segment locks to a direction square to the engine: across it
   (red), up or down (green), along the crank (blue), or 45 degrees between two of them, drawn in the
   mix of their colours. It can also carry straight on from the pipe it leaves, drawn in white. Point
   near the direction you want and the segment snaps to it, in 25 mm steps of length. The tool's card
   says which way the next segment runs and how long it is.
   Hold <kbd>Alt</kbd> to round the turn to 15 degrees off the pipe it leaves instead, or
   <kbd>Shift</kbd> to place the corner freely.
3. **Finish** by clicking a junction, a pipe or a pipe end to join onto it, or a turbo's inlet flange to
   feed it, or a loose pipe's start to attach it. Joining something, the pipe finishes in one smooth bend that arrives along what it joins:
   square into a turbo's flange, beside another pipe at its open end, and into the flow along a pipe's
   side, so the two merge rather than meet at a corner. Into another pipe, it arrives along the pipe the
   gas carries on through, so pivoting that pipe where it leaves the junction turns the bend with it. Drawn into the side of a pipe open at both ends, such as a loose one, it makes a T: the gas goes out of both ends. Hold <kbd>Shift</kbd> as you click a pipe's side to meet it square instead, at 90 degrees, rather than curving round into its flow. Either way its end matches the bore of the pipe it joins, and follows it when that pipe's bore is changed; the preview shows it as soon as Shift is down, and the bend stays square whenever it is fitted again. The preview shows the bend it will take. Joined
   at both ends, the bend is not edited: it is greyed out in its menu, and follows whenever the
   pipe before it or what it joins moves. A junction placed only where its pipes' ends average out has no
   place of its own to arrive at, and a pipe drawn to one ends in a corner. To leave the end open,
   double-click or press <kbd>Enter</kbd>; right-click leaves it open and ends the tool. Press
   <kbd>Escape</kbd> to cancel the pipe.

Joining onto a pipe is how you make a merge, and you draw the pipe after it yourself. The simulator
solves exactly the pipe network you draw: a junction with nothing after it yet is its pipes ending in
open air there. Deleting the pipe after a merge leaves the merge the same way.

## Bending a pipe

**Bend a pipe** bends a pipe where it lies, the way a tube is bent: it keeps its length.

1. Click a straight or a bend of any pipe, or choose **Bend this pipe** (**Bend it again** on a bend) in
   its menu. Two rings appear where it starts, each lying in one of the pipe's own planes: one in its
   up-and-down plane, one in its side-to-side plane.
2. Drag the ring of the plane to bend in. The whole straight curves into one arc in that plane, by as
   much as you drag; hold <kbd>Shift</kbd> for 15-degree steps. A ghost of the pipe shows where it is
   going, and the tool's card reads the bend out: "bend 60° up and down, radius 286 mm".
3. Let go, and the pipe takes that shape.

The straight stays as long as it was, so its radius is its length over the turn. Where it starts stays
put; the rest of the pipe swings round. It turns no tighter than one and a half bores: a straight too short
for the turn asked of it turns only as far as that allows, and the tool's card says so. A bend is bent again
from the straight it was, so it can be turned further, into the other plane, or back to straight. A pipe
joined at its far end by a fitted bend is fitted to its old length again once it is bent (**Pipe length**,
below). Cans, and the bend fitted into what a pipe joins, are not bent this way.

In its menu a bend shows its **Bend** angle and **Radius**, which can be typed. Changing them keeps the
pipe its length: the straights either side of the bend give or take what it gains or loses. Between two
straights, **Before** slides the bend along the pipe, the straight after it giving way.

**Pipe length** shows in the segment menu of a pipe joined at its far end by a fitted bend. Type a
length and the pipe is fitted to it, keeping every bend smooth: its last straight is lengthened or
shortened, and where that is not enough, it takes a swing on its way, out and back in two bends, as the
inner primaries of equal-length headers do. The swing is fitted and locked with the bend, and comes off
with it. Asked for less than the pipe can smoothly be, it is made as short as it goes, and a notice says
how short that is.

## Equal-length headers

**Equal-length header** runs pipes all the same length from any openings to one collector: from exhaust
ports with nothing on them, bending straight out of them, or on from the open ends of pipes. The primary
length then tunes the engine: see [Headers scavenge](engine.md#the-intake).

1. Click an opening's dot to pick it or leave it out.
2. Drag the triad's arrows to put the collector where it goes, and its rings to point it. A ghost shows the
   pipes: those with furthest to come reach it at the length, and the nearer ones take a swing on the way
   to make theirs up, as real equal-length headers do. No bend turns tighter than one and a half bores.
3. **Pipe length** sets how long each is; until it is set, it is the shortest that reaches. On a V or a
   boxer, **Mirror ports onto other bank** gives the ports on the bank away from the triad the mirror
   image, merging at the mirrored place; off, every opening merges at the triad. Open pipe ends always
   merge at the triad.
4. **Apply** or <kbd>Enter</kbd> builds it; <kbd>Escape</kbd> or right-click abandons it.

Each primary finishes in a fitted bend into the collector, and the swing and the bend are locked in its
menu.

## Selecting and deleting

- **Click a junction**, where pipes meet, to select it; its menu shows which pipes meet there. Junctions are not
  drawn, since joined pipes merge in bends of their own; you pick one by a small sphere where its pipes
  meet, which shows in green while it is selected.
- <kbd>Delete</kbd> or <kbd>Backspace</kbd> removes the selected segment or turbo.
- **A junction is not deleted on its own.** It goes when the pipes attached to it do: once only one pipe
  is left at it, it dissolves, and a pipe running straight through it joins back into one piece. A pipe drawn
  from a turbo's outlet goes with the turbo.
- **Deleting a segment in the middle of a pipe** leaves the segments after it as a loose pipe, where they
  lie, still joined to whatever the pipe ran into; the pipe before the gap ends in open air. A bend fitted at
  a pipe's end re-forms from wherever the pipe now ends.
- **Deleting a fitted bend** takes the pipe off whatever it joined, ending where it was drawn to; a pipe
  that is nothing but a bend goes whole. Where it was the only pipe in, the pipes carrying on from it are
  left loose, where they lie: a placed pipe it was drawn into is loose again. Taken off the side of a straight pipe, the two halves that pipe was
  split into for it merge back into one, and so does the segment the join cut in two: two straight
  segments in line, of one taper, always become one. The last pipe into a turbo takes the pipe drawn from its outlet with
  it, unless pipes carry on from that.
- **A pipe in the middle can be deleted**, one running between junctions with pipes carrying on after it:
  delete its last segment and it goes. Where it was the only pipe into the junction at its end, the pipes
  that carried on from it are left as loose pipes, where they lie, still attached to whatever was drawn on
  from them; the simulation does not hear them until they are attached again. A cylinder's own pipe is not
  deleted, since every cylinder needs one: emptied, it frees the pipes after it the same way.
- A turbo whose outlet pipe has pipes after it is not taken out; a notice says so. And the only pipe into a
  junction is not taken off it when an edit leaves it short.

## Sharing

The whole setup is saved in the page URL (the part after `#`): the engine, its exhaust, and the
launch's car and settings. So you can share an engine and exhaust you like as a link. Refreshing the
page also keeps what you were working on.

**Export engine**, under the engine preset menu, saves the engine as a file: its layout and settings,
and its exhaust as drawn, as JSON named for its layout, `engine-v8-crossplane.json` say. **Import
engine…** loads one in place of the engine you have. A file that is not an exported engine is refused,
and the engine is left as it was. An exhaust in the file that does not fit its engine, one edited by hand
to another cylinder count say, is replaced by one built from its pipe and collector, and a notice says so.
