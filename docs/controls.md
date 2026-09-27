# Controls

## Starting and choosing an engine

- **Click anywhere on the intro screen**, or click the start button in the panel. Browsers only let
  sound start after you click or press a key. The desktop app starts the engine as it opens.
- <kbd>Space</kbd> starts and stops the engine.
- **Engine preset** loads a complete engine: its layout, firing order, sizes and a matching exhaust.
  It starts idling at 800 rpm in neutral, on a throttle opening found for that engine. Open the
  throttle and add load to go from there.
- **Layout → Equal-length headers** gives each cylinder its own primary all the way to one merge
  per collector, instead of a manifold along the ports. The primary length then tunes the engine:
  see [Headers scavenge](engine.md#the-intake). Each primary leaves its port square to the head and
  finishes in a fitted bend into the collector, at its own bore, and every one is the primary length: the
  headers are equal-length. The collector sits where the primaries with furthest to come reach it at that
  length; those nearer take a swing on the way, bulging down away from the line to the collector, to make
  theirs up, as real equal-length headers do. No bend turns tighter than one and a half bores. The swing
  and the bend are fitted, and locked in the segment list; only the straight out of the port is edited. The collector's junction is
  fixed where the primaries meet. It can be moved from its pipe's triad like any other junction; the
  bends follow it, and the primaries' lengths change with them.
- **Layout → Cylinders** changes only the layout: single, twin, inline three to six, V6, V8, or a
  four- or six-cylinder boxer. It also builds an exhaust to suit. The **Exhaust** menu below it
  picks the pipe style: separate pipes, one collector per bank, or one collector for all cylinders.
- **Operating point** has no speed control: the engine turns as fast as its torque drives it
  against friction and the **Load**, so the **Throttle**, the load and the exhaust tuning all set
  it. **Flywheel inertia** sets how quickly it responds.
- **Operating point → Rev limiter** sets the highest speed the engine will run. Each preset sets
  the limit of the real engine it copies. Above the limit the spark is cut, and it comes back
  200 rpm lower, so the engine bounces off the limit the way a real one does.

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
- **Throttle bore** sets the throttle's size. At 0 it is sized for the engine's airflow at full
  throttle and 7000 rpm.

## Turbocharger

Turbos are part of the exhaust: you put them in the view and pipe them up.

- **Place a turbo** puts one down where you click, level with the exhaust ports. Near the open end of a
  pipe it snaps onto it, its inlet flange on the pipe's end, turned to take it, and the pipe is attached
  as it goes down. Press <kbd>Escape</kbd> or the button again to stop.
- **Attach pipes** by drawing into its inlet, the flange on the side of its turbine: see
  [Drawing pipes](#drawing-pipes). A pipe drawn to the inlet curves round to meet the flange square, in
  one smooth bend: an S where the flange is off to one side of the pipe but facing the same way, and
  straight only where it is on the pipe's own line. Joined at both ends, the bend is not edited: it shows
  in the segment list, greyed out, and follows whenever the pipe before it or the turbo moves. Taking the
  turbo out takes the bend with it. Nothing is added at its outlet: until a pipe is drawn from it, the turbine
  exhausts straight to the air at its outlet flange, and still makes boost. Start drawing on the outlet
  flange to make the downpipe. Several pipes can feed one turbo.
- **Click a turbo** to select it and see what feeds it, and move or turn it with its
  [triad](#the-triad), whose axes are the turbo's own: red along its shaft. The pipes feeding it follow,
  their curves into the inlet fitted again, and the pipe drawn up to each curve left as it was.
  <kbd>Delete</kbd> takes it out, leaving the pipes that fed it open.
- The line under the button says how many turbos there are. The next gives the boost, the turbos'
  speed, the pressure the exhaust works against at their inlets, and whether the wastegate is open, the
  blow-off valve is venting or the compressor is surging. See
  [The turbocharger](engine.md#the-turbocharger).

The settings below apply to every turbo, and show once there is one:

- **Boost** is what the wastegate holds.
- **Turbo size** is each compressor's airflow at full speed. Small spools early and runs out of breath
  at the top; big lags and holds its boost to the limiter. At 0 it is sized for the engine.
- **Intercooler** is how much of the compressor's heating it takes back out of the charge.
- **Blow-off valve** vents the charge when the throttle shuts on boost: to the atmosphere, back to the
  compressor inlet, or, with none, not at all, so the compressor surges.
- **Turbo sound** sets the level of the turbo's own sounds: the whine, the blow-off valve, the flutter
  and the wastegate. See [The turbocharger's sounds](acoustics.md#the-turbochargers-sounds).

## Combustion

- **Ignition advance** is the spark timing for a charge that burns over the reference **Burn
  duration**. An advance map moves it for each charge, so the spark fires later at low rpm and
  earlier at part throttle and high rpm. See [The flame](engine.md#the-advance-map).
- **Burn duration** is how long the charge takes to burn at full throttle, stoichiometric, at
  10 m/s mean piston speed. Each cycle burns faster or slower than this from its own flame speed.
- **Mixture** sets λ, the air-fuel ratio as a multiple of stoichiometric: below 1 rich, above 1
  lean.
- **Overrun fuel cut** stops the fuel with the throttle shut above 1500 rpm, until the engine
  drops below 1200 or the throttle opens. Off, it behaves like a carburettor and keeps firing
  weakly.

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

How to edit:

- **Click a pipe** to select the segment you clicked. The handles move onto that pipe, and the panel's
  segment list shows it.
- **Place a pipe** puts down a straight pipe attached to nothing: click where it should start. Select it
  and its triad moves it with the arrows and squares and turns it with the rings, as a whole. It carries
  no gas, and the simulation does not hear it, until a pipe is drawn into one of its ends: that attaches
  it, the end becoming a junction fixed where it was, and from then on it is a pipe like any other. Drawn
  into its far end, it is turned round where it lies to carry on from the pipe drawn into it, with no
  extra pipe added. A pipe fed by a cylinder, finished on at its end, is a merge instead: the pipe drawn
  into it curves in beside it at a junction fixed where the pipe ended. Nothing is added after the
  junction: draw the pipe that carries the merged flow on from it (select it, then **Draw a pipe from
  here**). Until you do, both pipes end in open air there.
- **Turn or move the selected pipe** with its [triad](#the-triad), which sits where the pipe starts.
  - A **loose pipe** is free: its three rings swing it every way about where it starts, turned with the
    pipe, red along the way it sets off.
  - A pipe that **starts at a connection** is held to the face it starts from, as a pipe is to a flange:
    it turns only in that face's plane, by one red ring round the connection's axis, so where it sets off
    from stays put and the rest of it swings round. The axis is an exhaust port's, a turbo outlet's, the
    way the pipe it carries on from finishes, or its junction's. A straight pipe along that axis would turn
    to no effect, so it has no ring.
  - Either way the whole pipe turns as one piece: every segment keeps its length and the corners and bends
    between them stay as they are.
  - Its **arrows and squares** move the junction the pipe starts from, and the pipe with it, and every
    pipe into the junction bends in to follow; from then on it stays where it was put. A pipe from an
    exhaust port has no arrows, since the port is part of the engine.
  - A bend the pipe finishes in, fitted to what it joins, re-forms. Change a single segment's length or
    bend in the panel.
- **Drag a diameter ring**, the pale blue one hugging the pipe, to make the pipe wider or narrower
  there: the ring at a pipe's start sets its inlet, and the one at each segment's end sets that end. Zoom
  in for finer control. The ring moves exactly as far as your pointer does in the scene.
- **Pipes match where they meet.** Each segment starts at the diameter the one before it ends at, whichever
  end you change. A bend fitted to what a pipe joins tapers from the pipe's diameter to the diameter of
  what it joins, a turbo's inlet or the pipe it merges into, and a pipe leaving a turbo starts at the
  turbo's outlet bore. At a junction, resizing one pipe's end there, by its ring or in the panel, resizes
  every other pipe end there that was the same bore, so pipes joined at a bore stay joined at it. Ends that
  were a different bore, as a header's primaries are from their collector, are left as they are.
- **Or type exact sizes** into the panel. A typed value takes effect when you press
  <kbd>Enter</kbd>, leave the field, or use the spinner arrows. The handles and the panel both edit
  the same segments of the selected pipe, so they always agree.
- **Apply to every cylinder** keeps all the cylinders' pipes the same while you edit one. Turn it
  off to build headers of different lengths.
- **+ pipe / + chamber** add a segment to the selected pipe.

### The triad

A selected pipe or turbo has a triad: handles along its own three axes, red, green and blue. A pipe held
to a connection has only the ring round the connection's axis, and arrows only where it leaves a junction
of its own.

- **Drag an arrow** to move along that axis.
- **Drag a square** to move in the plane of the two arrows beside it.
- **Drag a ring** to turn about its axis.

Hold <kbd>Shift</kbd> to move in 5 mm steps and turn in 15-degree steps. A snapped turn lands on
15-degree steps measured from the engine's axes, not from where the part started, so a part set at an
odd angle squares up to the engine, parallel or perpendicular, as it snaps.

Bends don't change the sound. The model only cares about each pipe's length and how wide it is
along that length. So folding a 1.4 m pipe to fit on screen doesn't change the note. This isn't a
shortcut: real sound waves in a pipe behave the same way.

## Drawing pipes

Click **Draw a pipe** to switch to draw mode.

1. **Start** from an exhaust port, a junction, the open end of a pipe (to extend it), or the side of
   a pipe (to branch off it). A turbo's outlet flange, with no pipe on it yet, is a place to start too.
2. **Click** to add each corner. Out of an exhaust port, the first segment runs straight on out of the
   port, as long as how far along it you click, and the corners start after it. Each segment locks to a direction square to the engine: across it
   (red), up or down (green), along the crank (blue), or 45 degrees between two of them, drawn in the
   mix of their colours. It can also carry straight on from the pipe it leaves, drawn in white. Point
   near the direction you want and the segment snaps to it, in 25 mm steps of length. The panel
   says which way the next segment runs and how long it is.
   Hold <kbd>Alt</kbd> to round the bend to 15 degrees off the pipe it leaves instead, or
   <kbd>Shift</kbd> to place the corner freely.
   **Bends instead of corners:** tick **Bends** beside the Draw button, or hold <kbd>B</kbd> while
   clicking, and each click lays a smooth bend, as a tube bender makes one, rather than a corner. Point
   the way the pipe should turn to, which snaps to the same directions and colours as a corner does, and
   how far from the pipe's end you point sets the radius, in 25 mm steps and no tighter than one and a
   half bores. Where the bend ends follows from those, so there is no point in space to find: a faint
   disc shows the plane it turns in, and the panel reads it out, "bend 90° to down, radius 200 mm". With
   Bends ticked, holding <kbd>B</kbd> lays corners instead. In the segment list a bend shows its **Bend**
   angle and **Radius**, which can be typed. Changing them keeps the pipe its length: the straights either
   side of the bend give or take what it gains or loses. Between two straights, **Before** slides the bend
   along the pipe, the straight after it giving way.
3. **Finish** by clicking a junction, a pipe or a pipe end to join onto it, or a turbo's inlet flange to
   feed it, or a loose pipe's start to attach it. Joining something, the pipe finishes in one smooth bend that arrives along what it joins:
   square into a turbo's flange, beside another pipe at its open end, and into the flow along a pipe's
   side, so the two merge rather than meet at a corner. Into another pipe, it arrives along the pipe the
   gas carries on through, so pivoting that pipe where it leaves the junction turns the bend with it. The preview shows the bend it will take. Joined
   at both ends, the bend is not edited: it is greyed out in the segment list, and follows whenever the
   pipe before it or what it joins moves. A junction placed only where its pipes' ends average out has no
   place of its own to arrive at, and a pipe drawn to one ends in a corner. To leave the end open,
   right-click, double-click or press <kbd>Enter</kbd>. Press <kbd>Escape</kbd> to cancel the pipe.

## Bending a pipe

**Bend a pipe** bends a pipe where it lies, the way a tube is bent: it keeps its length.

1. Click a straight of any pipe. Two rings appear where it starts, each lying in one of the pipe's own
   planes: one in its up-and-down plane, one in its side-to-side plane.
2. Drag the ring of the plane to bend in. The whole straight curves into one arc in that plane, by as
   much as you drag; hold <kbd>Shift</kbd> for 15-degree steps. A ghost of the pipe shows where it is
   going, and the panel reads the bend out: "bend 60° up and down, radius 286 mm".
3. Let go, and the pipe takes that shape.

The straight stays as long as it was, so its radius is its length over the turn. Where it starts stays
put; the rest of the pipe swings round. It turns no tighter than one and a half bores: a straight too short
for the turn asked of it turns only as far as that allows, and the panel says so. A pipe joined at its far
end by a fitted bend is fitted to its old length again once it is bent (**Pipe length**, below). Cans,
bends, and the bend fitted into what a pipe joins are not bent this way.

**Pipe length** shows under the segment list for a pipe joined at its far end by a fitted bend. Type a
length and the pipe is fitted to it, keeping every bend smooth: its last straight is lengthened or
shortened, and where that is not enough, it takes a swing on its way, out and back in two bends, as the
inner primaries of equal-length headers do. The swing is fitted and locked with the bend, and comes off
with it. **Match** sets it to the length of another pipe into the same place, so headers drawn by hand
can be made equal-length. Asked for less than the pipe can smoothly be, it is made as short as it goes, and
the panel says how short that is.

Joining onto a pipe is how you make a merge, and you draw the pipe after it yourself. The simulator
solves exactly the pipe network you draw: a junction with nothing after it yet is its pipes ending in
open air there. Deleting the pipe after a merge leaves the merge the same way.

## Selecting and deleting

- **Click a junction**, where pipes meet, to select it and see which pipes meet there. Junctions are not
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
- A turbo whose outlet pipe has pipes after it is not taken out; the panel says so. And the only pipe into a
  junction is not taken off it when an edit leaves it short.

## Sharing

The whole setup is saved in the page URL (the part after `#`). So you can share an engine and
exhaust you like as a link. Refreshing the page also keeps what you were working on.
