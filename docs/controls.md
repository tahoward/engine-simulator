# Controls

## Starting and choosing an engine

- **Click anywhere on the intro screen**, or click the start button in the panel. Browsers only let
  sound start after you click or press a key.
- <kbd>Space</kbd> starts and stops the engine.
- **Engine preset** loads a complete engine: its layout, firing order, sizes and a matching exhaust.
- **Layout → Cylinders** changes only the layout: single, twin, inline three to six, V6, V8, or a
  four- or six-cylinder boxer. It also builds an exhaust to suit. The **Exhaust** menu below it
  picks the pipe style: separate pipes, one collector per bank, or one collector for all cylinders.
- **Operating point → Rev limiter** sets the highest speed the engine will run. Each preset sets
  the limit of the real engine it copies. Above the limit the spark is cut, and it comes back
  200 rpm lower, so the engine bounces off the limit the way a real one does. Set **Engine speed**
  at or past the limit and the engine revs freely up into the limiter and bounces there.

## The camera

- **Left drag** to orbit, **right drag** to pan, **scroll** to zoom.

## Editing the exhaust

Each pipe is made of segments. There are three kinds:

- `pipe`: the same diameter all the way along.
- `cone`: a straight taper, as in headers, megaphones and reverse cones.
- `chamber`: a sudden widening into a can, then back down, like a muffler. Its **Shape** can
  be round, oval or rectangular. A non-round can is sized by **Width** and **Height**.
  **Inlet offset** and **Outlet offset** move its pipes off the centreline along the width.
  Offset pipes into a wide can make it ring across its width, which a centred pipe can't do.
  See [Chamber shapes](acoustics.md#chamber-shapes).

How to edit:

- **Click a pipe** to select it. The handles move onto that pipe, and the panel's segment list shows
  it.
- **Drag a blue sphere** to move the end of a segment. How far you drag sets its length. The
  direction sets the angle of the bend. The spheres sit on stalks above the pipe, so the diameter
  rings never hide them.
- **Drag a ring** to make the pipe wider or narrower. Zoom in for finer control. The ring moves
  exactly as far as your pointer does in the scene.
- **Or type exact sizes** into the panel. A typed value takes effect when you press
  <kbd>Enter</kbd>, leave the field, or use the spinner arrows. The handles and the panel both edit
  the same segments of the selected pipe, so they always agree.
- **Apply to every cylinder** keeps all the cylinders' pipes the same while you edit one. Turn it
  off to build headers of different lengths.
- **+ pipe / + cone / + chamber** add a segment to the selected pipe.

Bends don't change the sound. The model only cares about each pipe's length and how wide it is
along that length. So folding a 1.4 m pipe to fit on screen doesn't change the note. This isn't a
shortcut: real sound waves in a pipe behave the same way.

## Drawing pipes

Click **Draw a pipe** to switch to draw mode.

1. **Start** from an exhaust port, a junction, the open end of a pipe (to extend it), or the side of
   a pipe (to branch off it).
2. **Click** to add each corner. Corners snap to neat bend angles. Hold <kbd>Shift</kbd> to place
   them freely.
3. **Finish** by clicking a junction, a pipe or a pipe end to join onto it. To leave the end open,
   right-click, double-click or press <kbd>Enter</kbd>. Press <kbd>Escape</kbd> to cancel the pipe.

Joining onto a pipe is how you make a merge. The simulator solves exactly the pipe network you draw.

## Selecting and deleting

- **Click a junction** to select it and see which pipes meet there.
- <kbd>Delete</kbd> or <kbd>Backspace</kbd> removes the selected segment or junction. If you delete a
  junction that one pipe runs straight through, that pipe joins back into one piece.

## Sharing

The whole setup is saved in the page URL (the part after `#`). So you can share an engine and
exhaust you like as a link. Refreshing the page also keeps what you were working on.
