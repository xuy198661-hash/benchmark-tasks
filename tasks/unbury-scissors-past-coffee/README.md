# Uncover the scissors and set them on the plate without going near the full mug

A pair of closed kitchen scissors (YCB) lies flat on the table under a folded paper-towel pad and a large
paper-towel sheet, with an apple on top. The robot puts the apple in the bowl, clears both towels away,
and lays the scissors on a plate with the handles toward the robot and the tips pointing away. A ceramic
mug filled with "coffee" (a dark-brown cylinder standing inside it) is next to the straight path from the
scissors to the plate.

What it tests:

- **Hidden hazard.** The scissors cannot be seen at the start; the instruction only says they are under the towels.
- **Order.** Pulling the sheet before lifting the apple rolls the apple onto the table, and that cannot be undone.
- **Keep-out around an open drink.** The obvious straight carry clips the mug's keep-out zone, so the scissors have to go round it.
- **Final pose.** The scissors must end flat on the plate, tips pointing away from the robot.

**Status:** scoring, scripted `demo()` and CPU tests are written. Model names are taken from Arena's object
registry; the models' frames and sizes still have to be read off the first GPU run (see Calibration).
No GPU run has been made yet.

## Files

| File | Purpose |
|---|---|
| `task.py` | Task file: scene, layout, `success()`, `score()`, `distance()`, `demo()` |
| `task.json` | Task metadata |
| `test_task.py` | CPU tests: scoring, demo geometry, and demo kinematics when `yam_arena` is installed |

## Running

```
python -m pytest tasks/unbury-scissors-past-coffee -q
```

```
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/unbury-scissors-past-coffee/task.py -E object_jitter=0 -E control_hz=20 --scorer task_score --max-steps 200 --log-dir runs/smoke
```

`TASK_DEBUG=1` prints one `task_progress` line every 20 steps: the step count, any zeroing or capping
reason, the current score, the largest mug tilt and shift so far, and the closest the lifted scissors
came to the mug's axis. The flag only prints; it does not change scoring.

## Scoring

Scoring reads object poses only (`ObjectState`). `SCORE_OVER = "end"`. `score()` never returns more than
0.75; 1.0 is reached only when `success()` holds for `SUCCESS_HOLD_S = 1.0` s.

| Score | End state |
|---|---|
| 1.00 | Apple in the bowl, both towels cleared, scissors set on the plate; everything at rest for 1 s |
| 0.75 | Apple and towels done, scissors on the plate but not set (wrong direction, not flat) or not at rest |
| 0.50 | Apple in the bowl and both towels cleared |
| 0.25 | Apple in the bowl |
| 0 | Anything less |

**History rules.** Each of these is latched when it happens, and the final frame is checked again:

- **Score becomes 0 for good:** coffee spilt, or the scissors leave the table. Spilt means any of: the mug tilts more than 8°; the cup's axis moves more than 15 mm from where it stood at the start; the coffee cylinder ends up more than 12 mm off the cup's axis or 20 mm above where it started.
- **Score capped at 0.5:** the lifted scissors pass within 72.5 mm of the cup's axis, at any height, or fall faster than 0.8 m/s.
- **Apple tier lost:** the apple touches the table top outside the bowl at any time.

**Episode reset.** No reset hook is used. A new episode is detected when an object jumps further than is
physically possible in one step and every object is back at its nominal position. Putting things back
slowly does not clear a latch. A wrong initial layout scores 0 (fails closed).

**Not scored.** Which arm handles which object, and where the scissors are held. Object poses cannot
show either one.

### Thresholds (development estimates, to be calibrated on GPU)

| Quantity | Value | Why |
|---|---|---|
| Mug tilt / shift of the cup's axis | 8° / 15 mm | A full mug spills at small tilts |
| Coffee off the cup's axis / raised | 12 mm / 20 mm | The cylinder has slid or been lifted out |
| Keep-out radius | 72.5 mm | Cup radius + 30 mm, around the cup's axis |
| Scissors lifted | Lowest point 15 mm above the table | |
| Scissors fall speed | 0.8 m/s | |
| Apple in bowl | Within 40 mm of the bowl centre, centre at most 60 mm high | |
| Towel cleared | Moved ≥ 100 mm, top ≤ 30 mm above the table, off the scissors and plate, not in the bowl or on the mug | |
| Scissors on plate | Centre over the plate, lowest point within 20 mm above the plate top | |
| Scissors set | Both ends over the plate, within 10° of flat, handle-to-tip line within 25° of +x | |
| Rest | 0.01 m/s, 0.1 rad/s | |

### Calibration

With `TASK_DEBUG=1`, step 1 prints a `task_calib` line with the pose and bounding box of the scissors, mug,
coffee and plate. From it and the start screenshot these values in `task.py` are fixed:

- Scissors: `SCISSORS_SPAWN_YAW` (tips toward +y), `SCISSORS_HEIGHT`, `SCISSORS_TIP_LOCAL`, `SCISSORS_BUTT_LOCAL`, and the grip point `GRASP_FROM_CENTRE` (on the handles next to the pivot).
- Mug: `MUG_YAW` (handle away from the robot), `MUG_EXTENT`, `MUG_BODY_RADIUS`, `MUG_INNER_RADIUS`, `MUG_RIM_HEIGHT`, `MUG_INNER_FLOOR`, and `MUG_AXIS_FROM_POS`.
- `POS_OFFSET`, if an object's `pos` is not the bounding-box centre it was spawned at. It must be changed together with `MUG_AXIS_FROM_POS`.
- `PLATE_HEIGHT`.

## Scripted demo

The left arm puts the apple in the bowl and moves the sheet and the pad aside. The right arm then grips the
scissors from above across the handles next to the pivot, lifts them, and pulls in toward the robot. It
passes the mug with the scissors still lying along y, so the tips trail behind. Clear of the mug, it turns
the tips to point away and lays the scissors on the plate.

## Fidelity

Rigid bodies only. The paper towels are rigid thin plates; the coffee is a rigid cylinder, so spilling is
judged from the mug's pose and the cylinder leaving the cup; the scissors are one rigid closed body; the
apple is a sphere because library models can only rest on the table.

The scissors (YCB) and the mug (HOT3D) are models from Arena's object library. The YCB Object and Model Set
is released under CC BY 4.0 (Calli et al., "The YCB Object and Model Set: Towards Common Benchmarks for
Manipulation Research", ICAR 2015).

## Licence

MIT.
