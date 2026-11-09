# Clear a fruit plate without going near the full mug

A fruit knife lies on a plate under a folded paper-towel pad and a large paper-towel sheet, with an apple
on top. The robot puts the apple in the bowl, clears both towels off the plate, and stands the knife
blade-down in a short glass. A mug full of coffee sits on the straight path from the plate to the glass.

What it tests:

- **Hidden hazard.** The knife cannot be seen at the start; the instruction only says it is under the towels.
- **Order.** Pulling the sheet before lifting the apple rolls the apple onto the table, and that cannot be undone.
- **Keep-out around an open drink.** The obvious straight carry clips the mug's keep-out zone, so the knife has to go round it.
- **Final pose.** The knife must end tip-down and handle-up, leaning on the rim.

**Status: draft.** The scoring logic and its CPU tests are written. The scene objects (`OBJECTS`) and the
scripted `demo()` come next. No GPU run has been made yet.

## Files

| File | Purpose |
|---|---|
| `task.py` | Task file: scene description, layout, `success()`, `score()`, `distance()` |
| `task.json` | Task metadata |
| `test_task.py` | CPU tests of the scoring logic |

## Running

```
python -m pytest tasks/unbury-knife-past-coffee -q
```

```
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/unbury-knife-past-coffee/task.py -E object_jitter=0 -E control_hz=20 --scorer task_score --max-steps 200 --log-dir runs/smoke
```

`TASK_DEBUG=1` prints one `task_progress` line every 20 steps. Each line shows the step count, any zeroing or capping reason, the current score, and the largest mug tilt and shift seen so far. The flag only prints; it does not change scoring.

## Scoring

Scoring reads object poses only (`ObjectState`). `SCORE_OVER = "end"`.

`score()` never returns more than 0.75. A score of 1.0 is reached only when `success()` holds for `SUCCESS_HOLD_S = 1.0` s.

| Score | End state |
|---|---|
| 1.00 | Apple in the bowl, both towels cleared, and the knife set in the glass; everything at rest for 1 s |
| 0.75 | Apple and towels done, and the knife tip is inside the glass, but the knife is not set or not at rest |
| 0.50 | Apple in the bowl and both towels cleared |
| 0.25 | Apple in the bowl |
| 0 | Anything less |

**History rules.** Each of these is latched when it happens, and the final frame is checked again:

- **Score becomes 0 for good:** the mug tilts more than 8°, or moves more than 15 mm (both stand in for spilt coffee); the glass tilts more than 30°; or the knife leaves the table.
- **Score capped at 0.5:** the lifted knife passes within 72.5 mm of the mug's axis, at any height; or the knife falls faster than 0.8 m/s.
- **Apple tier lost:** the apple touches the table top at any time.

**Episode reset.** No reset hook is used. A new episode is detected when an object jumps further than is physically possible in one step and every object is back at its nominal position. Putting things back slowly does not clear a latch. A wrong initial layout scores 0 (fails closed).

**Not scored.** Which arm handles which object is not scored, and neither is whether the knife was held by its handle. Object poses cannot show either one. The layout places the bowl on the left arm's side and the glass on the right arm's side.

### Thresholds (development estimates, to be calibrated on GPU)

| Quantity | Value | Why |
|---|---|---|
| Mug tilt / shift | 8° / 15 mm | A full mug spills at small tilts |
| Glass knocked over | 30° | |
| Glass upright / moved, for success | 5° / 30 mm | |
| Keep-out radius | 72.5 mm | Mug radius + 30 mm |
| Knife lifted | 15 mm above the plate | |
| Knife fall speed | 0.8 m/s | |
| Apple in bowl | Within 35 mm of the axis, centre at most 60 mm high | |
| Knife set | Tip ≥ 25 mm below the rim; handle end ≥ 40 mm above the rim; axis 12–45° from vertical | A released knife leans on the rim at about 30°; below 12° it must still be held |
| Rest | 0.01 m/s, 0.1 rad/s | |

## Fidelity

Rigid bodies only:

- The paper towels are rigid thin plates.
- The coffee is not simulated.
- The bowl, mug and glass are rigid hollow meshes.

## Licence

MIT.
