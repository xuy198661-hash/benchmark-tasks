# Clear the fruit off a counter without going near the full mug

A fruit knife lies flat on a kitchen counter under a folded paper-towel pad and a larger paper towel, with an
apple resting on top. The robot puts the apple in the bowl, moves both towels out of the way, and stands the
knife blade-down in an open holder. A mug full of coffee sits on the straight path from the knife to the holder.

What it tests:

- **Hidden hazard.** The knife cannot be seen at the start; the instruction only says it is under the towels.
- **Order.** Moving the towels, or pulling the knife out, before lifting the apple drops the apple onto the counter, and that cannot be undone.
- **Keep-out around an open drink.** The obvious straight carry clips the mug's keep-out zone, so the knife has to go round it.
- **Final pose.** The knife must end tip-down and handle-up, leaning on the rim.

**Status: draft.** The scoring logic, the object list and the CPU tests are written. Still to do: the library names of the knife and holder models, and the scripted `demo()`. No GPU run has been made yet.

## Files

| File | Purpose |
|---|---|
| `task.py` | Task file: scene description, `OBJECTS`, `success()`, `score()`, `distance()` |
| `task.json` | Task metadata |
| `test_task.py` | CPU tests of the scoring logic and object list |

## Running

```
python -m pytest tasks/unbury-knife-past-coffee -q
```

```
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/unbury-knife-past-coffee/task.py -E object_jitter=0 -E control_hz=20 --scorer task_score --max-steps 200 --log-dir runs/smoke
```

`TASK_DEBUG=1` prints one `task_progress` line every 20 steps. Each line shows the step count, any zeroing or capping reason, the current score, and the largest mug tilt and shift seen so far.

The first line also prints the knife's tip and handle-end positions, its height, the holder's rim height and inner radius, and the bowl's rim height. These are used to check the knife model's axis and to calibrate the thresholds. The flag only prints; it does not change scoring.

## Scoring

Scoring reads object poses only (`ObjectState`). `SCORE_OVER = "end"`.

`score()` never returns more than 0.75. A score of 1.0 is reached only when `success()` holds for `SUCCESS_HOLD_S = 1.0` s.

The rim and inner radius of the holder, and the rim of the bowl, are read from each model's bounding box rather than hard-coded.

| Score | End state |
|---|---|
| 1.00 | Apple in the bowl, both towels cleared, and the knife set in the holder; everything at rest for 1 s |
| 0.75 | Apple and towels done, and the knife tip is inside the holder, but the knife is not set or not at rest |
| 0.50 | Apple in the bowl and both towels cleared |
| 0.25 | Apple in the bowl |
| 0 | Anything less |

**History rules.** Each of these is latched when it happens, and the final frame is checked again:

- **Score becomes 0 for good:** the mug tilts more than 8°, or moves more than 15 mm (both stand in for spilt coffee); the holder tilts more than 30°; or the knife leaves the table.
- **Score capped at 0.5:** the knife, while being carried, passes within 72.5 mm of the mug's axis, at any height; or the knife falls faster than 0.8 m/s. A knife standing in the holder is not being carried.
- **Apple tier lost:** the apple touches the counter at any time.

**Episode reset.** No reset hook is used. A new episode is detected when an object jumps further than is physically possible in one step and every object is back at its nominal position. Putting things back slowly does not clear a latch. A wrong initial layout scores 0 (fails closed).

**Not scored.** Which arm handles which object is not scored, and neither is whether the knife was held by its handle. Object poses cannot show either one. The bowl is on the left arm's side and the holder on the right arm's side.

### Thresholds (development estimates, to be calibrated on GPU)

| Quantity | Value | Why |
|---|---|---|
| Mug tilt / shift | 8° / 15 mm | A full mug spills at small tilts |
| Holder knocked over | 30° | |
| Holder upright / moved, for success | 5° / 30 mm | |
| Keep-out radius | 72.5 mm | Mug radius + 30 mm |
| Knife carried | Lowest point > 15 mm above the counter, tip not in the holder | |
| Knife fall speed | 0.8 m/s | |
| Apple in bowl | Within 35 mm of the bowl's centre, centre ≥ 5 mm below its rim | |
| Towels cleared | Top within 15 mm of the counter, off the knife's starting footprint | |
| Knife set | Tip ≥ 25 mm below the rim; handle end ≥ 40 mm above it; axis 12–45° from vertical | A released knife leans on the rim; below 12° it must still be held |
| Rest | 0.01 m/s, 0.1 rad/s | |

## Fidelity

Rigid bodies only:

- The paper towels are rigid thin plates, heavier than real paper for solver stability under the apple.
- The apple is a sphere: library models always spawn on the counter, and the apple starts on top of the towels.
- The coffee is not simulated, and the mug is a plain cylinder.

## Assets

The bowl, knife and holder are models from the Isaac Lab Arena object library. The bowl is the YCB bowl.

Calli, B., Singh, A., Walsman, A., Srinivasa, S., Abbeel, P., and Dollar, A. M. "The YCB object and model set: Towards common benchmarks for manipulation research." *International Conference on Advanced Robotics (ICAR)*, 2015. Models released under CC BY 4.0.

## Licence

MIT.
