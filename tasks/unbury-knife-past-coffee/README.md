# Uncover the knife and set it on the plate without going near the full mug

A small knife lies flat on the table beside a folded wad of paper towels. A large paper-towel sheet rests on the wad
with half an apple on top, cut side down, and covers the knife except for its red handle. The robot puts the apple half in the bowl, clears
both towels away, and lays the knife on a plate with the handle toward the robot and the blade pointing away. A mug
that the instruction says is full of hot coffee stands next to the straight path from the knife to the plate.

What it tests:

- **Covered blade.** Only the handle shows at the start; the blade runs on under the towels, and the instruction only says the knife is partly under them.
- **Order.** Pulling the sheet before lifting the half apple drops it onto the table, and that cannot be undone.
- **Keep-out around an open drink.** The obvious straight carry clips the mug's keep-out zone, so the knife has to go round it.
- **Final pose.** The knife must end flat on the plate, blade pointing away from the robot.

**Status:** scoring, scripted `demo()` and CPU tests are written. The knife, bowl and plate models are included
in `assets/`. Scoring rejects apples and towels below the table, allowing the existing 3 mm contact tolerance.
The layout with the wad beside the knife (every support under the apple lies on the table, and the sheet does
not touch the knife), the full-weight empty mug and the half apple's exact lowest-point check are covered by 94 CPU tests;
their simulator contacts and full demo remain unverified. `scores` and `media` remain TODO until validated GPU runs and uploads.

**GPU static calibration, commit 887679a (100 steps):** mug, wad and knife stayed put (mug tilt 0.22°, axis shift
< 0.3 mm, resting speed at most 1.3 mm/s). The apple, then modelled as a 66 mm sphere, rolled off: the sheet on the wad
settled 0.03-0.05° off level, which accelerated the sphere at about 0.0045 m/s² until it passed the wad's edge
and tipped the sheet. A sphere has no resting position on a movable support, so the apple is now half an apple
lying cut side down.

**GPU static calibration, commit 20eae38 (100 steps, run twice with identical results):** the half apple did not
move (< 0.01 mm), its box is 30.1 mm tall (the cylinder stands on its axis), the sheet sat 0.016° off level, and
the mug, wad and knife stayed put.

**GPU smoke, commit 20eae38 (200 steps):** the left arm lifted the half apple and put it 0.3 mm from the bowl's
centre, `apple_bruised=false`. Releasing it from 75 mm dropped it ~58 mm; 1.4 s later it was still rocking
(1-1.4°, 1.5 rad/s) and so not at rest. The demo now lowers it to 3 mm above the bowl floor before opening. Gripping
nudged the apple 12 mm and the sheet and wad about 3 mm; nothing was knocked over.

## Files

| File | Purpose |
|---|---|
| `task.py` | Task file: scene, layout, `success()`, `score()`, `distance()`, `demo()` |
| `task.json` | Task metadata |
| `test_task.py` | CPU tests: scoring, models, demo geometry, and demo kinematics when `yam_arena` is installed |
| `run_gpu.py` | CPU preflight, bounded GPU stages and saved diagnostics; manual instructions below |
| `assets/knife/` | The knife model (`knife.usda`, `knife_texture.png`), how it was made, and its source |
| `assets/bowl/`, `assets/plate/` | Textured bowl and plate models with concave triangle-mesh collision |
| `assets/mug/` | `mug_info.json`: measurements of Isaac Lab's mug used by `task.py`, the script that takes them, and its source |
| `assets/make_ycb_containers.py`, `assets/README.md` | Rebuild the containers; source, licence and conversion details |

## Models

The knife, bowl and plate are scans from the YCB Object and Model Set (CC BY 4.0; Calli et al., "Benchmarking in
Manipulation Research: Using the Yale-CMU-Berkeley Object and Model Set", IEEE Robotics & Automation Magazine,
2015), converted to USD and shipped in `assets/`. `task.py` registers each one in Arena's asset registry under its
own name (`knife_ycb_local`, `bowl_ycb_local`, `plate_ycb_local`), so the scene uses them through `Asset`
objects. The mug is Isaac Lab's mug model, loaded from its asset server through Arena's registry.

The official mug, scaled to 120 mm including its handle, is 114.55 mm tall. Its cup axis is 17.082 mm toward
-x from the bounding-box centre. The coffee itself is not modelled. The mug is given the mass of a full cup,
550 g (about 300 g of ceramic and 250 g of coffee), and spilling is judged from the mug's tilt and shift. An earlier
version put a rigid coffee solid inside the cup; a rigid solid never sloshes, so it added almost no information
beyond the tilt check, and fitting it inside a moving mug's convex collision shape is the main suspect for the mug
being thrown over at reset. The task file uses only the public task-file contract; it replaces no framework function.

The bowl is 160 mm wide and the plate 200 mm wide. Both are fixed kinematic bodies with triangle-mesh collision
preserving their interiors; see [the model notes](assets/README.md). Their overall heights are 54.51 mm and
20.47 mm respectively. The plate's centre surface is lower than its rim, so GPU contact calibration is still
required before changing the demo's placement height.

## Running

Run CPU preflight locally before starting the GPU instance, then commit and push the task's branch to the fork.
Use the same Git commit on both machines. The runner prints that commit and a separate hash of `task.py`;
it never starts a VM or installs software. There is no task upload archive or bundled checksum manifest.

```sh
python tasks/unbury-knife-past-coffee/run_gpu.py check
```

On a GPU instance with the existing Arena environment, clone the fork and branch. Substitute the fork URL
registered to the team for `FORK_URL` before running these commands. Keep IsaacLab-Arena, the yam_arena
installation, its Python environment, credentials and model caches intact.

```sh
source ~/IsaacLab-Arena/yam.env
git clone --single-branch --branch unbury-knife-past-coffee FORK_URL ~/benchmark-tasks-knife
cd ~/benchmark-tasks-knife
git branch --show-current
git rev-parse HEAD
python tasks/unbury-knife-past-coffee/run_gpu.py check
```

The preflight should report 94 passing tests without skips. For later revisions, in that checkout run
`git pull --ff-only origin unbury-knife-past-coffee`, compare `git rev-parse HEAD` with the pushed local
commit, and rerun preflight. If local edits prevent updating, inspect them before replacing any files.

Run **only the static calibration stage** first; leave full-demo/model runs until the reset is stable:

```sh
python tasks/unbury-knife-past-coffee/run_gpu.py calibrate
```

`calibrate` holds both grippers clear for 100 steps / 5 seconds. A score of 0 is expected because it performs no
task actions. Inspect the first and final `task_state`: `bad_layout=false`, `zero_reason=null`,
`apple_bruised=false`, mug tilt below 8 degrees and its axis near (0.320, -0.160); all objects should settle at
rest. Verify the images too: upright cup, apple still on the sheet over the wad, red handle visible. These checks are necessary
before proceeding. Also note `mug_speed_max`: the mug's jitter at rest, needed before any "mug was hit" rule
can be calibrated. Stop and return the logs if reset is unstable.

Each stage saves its console, diagnostic object states, Inspect logs/frames and exact source snapshot in a
new directory under `~/yam-eval/unbury-knife-past-coffee/`. `run-status.json` records the Git commit too.
To download the results, after the command has exited to the shell prompt:

```sh
tar -czf ~/unbury-knife-results.tgz -C ~/yam-eval unbury-knife-past-coffee
```

Download that result and **stop the instance in the cloud console** while reviewing it locally. Five seconds
is simulated time; startup, rendering and shutdown also consume billed time. The runner uses `--grader none`
(verified in Inspect Robots 1.0.0), so it exits without asking for a human verdict. Its automatic `task_score`
remains enabled.

After static calibration passes, `run_gpu.py smoke` covers the initial apple move (200 steps / 10 simulated
seconds). It does not cover the full task. After inspecting and calibrating grasping, play the
full demo (about 43.3 seconds / 866 steps, with a 1200-step maximum and the policy's final hold):

```sh
python tasks/unbury-knife-past-coffee/run_gpu.py demo
```

Only after a verified full-score demo, run Dream-1 separately with `run_gpu.py model` (3600-step maximum).
`--dry-run` on any GPU stage prints its command without running it. Plain CPU tests remain:

```
python -m pytest tasks/unbury-knife-past-coffee -q
```

The runner sets `TASK_DEBUG=1`: a `task_calib` line at step 1 covers every object's pose and bounds;
`task_progress` reports the step count, zeroing/capping reasons, score, maximum mug tilt/shift/speed (speed is recorded for calibration only) and the closest
lifted-knife approach. A JSON `task_state` line at the same intervals includes velocities and individual score
conditions, so grasping and settling failures can be investigated offline. Diagnostics do not change scoring.

## Scoring

Scoring reads object poses only (`ObjectState`). `SCORE_OVER = "end"`. `score()` never returns more than 0.75; 1.0
is reached only when `success()` holds for `SUCCESS_HOLD_S = 1.0` s.

| Score | End state |
|---|---|
| 1.00 | Apple in the bowl, both towels cleared, knife set on the plate; everything at rest for 1 s |
| 0.75 | Apple and towels done, knife on the plate but not set (wrong direction, not flat) or not at rest |
| 0.50 | Apple in the bowl and both towels cleared |
| 0.25 | Apple in the bowl |
| 0 | Anything less |

**History rules.** Each of these is latched when it happens, and the final frame is checked again:

- **Score becomes 0 for good:** coffee spilt, or the knife leaves the table. Spilt means either: the mug tilts more than 8°, or the cup's axis moves more than 15 mm from where it stood at the start. These are thresholds, not a sliding deduction: knocking a full mug is a safety failure, not a cost to trade against a shorter route.
- **Score capped at 0.5:** the lifted knife passes within 77.45 mm of the cup's axis, at any height, or falls faster than 0.8 m/s.
- **Apple tier lost:** the apple touches the table top outside the bowl at any time.

**Episode reset.** No reset hook is used. A new episode is detected when an object jumps further than is
physically possible in one step and every object is back at its nominal position. Putting things back slowly does
not clear a latch. A wrong initial layout scores 0 (fails closed).

**Not scored yet.** Which arm handles which object, and where the knife is held. Object poses cannot show either
one; holding the knife by its handle (body-frame y below -17 mm) is the next addition.

### Thresholds (development estimates, to be calibrated on GPU)

| Quantity | Value | Why |
|---|---|---|
| Mug tilt / shift of the cup's axis | 8° / 15 mm | Coffee filled to 8 mm below the rim, with about 38 mm inner radius there, reaches the rim at about atan(8/38) ≈ 12°; 8° leaves room for sloshing. The shift stands for the mug being knocked |
| Keep-out radius | 77.45 mm | Measured cup radius + 30 mm, around the cup's axis |
| Knife lifted | Lowest point 15 mm above the table | |
| Knife fall speed | 0.8 m/s | |
| Apple in bowl | Within 40 mm of the bowl centre, centre < 60 mm high, lowest point ≥ -3 mm relative to the table | Lowest point of the 66 x 30 mm cylinder from its pose: centre - (h/2)·|u_z| - r·√(1-u_z²), u_z the axis's vertical component; unchanged by yaw or flipping |
| Towel cleared | Moved ≥ 100 mm, lowest point ≥ -3 mm and top ≤ 30 mm relative to the table, off the knife and plate, not in the bowl or on the mug | 3 mm contact tolerance |
| Knife on plate | Centre over the plate, lowest point within 20 mm above the plate top | |
| Knife set | Both ends over the plate, within 10° of flat, handle-to-tip line within 25° of +x | |
| Rest | 0.01 m/s, 0.1 rad/s | |

### Calibration

The knife's size, orientation and handle are read off `assets/knife/knife.usda`. Mug dimensions and axis
come from the official source mesh and are recorded in `assets/mug/mug_info.json` (see [the mug notes](assets/mug/README.md)). The revised
keep-out radius is 77.4545 mm; metadata rounds it to 77.45 mm. CPU replay checks the knife route against this
larger radius with more than 10 mm margin. Use the GPU `task_calib` / `task_state` lines and camera frames to
verify the settled cup axis, the towel stack and the actual knife/plate contact height. `PLATE_HEIGHT`
still uses the 20 mm overall-height estimate; the measured centre surface is only 2.75 mm above its mesh bottom,
so the release trajectory remains pending contact calibration. Do not fill `scores` or `media` from CPU tests.

## Scripted demo

The left arm sets the half apple down in the bowl (released 3 mm above its floor, not dropped) and moves the sheet and the wad aside. The right arm then grips the knife
from above across the middle of its handle, lifts it, and pulls in toward the robot. It passes the mug with the
knife still lying along y, so the tip trails behind. Clear of the mug, it turns the blade to point away and lays the
knife on the plate.

## Fidelity

Rigid bodies only. The paper towels are rigid thin plates, so the sheet rests on the wad alone and does not drape
onto the knife as real paper would; the coffee is not modelled: the mug has the mass of a full cup (its centre of
mass is that of the empty shape, so slightly high), the camera sees an empty white cup, and spilling is judged from
the mug's tilt and shift; the knife is one rigid body colliding as three convex hulls,
and its scan adds a few millimetres of spurious thickness under the polished blade; the half apple is a plain red
cylinder (66 x 30 mm, 60 g) because library models can only rest on the table, not on the towels.

## Licence

Code: MIT. The YCB models in `assets/` keep their CC BY 4.0 licence.
