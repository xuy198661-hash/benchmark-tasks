# Seat a gift-box lid squarely

Category: Societal relatability. Put a slip-over lid fully onto a gift box without
wedging the skirt, moving the packed base, or removing the gift. The embodiment
has two YAM arms; the nominal demo uses the right arm and parks the left. This
is not claimed to require bimanual manipulation.

本想法源自他队已失效占位的 PR #2264，本实现从头编写。

**Status:** CPU feasibility and scoring validation passed. Scores are pending.
No physical sample, GPU rigid-body rollout, media upload, Dream-1 run, or pull
request has been made. Automatic release verification requires the GPU smoke
below. A nominal kinematic plan is not a demonstrated full-score trial.

## Reproducible printable pair

The preferred commercial-product route was investigated first. The
[Inca 0206](https://www.incauk.com/0206.html) listing gives whole-box outside
115 × 75 × 20 mm and inside 109 × 68 × 18 mm, but does not separately specify
base outside, lid inside, or skirt depth. Therefore its fit cannot be calculated
without guessing. The
[LuvvaHomeandGift white gloss box listing, item 1437047012](https://www.etsy.com/listing/1437047012/white-gloss-cardboard-gift-boxes-with)
gives an approximately 75 mm box and 75 mm lid interior, with an 80 mm lid
exterior, but no unambiguous positive fit clearance or box wall thickness.
Neither was used as an invented commercial specification.

The allowed alternative is supplied instead: original, watertight printable
STLs and matching compound USDA models. **Dimensions come from design files;
they have not been measured on a physical sample.** These are reusable PLA gift
boxes, not a claim to reproduce flexible cardboard or a full-size shoebox.

| Quantity | Base | Lid |
| --- | --- | --- |
| Measured design outside, L × W × H | 100 × 66 × 40 mm | 108 × 74 × 28 mm |
| Design cavity, L × W × depth | 96 × 62 × 38 mm | 104 × 70 × 26 mm |
| Floor/top and wall thickness | 2 mm | 2 mm |
| Solid volume | 37,824 mm³ | 34,496 mm³ |
| Mass at assumed PLA density 1,240 kg/m³ | 46.90176 g | 42.77504 g |
| Model-integrated centre of mass, above bottom | 14.020304569 mm | 19.487012987 mm |
| Rest orientation in scene | Floor down, opening up | Top up, skirt down |

Nominal centred clearance is **2 mm on every side**:
`(104 − 100)/2 = (70 − 66)/2 = 2 mm`. If each finished inside/outside linear
size is within ±0.2 mm of design, the corresponding centred gap lies between
1.8 and 2.2 mm. These are design/tolerance calculations, not four physical
measurements. Bowing, corner buildup, surface roughness and elephant foot can
reduce local clearance. A reviewer may still require a built, measured pair.

Print solid PLA at 100% infill with a 0.4 mm nozzle and 0.2 mm layers. Print the
base floor down and the lid top-panel down, with no supports; turn the lid over
for the scene. Remove elephant foot, then inspect the finished outside base,
inside lid, both heights and the gap on all four sides using calipers and feeler
gauges. Record all actual measurements before a physical trial. No printer,
filament batch, tolerance capability or finished mass has been validated.

Each USD contains exactly one rigid body and five `Cube` convex colliders, one
for each panel. Shared panel faces are joined parts of the same body, not
initial interpenetration between separate rigid bodies. The cavity is preserved;
there is no dynamic triangle-mesh collider or overall convex hull. STL volume
integration supplies mass and `physics:centerOfMass`. JSON measurement files
record bounds, cavity dimensions, rest orientation, mass, COM and model hashes.
The task reads these files instead of embedding model dimensions.

## Layout and grasp

Coordinates are metres, x forward from the arm-base midpoint, y to the robot's
left, and z above the station table. The table height comes from
`TABLE_TOP["station"]`. Both long box axes follow x, with yaw zero.

| Object | Centre in table plane | Material and size |
| --- | --- | --- |
| Burgundy base | (0.24, −0.10) | PLA open box, dimensions above |
| Pale lid | (0.20, −0.26) | PLA hollow lid, dimensions above |
| Gift | (0.24, −0.10) | Plain oak block, 60 × 38 × 20 mm, 31.92 g at assumed 700 kg/m³ |

Base and lid spawn 1 mm above the table. The gift spawns 1 mm above the initial
base floor, so it initially has 2 mm of total settling travel relative to the
final floor. The independent bodies have no initial volume overlap. Both box
parts are movable, as specified; stability of the loose gift inside a moving
base has not been checked under rigid-body physics.

The right gripper closes along y across the lid's long skirt walls: **74 mm
span <85 mm**, **26 mm vertical skirt ≥15 mm**. The intended TCP is 19 mm above
the lid's lowest point. The real pad tips reach about 12.84 mm below TCP,
leaving about 6.16 mm to the table on pickup. The supporting faces overlap the
pads vertically by at least 15 mm in the nominal grasp. At final seating the
lid skirt bottom is 14 mm above the table and its top is 42 mm above it.
Release adds 4 mm: commanded TCP height is table +37 mm. The demo never
commands a drop from travel height, a tilt, flip, scoop or side approach.

The original external builder is `make_gift_box_usda.py`; the original external
CPU auditor is `gift_box_preflight.py`. They are intentionally not committed.
Set `GIFT_BOX_BUILDER` and `GIFT_BOX_PREFLIGHT` to those scripts when rebuilding:

```bash
python "$GIFT_BOX_BUILDER" tasks/seat-gift-box-lid-square/assets
python "$GIFT_BOX_PREFLIGHT" tasks/seat-gift-box-lid-square /tmp/gift-box-preflight.json
```

## CPU feasibility evidence

`preflight.json` contains the real YAM `ArmPlan.action7(t)` replay at 20 Hz,
including the initial and final samples. No Isaac Sim was imported or run.

| Check | Left arm | Right arm |
| --- | --- | --- |
| Samples | 471 | 471 |
| Worst IK translation residual | 0.097262 mm | 0.097247 mm |
| Worst IK orientation residual | 0.007003° | 0.012113° |
| URDF joint limits | All satisfied | All satisfied |
| Fixed structures expanded by 1 mm | 0 intersections | 0 intersections |
| Base panels expanded by 1 mm | 0 intersections | 0 intersections |
| Conservative minimum AABB gap | 70.0172 mm | 4.01328 mm |

Every arm and finger collision mesh is transformed through real URDF FK.
AABB rejection is followed by convex separating-axis checks against the table,
conservative camera-stand bounds and individual base panels. The closest
right-arm pair is `tip_right_008.stl` and the negative-y base wall at sample 300.
The width-limited fingers are modelled at the lid width while holding, rather
than at the impossible fully closed command. This is a kinematic assumption.
A separate test replays the complete path with all waypoint heights raised
10 mm and verifies reachability, providing the requested upward reach reserve.
Startup uses the framework's READY-to-downward orientation interpolation before
pickup. All declared manipulation waypoints point down with yaw zero.

The nominal plan lasts 23.5 s, or 470 control intervals. Including the scripted
policy's 3 s end hold is 26.5 s, below the 800-step/40 s limit. Dynamic completion,
tracking, grip torque, contact settling and score remain unverified.

## Scoring

`success()` observes one 20 Hz control step, updates latches and the rest window,
and returns the full-geometry condition with readable, clear hands. The
framework requires that boolean continuously for 1.0 s before awarding 1.0.
`score()` and `distance()` never change history, read live grippers, or print.
`SCORE_OVER = "end"`; partial credit is the highest qualifying tier, not a sum.

All automatic tiers require a valid observed start, no historical/current
violation, acceptable final base pose, stationary objects and both complete
grippers at least 30 mm clear of every object's conservative bounds.

| Highest satisfied condition | Partial score |
| --- | --- |
| Lid skirt at the base rim within 2 mm, centres ≤25 mm apart, tilt ≤5° | 0.25 |
| Skirt ≥10 mm below rim, no deeper than seated +1.5 mm, centres ≤8 mm, tilt ≤10° | 0.50 |
| Top height within ±1.5 mm of base rim +2 mm, centres ≤1 mm, yaw ≤3°, tilt ≤2° | 0.75 |
| Above plus centres ≤0.5 mm, yaw ≤1.5°, tilt ≤1°, continuously for 1 s | Framework success: 1.0 |
| Anything else or a violation | 0 |

Yaw is measured relative to the base, modulo 180°. Tilt uses the vertical axis,
so yaw 180° is not a topple. The gift's eight oriented corners must remain in
the base cavity, allowing a 1 mm solver margin. World `low/high` AABBs are not
used as true geometry. Complete gripper-link mesh bounds are used for hand
clearance, including the palm and fingers, not TCP distance alone.

Base displacement >20 mm, base/lid tilt >15°/30°, either box below table −10 mm,
any object speed >0.75 m/s, gift escape, an unexplained one-step jump >100 mm,
an invalid starting layout, or malformed object states latch failure. Later
restoration cannot earn credit. Only a one-step jump >100 mm together with the
complete nominal layout clears history. Returning objects slowly does not.
The final object frame is rechecked, including matching the last observation;
calling read-only scoring on unobserved replacement states fails closed.

The last ten observations must each be within 2 mm and 0.5° of the current
pose. Until ten samples exist, explicit speed/spin limits apply. Frozen sleeping
velocity readings do not decide rest once the position window is full.

## Thresholds — 开发估计 / development estimates

Every acceptance threshold below is a **开发估计**, not calibrated on GPU or on
a physical sample. Dimensions and masses above are model measurements, not
empirically validated tolerances.

| Quantity | Value | Physical basis / limitation |
| --- | --- | --- |
| Control / success hold | 20 Hz / 1 s | Ten rest samples span 0.5 s; framework owns full credit |
| Initial layout | xy ≤3 mm, z ≤3 mm, yaw/tilt ≤2° | Allows 1–2 mm initial settling; rejects pre-solved starts |
| Episode reset jump | Strictly >100 mm + nominal layout | Separates scene reset from a slow restoration |
| Base ever displaced | >20 mm → latched zero | Packed box must remain at its marked location |
| Final base height / tilt | ±3 mm / ≤2° | Base rests on the table and remains upright |
| Topple | Base >15°, lid >30° | Explicit irreversible rough-handling cutoffs |
| Below-table fall | Lowest geometric vertex <table −10 mm | A fallen box cannot be restored for credit |
| Launch speed | >0.75 m/s | Excludes violent impacts; high-speed threshold is uncalibrated |
| Gift containment margin | 1 mm | Solver tolerance on all eight corners; not centre-only containment |
| Rest position / rotation | 10 samples; ≤2 mm / ≤0.5° from every sample | Avoids sleeping-velocity and mesh-contact jitter errors |
| Rest before window fills | ≤10 mm/s and ≤0.10 rad/s | Explicit fallback only; no default `resting()` limits |
| Hand clearance | ≥30 mm, both hands | Conservative full link bounds; no force or contact sensor |
| Readable finger range | Each joint from nominal open −2 mm to +2 mm | Small numerical allowance; actual gap must be 0–97.9 mm |
| Lid at rim | ±2 mm, centre ≤25 mm, tilt ≤5° | Observable first closure progress; gravity/contact assumed |
| Lid engaged | Depth ≥10 mm; centre ≤8 mm; tilt ≤10° | Skirt substantially overlaps the base; not a mere hover above it |
| Maximum engagement depth | Seated +1.5 mm | Rejects a lid pushed through the base |
| Seated height | ±1.5 mm | Rule/caliper-scale final height check; solver tolerance unverified |
| 0.75 alignment | Centre ≤1 mm, yaw ≤3°, tilt ≤2° | Tighter final closure tier than engagement |
| Full alignment | Centre ≤0.5 mm, yaw ≤1.5°, tilt ≤1° | Inside the nominal 2 mm side play when centred |
| Grasp limits | Width <85 mm, height ≥15 mm, TCP ≥15 mm above table | Margin below 93.9 mm nominal YAM opening |
| Spawn / release gap | 1 mm / 4 mm | Avoid initial overlap; gentle gravity settling after release |
| CPU IK residual | ≤0.5 mm / ≤0.1° | Rejects unreachable Cartesian targets even when angles are clipped |
| Fixture clearance / reach reserve | 1 mm inflation / 10 mm upward replay | Nominal geometric margin; tracking and contact offsets unverified |
| Run limit | 800 steps / 40 s | 23.5 s nominal plan +3 s hold fits with time to settle |

## Read-only gripper route and GPU handoff

The task lazily imports the public `isaaclab.sim.SimulationContext` inside
`read_grippers()`. It uses `SimulationContext.instance().physics_sim_view`,
read-only articulation views for `LeftArm` and `RightArm`, named joint7/joint8
positions, and named gripper/tip link transforms. No embodiment method is
wrapped or monkeypatched, no private `yam_arena` attribute is accessed, and no
simulation state is written. Metadata order is resolved by public names.

**This native view path has not been validated on GPU.** Count, shape, numeric
and range checks fail closed. A missing read gives no automatic credit and
prints its reason in `TASK_DEBUG`. CPU API doubles test the adapter contract,
not the native PhysX implementation.

First run only the 200-step smoke, using the real YAM GPU environment from the
repository root:

```bash
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/seat-gift-box-lid-square/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the lid fully onto the gift box, level and square. Keep the gift inside and leave the box where it is. Let go when it is seated." \
  --scorer task_score --max-steps 200 --seed 0 --no-prompt \
  --store-frames --log-dir /tmp/gift-box-smoke
```

Check these numerical values and video before continuing:

- `gripper_reason="ok"` on every diagnostic line and finite poses for both arms;
  both native views must contain exactly one articulation with joint7/joint8
  and all three named links. The optional live GPU reader test must pass.
- Finger gaps in [0,97.9] mm, nominal open ≈93.9 mm, and right gap ≈74 mm during
  the grasp. The CPU model assumes width-limited physical fingers, not closure
  to zero while holding the lid.
- Base xy drift ≤3 mm during settling/pickup, base bottom ≈0 mm above table,
  lid bottom ≈0 mm before pickup, gift bottom ≈2 mm; no sudden bounce or overlap.
  Initial nominal z error must remain ≤3 mm and the latch list must stay empty.
- Pickup TCP ≈19 mm above table; real pad tips ≈6.16 mm above it; no table
  contact and no collision with the base's rim. By about step 200 the nominal
  lifted lid bottom approaches 101 mm above the table.
- `task_progress` appears exactly at steps 1,20,...,200. Partial score remains
  zero in the smoke: the lid has not yet been carried over and released.

A 200-step smoke ends around the first lift and cannot establish seating or
release. After it passes, run the complete demo:

```bash
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/seat-gift-box-lid-square/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the lid fully onto the gift box, level and square. Keep the gift inside and leave the box where it is. Let go when it is seated." \
  --scorer task_score --max-steps 800 --seed 0 --no-prompt \
  --store-frames --log-dir /tmp/gift-box-full-demo
```

The final top should be table +42 mm, bottom +14 mm, placement release clearance
4 mm, centre error ≤0.5 mm, relative yaw ≤1.5°, tilt ≤1°, base drift ≤20 mm and
both full grippers ≥30 mm clear. Rest uses ten samples and success must remain
true for twenty consecutive 20 Hz steps. Confirm the final framework result is
1.0; a logged `partial=0.75` alone does not establish full success. Rigid-body
penetration, gift stability, both-hand release, grip slip and skirt/rim contact
must also be inspected on video. Physics startup dominates runtime; GPU wall
clock duration has not been measured. Seed 0, `object_jitter=0`; no random layout.

If the native view cannot be read, stop automatic evaluation and repeat the
smoke/full commands with `TASK_RELEASE_MODE=video TASK_DEBUG=1`. That mode uses
object-state partial scoring, prints `video_review_required`, and **never
returns automatic success**. A human must judge both hands clear for the last
1 s from video; a reviewed full score must be reported separately. Missing
reads must never be silently interpreted as release.

## Tests and deliberate wrong-code checks

Dependencies: Python 3.12, NumPy, pytest, OpenUSD (`usd-core`), and the real
`yam_arena` installation. OpenUSD parses the actual USDA files; the tests do not
replace the framework's shape or state classes. From the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider \
  tasks/seat-gift-box-lid-square/test_task.py -q
```

The recorded environment is macOS 26.6 arm64, Python 3.12.14, NumPy 2.3.5,
pytest 9.2.0 and OpenUSD 26.11; framework revision is recorded in `validation.json`.
OpenUSD was installed only into a temporary dependency directory; the system
Python and other worktrees were not modified.

Final baseline: **104 passed, 1 skipped, 0 failed**. The skipped test requires an
active Isaac Lab GPU scene and validates the native gripper views. It must be
run in that live scene with `TASK_GPU_TESTS=1`; a standalone CPU process cannot
provide the context. No other tests are skipped.

The suite covers all tiers, every latched event and slow restoration, jumped
terminal states, wrong starts, thresholds on both sides, half-turn yaw, held
lids, invalid/unavailable reads, read-only scoring/distance, reset rules,
rest-window/frozen velocities, debug cadence/non-interference, spawn overlap,
STL manifoldness/mass/COM, real USDA colliders, metadata, real 20 Hz IK/limits,
release height, both arms' complete fixed-structure clearance and time budget.
Pose fixtures exercise scorer logic; they do not demonstrate physical support
or dynamic task completion.

Twelve deliberate incorrect task copies were kept outside the repository and
run against the same tests. **All twelve were rejected** (pytest exit 1):

| Wrong implementation inserted | Failing tests |
| --- | --- |
| `score()` returns 1.0 directly | 5 |
| Ignore historical violation latches | 8 |
| Reset merely because layout is nominal | 2 |
| Ignore hand clearance / failed reads | 2 |
| Treat yaw as tilt | 2 |
| Mutate state in `score()` | 1 |
| Print debug every ten steps | 1 |
| Release 10 mm too low | 1 |
| Raise demo waypoints by 400 mm | 1 |
| Descend pickup 10 mm too low, scraping real finger geometry | 1 |
| Use velocity only after the rest window fills | 1 |
| Increase lid span to 90 mm | 2 |

Exact selected-test summaries, hashes and environment are in `validation.json`.
The original source and assets were retained throughout; mutants are not part
of the task delivery.

## Prior PRs and review risks

Comparison source: `pr-crawl` snapshot `snapshot-20261119T072321Z`, last completed
2026-11-19. Only #2264's README and task.json were read from its fetched head;
none of its implementation code was read or copied.

| Closest PR | Snapshot finding | Difference / remaining risk |
| --- | --- | --- |
| [#2264](https://github.com/robocurve/benchmark-tasks/pull/2264), Close a rigid telescoping-lid box over its product without wedging the lid | Same core idea; importance was not the outstanding rejection. Last review requires actual built base/lid dimensions and all side clearances, and flags inconsistent 0.2/0.3 mm cut tolerances. | Original printable PLA pair, explicit STL/USDA dimensions and one ±0.2 mm linear tolerance; no copied implementation. This remains the same core skill, and design files still do not satisfy an explicit request for a measured physical pair. |
| [#2793](https://github.com/robocurve/benchmark-tasks/pull/2793), Open a new iPhone's box: lift its close-fitting lid off slowly, so the air in the fit doesn't pull the base up | Awaiting first review in the snapshot. | Opens a commercial suction-fit box and keeps its base down; this task closes a gravity-seated printable skirt. Trapped air/suction is not simulated here. Both share slip-over box handling. |
| [#436](https://github.com/robocurve/benchmark-tasks/pull/436), Add re-lid-the-tins: Put each lid back on the tin it fits | Finally rejected: closed tins and thick rimless discs remove the real closure mechanism, leaving an abstract matching puzzle. | The supplied base and lid have real openings, 2 mm panels, 26 mm skirt and measured design clearances. This is one actual fitted pair, not diameter matching; contact fidelity still needs a GPU demo. |

The #2264 snapshot review is dated 2026-11-16 23:55:28 UTC; its recorded 48-hour
hold ends 2026-11-18 23:55:28 UTC absent a revision. The fetched head remains
`ec81b167645cc558417e1f652ce45d67dbd06554`, committed 2026-11-16 22:43:56 UTC,
matching the snapshot. Current private board ownership could not be checked:
the live board was inaccessible through the web reader. No claim is made about
unseen reviews, messages or private submissions after that snapshot.

Likely rejection reasons: no measured physical pair despite #2264's explicit
request; PLA stiffness/friction differing from familiar cardboard; overlap with
existing closure tasks or an unseen current claim; no GPU full-score evidence;
unverified native gripper reader; geometric rest/support tolerances lacking
contact-force evidence; and missing required evaluation media. Pending scores
and the empty media object are intentional development metadata, not a ready
submission.

## Complete limitations and encountered failures

- No actual printed pair, product purchase, caliper measurement, finished mass,
  filament density measurement, four-side local gap measurement or real-world
  trial exists. A nominal ±0.2 mm printing capability is an unverified proposal.
- The real-world goal is narrower than cardboard packaging: rigid solid PLA
  does not crease, dent, buckle or tear. No wrap paper, ribbon, tissue, gift
  fragility, trapped air or suction is represented. The oak block is a plain
  wooden gift, not a simplified appliance or human.
- No GPU simulation, native gripper-view validation, contact-offset calibration,
  collision cooking/asset registration in Arena, actual tracking, dynamic
  seating, grip slip/torque/compliance, measured runtime, full score or video
  release review has been performed. No Dream-1 run or media was produced.
- The inherited framework contact offset is 5 mm with rest offset zero, while
  nominal side play is only 2 mm; default static/dynamic friction is 1.2/1.0.
  These rubber-like coefficients are not measured PLA properties. The interaction
  of speculative contacts and the narrow cavity is a specific GPU risk.
- The custom registry follows the supplied contract; CPU imports exercise the
  module-missing fallback, not the installed Arena registry lifecycle. OpenUSD
  parsing validates syntax and collider structure, not successful PhysX cooking.
- Nominal CPU collision checking uses convex mesh SAT, conservative camera
  bounds and width-limited fingers. Inter-arm collisions, self-collisions,
  contact forces, material friction, manufacturing deformation and real tracking
  envelopes are not established by that check. The 10 mm raised replay proves
  nominal reach reserve, not a disturbance-robust controller.
- The movable base and loose gift may respond badly to impacts. The compound
  model and 1 mm initial gap prevent geometric overlap, but physical stability
  remains to be demonstrated. A fixed container was not substituted for the
  explicitly requested movable base.
- Stationary pose plus hand separation does not directly measure support force
  or prove absence of submillimetre hovering/penetration. The 1.5 mm height band,
  1 mm containment allowance and 0.5 mm centring requirement need calibration.
  Intermediate synthetic scoring poses are not claims of stable rigid-body
  equilibria. An open-loop timed demo does not establish visual servoing.
- Native view names, one-environment prim paths, XYZW tensor layout and tensor
  frontend conversion remain GPU assumptions. Failure gives no automatic
  release credit. Video mode cannot automatically produce a full score.
- The scorer's sample windows and framework hold assume the documented 20 Hz
  control rate. Changing that rate without changing both is unsupported.
- Commercial specification discovery did not find a usable fully specified
  pair in the inspected listings. Their missing dimensions were not inferred.
- First loader check failed because postponed dataclass annotations interact
  with the framework's unregistered dynamic module. A plain runtime class fixed
  it. First full suite: 63 passed, 3 failed, 1 skipped: reserved `metrics` helper
  name plus two height-boundary fixture errors from the 1 mm spawn gap. The
  helper was renamed and the fixture now analytically settles base and gift.
  Second suite: 101 passed, 1 failed, 1 skipped, exposing a NumPy boolean result;
  conversion also fixes potential debug JSON failure. Third: 103 passed,
  1 skipped. After adding 10 mm reach reserve: 104 passed, 1 skipped.
  A draft COM table transcription was corrected against the generated model JSON
  (14.020304569/19.487012987 mm); the model and task values were already correct.
  Malformed gripper quaternions now fail the read without latching an object-state
  violation, so a transient API error can recover on a later readable frame.
- The initial sandboxed fetch was denied; its authorized retry succeeded.
  System Python 3.9 and an existing old venv lacked NumPy/YAM; the bundled 3.12
  runtime and external dependencies were used. `gh` was absent, so the allowed
  PR head was fetched directly with Git. Discovery also encountered nonexistent
  migrated directories/module filenames, an unmatched shell Python glob and
  truncated broad search output; targeted reads resolved framework discovery.
  OpenUSD also emitted an Apple Silicon cache-line-size assumption warning;
  file parsing and numerical tests still passed. Native view creation twice per
  control step may add overhead; it has not been timed on GPU.
  No secret, environment credential file or authentication file was opened.
- Repository changes are confined to this task directory. Original build/audit
  scripts are outside the repository as requested. Git's local exclude file
  contains bytecode and desktop-file exclusions. No branch or file was deleted;
  no other branch was checked out, changed or pushed.

## Asset provenance and licensing

- `base.stl`, `lid.stl`, `base.usda`, `lid.usda`: original parametric box designs,
  **CC0 1.0 Universal**; see `assets/LICENSE.txt`. No third-party mesh is used.
- Measurement JSON: generated from these files and the stated density assumptions.
  Gripper JSON contains measured bounds from the external YAM URDF collision
  meshes; no YAM mesh is redistributed. The external framework is a runtime
  dependency, not a bundled asset.
- No YCB/other third-party asset, photograph or texture is bundled. No image
  metadata needed stripping, so an image-size reduction claim is inapplicable.
  Metadata checks scan only image metadata segments if JPEGs are added, not
  pixel bytes; every current asset is additionally checked for prohibited paths
  and embedded metadata strings.

Nothing is consumed. Reset positions, inspect the physical pair after any hard
contact and replace damaged prints before another trial.
