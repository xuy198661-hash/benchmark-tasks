# Sort aluminium parts with a GO / NO-GO gap gauge

Three aluminium bars must be physically inspected and sorted. Every credited
part needs a released, supported gauge reading before its final tray placement.
The revised geometry and plan pass CPU checks. The previous GPU demo scored 0.0;
the revised physical demo and Dream-1 remain pending. No media has been uploaded.

本想法源自他队已失效占位的 PR #1849，本实现从头编写，并修复了该 PR 最后一次评审指出的问题。

## Instruction and fixed setup

Gauge every aluminium block before putting acceptable parts in the green tray
and rejects in the red tray. Accept a part only if it enters the GO gap and stays
out of the NO-GO gap. Keep unchecked parts out of the finished trays, and leave
all parts settled and released. A tipped part can be righted and inspected again.

`geometry.json` specifies every Box. Coordinates are metres: x forward, y left,
z up; the station table top is 0.7475 m. No object jitter is used.

| Aluminium bar | Length × width × height | Mass | Start xy, m | Tray |
| --- | --- | ---: | --- | --- |
| 1 | 90 × 25 × 40 mm | 243.0 g | (0.32, +0.24) | Red |
| 2 | 90 × 30 × 40 mm | 291.6 g | (0.32, −0.24) | Green |
| 3 | 90 × 35 × 40 mm | 340.2 g | (0.32, −0.36) | Red |

Use 40 mm thick rectangular aluminium stock, cut to 90 mm length and machined
to the stated widths; density is 2,700 kg/m³. The bars are silver, flat, and
aligned along x. Their bottoms spawn 1 mm above the table, then settle.

The fixed dark steel base is 120 × 190 × 6 mm, centred at (0.225, 0), bottom on
the table. Each gap has two 18 × 24 × 10 mm anvils, with tops 17 mm above the
table. Their bottoms are 1 mm above the base, representing rigid mounting
spacers omitted from collision geometry. GO is 32 mm wide at (0.225, +0.040);
NO-GO is 28 mm at (0.225, −0.040). Acceptable width is 28 < w < 32 mm.

Green and red fixed painted-steel trays are centred at (0.145, +0.25) and
(0.145, −0.25). Each floor is 140 × 220 × 3 mm; walls are 3 mm thick and
10 mm high, from 4 to 14 mm above the table. All wall poses are in geometry.json.
The robot, table and camera stand come from the station framework.

## Grasp, alignment and handoff

The left arm owns +y starts and the green tray; the right owns −y starts and
the red tray. Bar 1 transfers left-to-right at NO-GO, bar 2 right-to-left there;
bar 3 stays on the right. The source releases, retreats, and parks before the
receiver approaches. The receiver pauses 1.5 s at its pick pose before closing.

All URDF collision vertices, scales and origins give TCP-to-lowest-tip distance
**d = 12.842499535 mm**. The grasp TCP is **35.842499535 mm above the bar bottom**:
tips are 23 mm above its bottom, leaving 17 mm of bar side above the tip. The
TCP is 33 mm toward the near x end. Anvil tops remain at table +17 mm.

| Placement | Support above table | Released bottom | TCP above table | Nominal drop |
| --- | ---: | ---: | ---: | ---: |
| Passing GO or NO-GO | 6 mm | 7 mm | 42.8425 mm | 1 mm |
| Blocked GO or NO-GO | 17 mm | 18 mm | 53.8425 mm | 1 mm |
| Tray | 3 mm | 4 mm | 39.8425 mm | 1 mm |

The 25 mm bar passes both gaps; 30 mm passes GO and is blocked at NO-GO; 35 mm
is blocked at GO and skips NO-GO. Release height follows this support outcome.
Table pickup TCP is table +35.8425 mm; lowest tip is table +23 mm.

Every gauge placement first lowers for 1.2 s until the bar bottom is **10 mm
above the anvil tops**, holds **1.5 s** to converge laterally, lowers for another
1.2 s to the release pose, holds **1.5 s**, opens over 0.7 s, retreats over 1 s,
and settles for 1.5 s. Tray placement also pauses between descents and before
release; its intermediate bottom is 10 mm above the tray floor. Every pickup
has an additional 1.5 s alignment hold before closing. Hover TCP is table +110 mm.

At the start of second descent, diagnostic acceptance is:

- Blocked bar: absolute actual centre y error <0.5 mm.
- Passing bar: both side clearances ≥1 mm, using the entire rotated box projection.

These are runtime acceptance checks, not feedback that changes this open-loop
plan. The 30 mm bar in the 32 mm GO gap has exactly 1 mm per side when perfectly
aligned, so that requirement has no spare lateral/yaw tolerance. GPU measurements
must decide whether the holds suffice; heavier bars may need longer holds.

## Scoring, recovery and thresholds

`SCORE_OVER = "end"`. Only `success()` changes history. `score()` and `distance()`
are read-only. Each bar needs GO first, then NO-GO if GO passed. A GO-blocked bar
needs no NO-GO. Seven consecutive 20 Hz samples span the 0.3 s reading dwell.
The final supported, released, still, correct tray placements score 0 / 0.25 /
0.50 / 0.75 for zero / one / two / three inspected bars. The framework awards
1.0 only after all three satisfy success continuously for 1 s. `distance()` is
the number of unfinished placements, not a Euclidean distance.

Support is a contact certificate: reconstruct bottom corners, clip to each
required surface, require contact on every patch, and place the COM inside the
combined support hull. Passing bars contact the base; blocked bars contact both
anvils. Positive air gaps, one-anvil support, held bars and midair wedges fail.
Nearby grippers must open beyond the bar's projected width plus 5 mm. Missing
public gripper readings fail closed; there is no automatic release bypass.

| Quantity | Threshold / meaning |
| --- | --- |
| Gauge centre / long-axis yaw | ≤2 mm / ≤2° modulo 180° for a scored reading |
| Gauge and final tray tilt | ≤0.5° |
| Positive contact roundoff | ≤0.0000005 mm; no physical positive-gap band |
| Solver penetration | ≤0.2 mm |
| Contact patch area | ≥1 mm² on every required support |
| COM / tray boundary margin | ≥1 / ≥1 mm |
| Nearby gripper / release opening | ≤80 mm / projected width +5 mm |
| Rest | Last 10 position samples, all pair distances ≤2 mm |
| Before rest window fills | Speed <10 mm/s and spin <0.5 rad/s |
| Inspection dwell / final success hold | 0.3 / 1.0 s |
| Tilt recovery | >15° clears that bar's checks/dwell; right and re-inspect |
| Irreversible episode failures | Bottom below table−2 mm; speed >0.8 m/s |
| Current work-envelope gate | x 0.03–0.55 m; abs(y) ≤0.57 m |
| Initial position / orientation | ≤4 mm / tilt ≤1°, long-axis yaw ≤2° |
| Fixed structure position / rotation | ≤0.5 mm / matrix deviation ≤0.001 per entry |
| Shape extent / AABB centre consistency | ≤0.5 / ≤0.5 mm |
| CPU IK position / orientation | <0.5 mm / <0.1° |
| CPU robot–fixture / operational tip–anvil gap | ≥3 / ≥3 mm |
| Nominal release drop | 1 mm; every release must be <2 mm |
| Run limit | 6,000 steps at 20 Hz, 300 s |

Tilt, wrong/unchecked tray placement and work-envelope errors are repairable
current-state invalidity: no score while invalid, and affected inspection
records are cleared. Missing/changed objects or moved fixed structures clear
all inspection records. Restore them and repeat inspection. An invalid initial
layout cannot start a valid episode. Only below-table loss and excessive speed
latch irreversible failure. Uprighting alone cannot restore an erased check.

A real trial uses continuous video for release, order and dwell; photos establish
visible support, and a tape/caliper checks dimensions. Reset every named bar
and fixed structure to its specified pose. A new process starts fresh; an
in-process reset requires a >100 mm single-frame jump together with a fully
nominal layout. Slow restoration cannot erase an irreversible failure.

## CPU verification

Run with the installed real yam_arena and the team's designated Python environment:

```bash
PYTHONDONTWRITEBYTECODE=1 "$YAM_PYTHON" tasks/sort-blocks-go-no-go-gauge/audit.py "$HOME/yam-eval/gauge-cpu-preflight.json"
PYTHONDONTWRITEBYTECODE=1 "$YAM_PYTHON" -m pytest -p no:cacheprovider tasks/sort-blocks-go-no-go-gauge/test_task.py -q
```

`audit.py` reads this task's geometry and real ArmPlan, all URDF collision hulls,
origins/scales, station bases, table and conservative camera-stand bounds.
Mounted robot bases are excluded from table contact by design. AABB bounds prune
pairs; converged GJK supplies convex-hull distances and witnesses. No AABB bound
is labelled an exact mesh distance. Grip closure is clamped to held bar width;
this is a kinematic assumption, not simulated contact force.

The full 146.2 s plan has 2,924 intervals / **2,925 frames per arm**.

| CPU result | Left | Right |
| --- | ---: | ---: |
| Worst IK position | 0.082966 mm | 0.083868 mm |
| Worst IK rotation | 0.012798° | 0.012798° |
| Joint limits / 3 mm expanded-fixture checks | Pass | Pass |
| Conservative minimum AABB gap | 16.131600 mm | 16.131864 mm |
| Minimum convex-hull distance | 21.254281 mm | 21.251530 mm |
| Lowest tip above anvil during manipulation | 6.017182 mm | 6.017182 mm |

An explicit ideal-settling replay reaches the framework-style 20-frame success
hold at **step 2,826**, with minimum inter-arm link AABB gap **32.549222 mm**.
Ideal released bars are assigned their analytic support pose: this is not a GPU
score. 6,000 exceeds 2,826 and is below 20×2,826; recheck against the eventual
physical demo's actual completion steps before submission.

The full suite passes; exact count, environment, hashes and negative-test output
are recorded in validation.json. Reverting the actual release gap from 1 to
4 mm makes `test_release_drop_below_two_mm` fail on the first GO release. It was
then restored and the full suite rerun. Tests also cover repair/reinspection,
final tilt, irreversible loss/impact, actual rotated alignment, object native
float types, support counterexamples, public sensor shape contracts and debug.

## TASK_DEBUG and subsequent GPU validation

`TASK_DEBUG=1` prints `task_progress` every step during descending, alignment,
closing, pre-release hold and release, including ±1 s around these phases.
Outside these windows it prints at step 1 and every 20 steps. Windows derive
from cumulative demo durations. Framework success is called after each control
step, so the debug action step is the success-call count; it is not offset by a
reset observation. Phase endpoint targets are labelled `waypoint_target_tcp_m`.

Each record includes block position, quaternion, velocities, bottom corners,
base/anvil/target-support signed gaps, dwell, release/stillness/support checks,
actual TCPs/openings, public gripper/finger link poses, and nearest fixed-Box
convex-hull distance/witnesses. Second-descent entry explicitly prints actual
centre error and both side clearances. Unavailable or unconverged distance data
is identified; it does not become an invented clearance or alter scoring.
Optional finger-link reads and convex distance calculations occur only inside TASK_DEBUG output. Their failures print a diagnostic reason and do not change gripper release sensing, success or score. The new finger-link view path and debug performance require GPU validation.

Read instruction from task.json; create `$HOME/yam-eval` first. Confirm flags in
`inspect-robots run --help` before adding optional seed/no-prompt flags. Use tmux
for long jobs and distinct result paths for each new commit:

```bash
T=sort-blocks-go-no-go-gauge
INS="$(python -c "import json; print(json.load(open('tasks/$T/task.json'))['instruction'])")"
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/$T/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$INS" --scorer task_score \
  --max-steps 200 --store-frames --log-dir "$HOME/yam-eval/$T-smoke" \
  2>&1 | tee "$HOME/yam-eval/$T-smoke.log"
```

For the full demo use `--max-steps 6000` and `$HOME/yam-eval/$T-demo`. With the
new holds a 200-step smoke ends before the first GO second descent (step 260).
A full demo is needed to validate release/alignment. Check result JSON status,
score and termination; CLI exit zero does not establish success. Revised
runtime/GPU time is unknown. Dream-1, media upload and PR await separate approval.

## Evidence and unresolved limits

- Prior 1fe82d8 GPU demo: JSON status success, task score 0.0, 2,041 steps,
  policy_stop, 276.441 s framework runtime. First reported tip was step 220.
  At step 200 the first bar's bottom was 10.328–11.617 mm above the base and
  the TCP y error was +7.355 mm. At 220 its lowest corner dropped 10.330 mm,
  while its highest corner rose 5.773 mm. This supports a drop/edge-collision
  mechanism, but sparse 20-step logs do not prove a whole-bar 11 mm free fall.
- Revised grasp load, friction, torque, lateral convergence, actual corner gaps,
  settling, handoffs and final score remain physically unverified. The actual
  release must have every bottom corner <2 mm above the intended support and
  fingers ≥3 mm clear of fixed structures. CPU targets do not prove this.
- Contact is a conservative rigid-geometry/COM certificate, not a force sensor.
  Tiny positive solver gaps can reject otherwise plausible support. No threshold
  was relaxed to manufacture a score. Chamfers, burrs, compliance, tolerances,
  wear and mounting spacers are omitted; shared friction is rubber-like.
- The scripted demo knows the fixed manufactured widths and chooses support
  heights/destinations accordingly. It demonstrates mechanics, not perception.
  Fixed visible assignments retain a memorised-layout review concern.
- Debug phase windows follow the scripted plan; a different policy can perform
  actions at other times. Object states and gripper readings remain real, but
  phase labels then represent the scripted schedule rather than inferred intent.
- PR #1849's importance passed, while its positive support-gap rule was rejected;
  this independently written task uses contact patches, COM and release checks.
  Its recorded hold had expired in the referenced November 19 snapshot. Private
  live-board changes remain unverified. Related rejected tasks #658 and #48
  show that gauge-task novelty is not guaranteed by geometry changes alone.
- No third-party assets are redistributed. Robot collision files are read from
  the installed framework. There are no media IDs or full-score claims yet.
