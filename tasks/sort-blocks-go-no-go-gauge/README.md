# Sort blocks with a GO / NO-GO gauge — feasibility stopped

**Status: the candidate layout failed feasibility gate 3. No runnable task, scorer,
or full-score demo has been implemented.** The requested stop condition applies.
This is a preflight report, not a submission. Category: Impact on jobs.

本想法源自他队已失效占位的 PR #1849，本预检从头编写。由于可行性门槛未通过，
尚未实现或修复该 PR 最后一次评审指出的支撑评分问题。

The requested provenance sentence's claim that the scoring problem is fixed
cannot truthfully be made at this stop point.

## Feasibility results

Measured on macOS 26.6, Apple Silicon, Python 3.12.14, NumPy 2.3.5, using the real
`yam_arena` package and its URDF collision meshes. No Isaac Sim, GPU, or rigid-body
simulation was used. `preflight.json` records the complete numerical result.

| Gate | Evidence | Result |
| --- | --- | --- |
| Grasp dimensions | Actual `Box` dimensions: block 1: 90 × 25 × 20 mm; block 2: 90 × 30 × 20 mm; block 3: 90 × 35 × 20 mm. Grip closes across width. Widths are all below 85 mm; heights are all at least 15 mm. | Pass |
| Allowed motion primitives | Candidate uses only real `Waypoint(duration, pos, finger, label, yaw)` calls. All waypoint orientations point down, with zero yaw. No tilting, scooping, or flipping. Initial READY-to-down interpolation is the framework's normal startup transition. | Pass |
| Reach and clearance | Both real `ArmPlan` instances sampled with `action7(t)` at 20 Hz, including t = 0 and the final endpoint: 1,921 frames per arm. Real URDF joint limits and collision meshes were checked. | **Fail** |

The candidate has a 96.0 s trajectory (1,920 control intervals). Left-arm worst IK
error is **83.429895 mm / 9.378597 degrees**. At frame 700 (35.0 s), the desired
TCP is `(0.087, -0.290, 0.8675)` m; actual FK gives
`(0.075848, -0.213731, 0.835576)` m. The target is the end of the red-tray retreat,
at the start of the segment labelled `1 park`. It requires the left arm to reach
across to the far side of the red tray. The q values remain within joint limits;
being inside joint limits does not establish target reachability.

There are **212 mesh/fixture/frame pairs** intersecting fixed geometry expanded
by 1 mm on the left arm. These are repeated collision witnesses, not 212 separate
impacts. The first is frame 636 (31.8 s), `1 tray lower`, a left finger pad against
the red tray floor. The list is truncated to the first 30 in the evidence file.
An independent check at frame 650 (32.5 s), `1 tray release`, with **unexpanded**
geometry confirms real intersection with the floor:

| Collision mesh | Lowest vertex below floor top | Exact mesh/box intersection |
| --- | ---: | --- |
| `tip_left_014.stl` | 6.469021 mm | Yes |
| `tip_left_015.stl` | 6.364068 mm | Yes |
| `tip_right_008.stl` | 2.615365 mm | Yes |

These heights describe vertices, not a minimum penetration depth. The same
left-finger geometry also intersects the table in the recorded failing segment.
The right arm's sampled result is 0.082819 mm / 0.012637 degrees worst IK error,
all joints within limits, zero expanded-fixture intersections, and a conservative
AABB clearance lower bound of 5.179388 mm. Both arms' shape declarations bind to
the real constructor signatures. No task-object spawn overlap was found.

## Candidate geometry and audit method

The gauge base is a fixed 120 × 240 × 6 mm slab centred at `(0.24, 0)` m. The GO
and NO-GO gaps are centred at y = +65 and -65 mm, with widths 32 and 28 mm.
Each anvil is 18 × 24 × 10 mm. Anvil tops are 17 mm above the table; base top is
6 mm above it. The anvils have a 1 mm assembly gap above the base in this
candidate; their fastening and that gap were not physically validated.

The green and red tray floors are 140 × 220 × 3 mm, centred at `(0.12, +0.25)`
and `(0.12, -0.25)` m. Their 3 mm thick walls are fixed. Block start centres in
the tabletop plane are `(0.33, +0.19)`, `(0.33, +0.29)`, `(0.33, -0.24)` m.
Blocks spawn 1 mm above the table, with no interpenetration. Mass uses uniform
aluminium density 2,700 kg/m³. The primitive geometry needs no third-party asset.

To keep fingers outside the short anvils, the candidate grasps 33 mm toward the
near end of each block. The TCP is 18 mm above the block bottom. A release target
adds 4 mm to the intended supported height. These are intended geometry values;
the failed IK shows that the left arm does not achieve them at the red tray.

The audit reads every robot URDF collision mesh, applies collision origins,
mesh scales, joint FK, and the station base positions. AABB broad phase is
followed by convex separating-axis checks against the fixed boxes. The table
and camera stand are included; capsules are conservatively bounded as boxes.
The mounted robot base is excluded from table contact checks. During grasping,
finger positions are clamped to the assumed block width rather than the
impossible fully-closed command. This is an explicit kinematic assumption.

## Thresholds — development estimates

All tolerances below are **开发估计 / development estimates**, not GPU-calibrated
acceptance thresholds. No scoring thresholds were implemented.

| Quantity | Candidate value | Physical basis / limitation |
| --- | --- | --- |
| Maximum grasp width | Strictly < 85 mm | Margin below the YAM nominal 94 mm opening |
| Minimum block height | ≥ 15 mm; actual 20 mm | Vertical opposing faces for a downward parallel grasp |
| IK position / rotation error | ≤ 0.5 mm / 0.1 degree | Framework demo-test accuracy; large errors invalidate placement and clearance |
| Fixed-structure clearance | 1 mm expansion on all fixed faces | Detects contact and close approaches in nominal kinematics; tracking error is unmodelled |
| Spawn vertical gap | 1 mm | Avoids initial interpenetration |
| Intended release clearance | 4 mm | Short settling drop within the requested 3–5 mm range |
| Gauge inspection dwell | ≥ 0.3 s | Requirement only; no scorer or dwell test implemented |
| Position rest window | 10 frames / 0.5 s; ≤ 2 mm from every sample | Requirement only; no scorer implemented |
| Full-score hold | 1.0 s | Requirement only; no success implementation |

## Reproduction and tests

The original build/audit script is kept **outside this repository**, in the
requested reference tools directory. It does not contain or import PR #1849's
code. Set `GAUGE_PREFLIGHT_SCRIPT` to that script's absolute path and make the real
`yam_arena` package importable in a Python 3.12 environment with NumPy and pytest.
From this checkout:

```bash
PYTHONDONTWRITEBYTECODE=1 python "$GAUGE_PREFLIGHT_SCRIPT" /tmp/gauge-preflight.json
PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider tasks/sort-blocks-go-no-go-gauge/test_task.py -q
```

The audit writes numerical results even for an infeasible candidate. The tests
verify that this result is **rejected**, using real constructors, real ArmPlan
replay, and real collision geometry. Passing these regression tests does not
mean the task passed feasibility. Final result: **18 passed, 0 failed, 0 skipped**
in 10.63 s, with pytest 9.2.0. No tests were skipped. All four deliberate incorrect
implementations were caught: ignoring IK errors (2 failures), ignoring fixed
clearance (1 failure), returning false for actual finger/fixture intersection
(1 failure), and ignoring grasp dimensions (2 failures). Mutants were separate
external copies; the original audit script was preserved. See `validation.json`
for exact results, script/evidence hashes, and framework revision.

An initial run had 7 passes and 11 fixture errors because the evidence copy was
sandbox-denied and the tests started before the file existed. After copying the
evidence with approved write access, the complete test run passed as above.

No task-scoring tests were run: there is no scorer. The requested tier, event
latch, replay reset, tilt/yaw, held-block, invalid-start, support-hover, debug,
and full-score tests therefore remain unimplemented. There is no `task.py` or
`task.json`; placeholder task metadata would misrepresent the gate outcome.

## GPU handoff — blocked by feasibility

Do not execute a GPU demo for this candidate. There is no runnable task file.
After a replacement layout passes the CPU gate and the task is implemented,
the required command sequence would be:

```bash
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/sort-blocks-go-no-go-gauge/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Gauge the aluminium blocks and separate the acceptable parts into the green tray and the rejects into the red tray. Keep untested parts out of the finished trays." \
  --scorer task_score --max-steps 200 --seed 0 --no-prompt \
  --store-frames --log-dir /tmp/gauge-smoke

# Only after the smoke succeeds; set GAUGE_RUN_LIMIT to the validated full-demo limit.
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/sort-blocks-go-no-go-gauge/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Gauge the aluminium blocks and separate the acceptable parts into the green tray and the rejects into the red tray. Keep untested parts out of the finished trays." \
  --scorer task_score --max-steps "$GAUGE_RUN_LIMIT" --seed 0 --no-prompt \
  --store-frames --log-dir /tmp/gauge-full-demo
```

Smoke acceptance values to instrument in a future implementation: both arms'
gripper readings available on all 200 steps; finite TCP positions and pad gaps
in `[0, 93.9]` mm; open gap at least block width + 5 mm near any credited part;
zero credits when readings are unavailable; no nonzero unsupported check at a
0.3 mm vertical gap; task_progress lines exactly at steps 1, 20, ..., 200; zero
scoring before a valid released, supported dwell. Physics contact and actual
robot tracking must be inspected in logs/video. No such smoke diagnostics exist
yet. A 200-step smoke would end before this candidate's first failure at step
636, so it cannot establish full-demo feasibility. No run_limit is claimed;
1,920 intervals is only this failed nominal trajectory's duration.

The permitted delayed public-API gripper reader is unimplemented and unverified
on GPU. If it cannot read both arms, checks must fail closed with a debug reason.
The requested object-state-plus-video fallback would require explicit external
human judgment of release; it cannot be presented as an automatically verified
1.0. In real trials, video must show release and a photo must show no visible
gap between the block bottom and its gauge support. These remain requirements,
not verified behavior of this report.

## Closest prior PRs and review risk

Source: `pr-crawl` snapshot `snapshot-20261119T072321Z`, completed
2026-11-19 07:34:28 UTC. Only PR #1849's task.json and README were fetched and
read; its implementation code was not read or copied.

| PR | Snapshot finding | Difference / risk for this candidate |
| --- | --- | --- |
| [#1849](https://github.com/robocurve/benchmark-tasks/pull/1849) | Same GO / NO-GO block-sorting idea. Latest recorded rejection: support cannot be established by the documented 0.5 mm proximity test. | Candidate has three 90 mm long blocks and grasps across width at an overhanging end; prior documentation uses seven 50 mm long blocks gripped along length. The required support/release fix is unimplemented. There is no demonstrated scoring improvement. |
| [#658](https://github.com/robocurve/benchmark-tasks/pull/658) | Two-limit lot inspection, rejected as a duplicate; fixed distinguishable layouts and a demo using hidden block kinds were also criticised. | Candidate omits the lot-release decision. Its preflight uses known widths to choose target support heights and routes; this is unsuitable evidence of a perception-based demo. Randomized assignments and observable decisions remain unresolved. |
| [#48](https://github.com/robocurve/benchmark-tasks/pull/48) | Pin-and-gauge-plate fit inspection, finally rejected as duplicating an earlier inspection capability. | Candidate uses rectangular widths and two gaps instead of cylindrical pins and pockets. That alone does not establish a distinct core capability. |

The snapshot's last #1849 review is 2026-11-17 22:32:26 UTC, so its documented
48-hour hold expired at 2026-11-19 22:32:26 UTC if no revision arrived. A fresh
fetch on 2026-11-20 still returned head `8369fe24691b5693350900b66e8d5f0dd35fdf07`,
committed 2026-11-17 19:53:28 UTC, matching the snapshot. The live task board
redirected to sign-in; current private board ownership was not verified.

Likely rejection reasons, if submitted now: failed physical feasibility; no
full-score demo or calibrated contact/release scorer; possible duplication;
fixed, recognizable width assignments; preflight routing based on hidden widths;
no GPU results or required media. No PR, media upload, or Dream-1 run was made.

## Remaining failures, assumptions, and operational limits

- Gate 3 fails for the tested candidate. No repair was attempted after this
  failure, following the explicit stop instruction.
- Collision tests use nominal kinematics, the real decomposed URDF meshes, and
  assumed width-limited finger positions. They do not simulate finger friction,
  actuator compliance, off-centre grasp torque, block settling, or tracking error.
- Inter-arm collisions, contact forces, achieved grasp alignment, upper-reach
  safety margin, release heights under physics, and the final score remain
  unverified. The right arm's isolated pass is not a whole-task pass.
- A 1 mm assembly gap below fixed anvils, coarse 28/32 mm limits, and one shared
  material/friction model have not been validated as a faithful physical gauge.
- No actual-support proof exists. Tightening a pose tolerance alone must not be
  called a verified support fix. The restricted ObjectState interface and allowed
  gripper reads have not yet been reconciled with the contact requirement.
- The system Python was 3.9 without NumPy, pytest, or yam_arena. Tests instead used
  bundled Python 3.12.14, NumPy 2.3.5, pytest 9.2.0, the real local framework, and
  an existing external pytest dependency directory. No package was installed.
- Initial sandbox Git-fetch and DNS attempts were blocked; approved retries
  succeeded. A broad initial file search encountered protected macOS directories;
  framework discovery was completed with targeted project paths. No credential,
  environment secret, or authentication file was opened.
- No assets directory or third-party model/texture was created. The asset metadata
  and image-stripping checks are inapplicable. Robot mesh use is through the
  existing framework installation; no mesh is redistributed.
- All repository changes are confined to this task folder. The external audit
  script and local Git exclude entries are the explicitly requested exceptions.

Nothing is consumed. For a future trial, the blocks must be reset to a validated
starting layout and the inspection history reset only by the specified episode
jump mechanism. That reset mechanism is not implemented here.
