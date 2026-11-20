# Sort aluminium parts with a GO / NO-GO gap gauge

The single authorised layout repair passes CPU feasibility. The task, end-state
scorer, gripper reader, and nominal demo are implemented. **GPU physics and
full-score execution remain pending.** `scores` is `pending` and `media` is `{}`.

本想法源自他队已失效占位的 PR #1849，本实现从头编写，并修复了该 PR 最后一次评审指出的问题。

That correction is verified on adversarial ObjectState sequences: unsupported
positive-gap poses, held parts, and tilted midair poses cannot earn inspection
credit. It is not a claim that the new scorer already reached 1.0 in GPU physics.
No prior task implementation was read or copied.

## Instruction and physical setup

“Gauge every aluminium block before putting acceptable parts in the green tray
and rejects in the red tray. Accept a part only if it enters the GO gap and stays
out of the NO-GO gap. Keep unchecked parts out of the finished trays, and leave
all parts settled and released.”

The task uses `SCENE = "station"`, with `TABLE_Z = TABLE_TOP[SCENE]`, 0.7475 m.
World x points forward, y left, z up. `geometry.json` is the authoritative numeric
setup. Every shape is a real Box primitive, not a substitute for an appliance.

| Movable aluminium bar | Length × width × height | Uniform-density mass | Start xy, m | Correct tray |
| --- | --- | --- | --- | --- |
| Block 1 | 90 × 25 × 25 mm | 151.875 g | (0.32, +0.24) | Red |
| Block 2 | 90 × 30 × 25 mm | 182.250 g | (0.32, −0.24) | Green |
| Block 3 | 90 × 35 × 25 mm | 212.625 g | (0.32, −0.36) | Red |

Bars are silver, flat, and long edges along x. Density is 2,700 kg/m³. The bars
use common 25 mm high rectangular aluminium stock, machined to width. Their
bottoms spawn 1 mm above the table and settle. Names identify reproducible parts;
there are no visual labels or third-party models.

Fixed dark steel gauge: 120 × 190 × 6 mm base centred at (0.225, 0), bottom on
the table. Each gap has two 18 × 24 × 10 mm anvils. GO width 32 mm at y = +40 mm;
NO-GO width 28 mm at y = −40 mm. Their tops are 17 mm above the table. The fixed
anvils have a 1 mm assembly gap above the base, representing rigid mount spacers
not included in collision geometry. Gauge gaps run along x. Acceptable width is
28 < w < 32 mm; exact equality would be rejected, although no bar is at a limit.

Green and red fixed painted-steel trays are centred at (0.145, +0.25) and
(0.145, −0.25). Each has a 140 × 220 × 3 mm floor. Walls are 3 mm thick, 10 mm
high, from 4 to 14 mm above the table; their horizontal centres and lengths are
in `geometry.json`. Robot/table/camera geometry comes from the station framework.
Nothing is consumed; no object jitter is permitted.

## Arm ownership, grasp and handoff

The left arm handles +y starting parts and the green tray; the right handles −y
starting parts and the red tray. Both can reach the shared gauge. Block 1 goes
left-to-right at the NO-GO gap; block 2 goes right-to-left there. The source arm
releases, rises, and parks before the receiver approaches. Block 3 stays on the
right. No arm reaches over the midline to the opposite tray. This starting
layout deliberately cannot be sorted correctly by keeping every part on its
starting side.

URDF measurement uses **all collision-mesh vertices** of both fingers, including
mesh scales, collision origins and joint transforms, in open, closed and
width-limited states. TCP to lowest fingertip is **12.842499535 mm**. The TCP
stands **17.842499535 mm above the bar bottom**, leaving a nominal 5 mm tip
clearance and 20 mm of vertical bar face above the tip. Width, 25–35 mm, is the
closing direction. The grasp is 33 mm toward the near x end so the short anvils
do not obstruct the finger approach. Block height is increased from 20 to 25 mm.

Release TCP = intended support top + measured grasp offset + 4 mm. The bar's
bottom therefore has a 4 mm settling drop. Hover is 110 mm above the table.
The 99 s plan uses only downward Waypoints, translation, yaw=0, opening and
closing. READY-to-down startup interpolation is the framework's normal motion.

## Scoring and actual geometric support

`SCORE_OVER = "end"`. `success()` alone advances history. `score()` and
`distance()` are read-only, including arrays and debug counters. The distance
hook reports the count of unfinished valid placements, not a Euclidean metre
measurement; use `--scorer task_score`.

A valid check requires release, stillness, and a contact certificate on every
step of a 0.3 s dwell. Seven 20 Hz samples span six intervals = 0.3 s. Every bar
needs GO first; a GO passer then needs NO-GO. A blocked GO bar needs no NO-GO.
First valid readings latch. Checks alone do not earn a placement tier.

**Support is not a positive proximity tolerance.** The scorer reconstructs the
real bottom polygon from position, quaternion and the known box dimensions.
It clips that polygon against the designated horizontal support rectangles.
A seated bar must fit inside the proper gap and contact the base. A blocked bar
must have a nonzero bottom contact patch on **both** anvils. All vertices of
each required contact patch must have nonpositive signed separation, allowing
only 0.5 nanometres for floating-point roundoff. Penetration up to 0.2 mm is
accepted for the solver; a positive air gap is not. The uniform bar's centre of
mass projection must lie at least 1 mm inside the combined convex support hull.
Wrong-lane, one-anvil, rotated-across-the-gap, and midair tilted readings fail.
This geometric certificate establishes no-gap support under the rigid-box model;
it does not measure load-bearing contact force.

Both grippers are read through a **function-local delayed import** of the public
`isaaclab.sim.SimulationContext`, its `physics_sim_view`, and articulation views.
Only joint positions and gripper transforms are read. No embodiment method is
wrapped or patched; no private yam_arena attribute is read. Every gripper within
80 mm of the bar AABB must have pad gap ≥ projected gripped width + 5 mm. A
missing/invalid reader gives no inspection or placement credit and a debug reason.

Rest uses ten positions spanning 0.45 s at 20 Hz (a conventional 0.5 s sample
window); all pair distances must be ≤ 2 mm. The current queried position is
also checked. Before ten samples exist, explicit speed < 10 mm/s and spin <
0.5 rad/s are used. Once full, position history overrides stale sleeping velocities.
The separate seven-sample released/support dwell must also hold.

At the final frame, a credited bar needs all required readings, release, rest,
its entire footprint ≥ 1 mm inside the correct tray, and floor contact by the
same certificate. Rim placements, unsupported stacks and held bars do not count.

| Highest satisfied final tier | Score |
| --- | ---: |
| No correctly placed, inspected bar | 0 |
| One | 0.25 |
| Two | 0.50 |
| All three | 0.75 |
| All three continuously successful for 1.0 s, awarded by framework | 1.0 |

Tiers never add. Tipping >15°, an impact speed >0.8 m/s, leaving the working
bounds, changed/missing geometry, moving a fixed structure, and a released,
settled wrong or unchecked tray placement invalidate the whole episode. The
failure remains after repair. Final geometry and placement are rechecked.

An in-process reset requires a >0.10 m **single-step** position jump of a bar
and the entire scene back at nominal poses. Slowly returning everything cannot
clear history. A true reset clears checks, dwell and position windows, and
restarts debug cadence at step 1. A wrong initial layout fails closed. Starting
a new task process also starts a new episode.

**Real trials:** use continuous video to judge gripper release, order and dwell.
Use a side/top photo per reading to confirm there is **no visible gap** between
the bar bottom and the base or both anvil tops. A tape/caliper verifies fixture
sizes and final tray margins. A held bar, a 0.3 mm hover, and an unsupported
slanted pose are invalid. Record the final result only after the arms release.

## Thresholds — 开发估计 / development estimates

These are engineering estimates, not GPU-calibrated measurements.

| Quantity | Value | Physical basis |
| --- | --- | --- |
| Grasp width / face height | <85 mm / ≥15 mm; actual 25–35 / 25 mm | Margin below 94 mm nominal jaw opening; usable opposing faces |
| Tip clearance above bar bottom | 5 mm nominal, ≥3 mm required | Measured d plus tracking margin; no floor/anvil scraping |
| Release clearance | 4 mm | Short settling drop, within requested 3–5 mm |
| Robot/finger–fixture clearance | ≥1 mm | Expanded real collision geometry, all interpolated frames |
| IK errors | <0.5 mm / <0.1° | Precise placement and conservative clearance validity |
| GO / NO-GO | 32 / 28 mm | Coarse physical discrimination resolvable by rigid contacts |
| Gauge centre / long-axis yaw | ≤2 mm / ≤2° modulo 180° | Correct gauge lane and width presentation |
| Gauge/tray upright test | ≤0.5° | Reject tilted false readings; contact patches further constrain tilt |
| Positive contact roundoff | ≤0.0000005 mm | Floating-point arithmetic only, never a physical support band |
| Maximum penetration | 0.2 mm | Solver interpenetration allowance, one-sided toward the solid |
| Contact patch area | ≥1 mm² per support | Avoid degenerate point/line support |
| COM / tray boundary margin | ≥1 / ≥1 mm | Stable support hull and full placement inside walls |
| Gripper proximity / gap margin | ≤80 mm / ≥5 mm extra opening | Includes offset grasp and nearby fingers; release must be observable |
| Rest | 10 samples; pairwise ≤2 mm | Avoid frozen velocity and residual solver noise |
| Initial fallback speed / spin | <10 mm/s / <0.5 rad/s | Explicit thresholds before position window fills |
| Inspection dwell / success hold | ≥0.3 / 1.0 s | Inspect after release; maintain final outcome |
| Tip / impact latch | >15° / >0.8 m/s | Knocked-over or flung parts cannot recover credit |
| Work bounds | x 0.03–0.55 m; abs(y) ≤0.57 m; bottom ≥table−2 mm | Detect ejection/off-table outcomes |
| Initial position / fixed position tolerance | ≤4 / ≤0.5 mm | Reproducible layout and fixed gauge geometry |
| Initial upright / x-axis alignment | ≤1° / ≤2° modulo 180° | Flat stock at known pick poses |
| Fixed orientation matrix deviation | ≤0.001 per entry | Fixed structure should not move or rotate |
| Shape extent / AABB-centre tolerance | 0.5 / 0.5 mm | Reject changed geometry or inconsistent states |
| Reset jump | >100 mm in one sample plus full nominal layout | Episode reset distinct from slow restoration |
| Run limit | 2,600 steps / 130 s | 1.31 times nominal 1,980-step path, not an excessive limit |

## CPU validation and reproduction

Environment: macOS 26.6 arm64; Python 3.12.14, NumPy 2.3.5, pytest 9.2.0; real
`yam_arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`.
No Isaac Sim, GPU, package installation, or replacement framework was used.
The external original audit is `gauge_preflight_round1.py`, stored in the
requested reference tools directory. Set `GAUGE_PREFLIGHT_SCRIPT` to its path
only to regenerate the external audit or test its deliberate mutants. The
committed tests also run without that variable: they use the real package,
committed geometry, and test-only URDF collision readers. With the real package
and pytest importable, from the checkout root:

```bash
PYTHONDONTWRITEBYTECODE=1 python "$GAUGE_PREFLIGHT_SCRIPT" /tmp/gauge-preflight.json
PYTHONDONTWRITEBYTECODE=1 python -m pytest -p no:cacheprovider tasks/sort-blocks-go-no-go-gauge/test_task.py -q
```

Each real ArmPlan was sampled at 20 Hz with `action7(t)`: 1,981 frames per arm,
including endpoints. Every declared shape/waypoint binds to its real signature.
Initial geometry has no overlap. The complete task demo matches the audited
plan exactly; physical trajectories were not silently changed after the gate.

| Result | Left | Right |
| --- | ---: | ---: |
| Worst IK position | 0.092057 mm | 0.091828 mm |
| Worst IK rotation | 0.012565° | 0.013348° |
| All joint limits | Pass | Pass |
| Fixed structures expanded by 1 mm: intersections | 0 | 0 |
| Conservative robot/finger–fixture AABB gap | 5.016082 mm | 5.013383 mm |

Real collision meshes, collision origins/scales, station base positions, table
and camera stand are checked. The mounted bases are excluded from table contact
by design. Camera stand capsules have conservative box bounds. Finger closure
is clamped at the held bar's width; this is kinematics, not a force simulation.
Inter-arm whole-link AABB clearance in the complete replay is at least
32.529763 mm. This bound includes the fingers and gripper bodies.

Final standalone suite: **84 passed, 0 failed, 0 skipped**, in 26.09 s, including
every interpolated frame's robot/finger–fixture clearance. The original 18 feasibility
regressions are retained with the repaired dimensions and expected passing
outcome. Added tests cover all tiers; contact support/COM; 0.3 mm hover; clamped
bars at gap height, either arm; midair slants; threshold boundaries; yaw 180°;
irreversible events after slow repair; invalid starts; direct terminal jumps;
held final placements; real reset; read-only hooks; debug; missing/changed shapes;
frozen velocities; public reader shape contracts; and full plan logic.

The full demo is also replayed through real IK at 20 Hz with an **explicit ideal
settling model**: a released bar is assigned its analytic support pose. In this
model, framework-style 20-step success hold completes at step **1,905**. This
checks sequencing, release, handoffs, scoring and runtime; it does not establish
that frictional grasps and drops work in PhysX.

Four audit mutants and ten task mutants were tested in isolated external copies;
all 14 caused test failures. They deliberately: ignore IK, ignore fixture
clearance, misreport finger intersection, ignore grasp dimensions, accept a
positive 0.5 mm support gap, ignore release, omit COM support, erase failure
latches, allow slow reset, mutate score state, add score tiers, use velocity-only
rest, print every debug step, and treat yaw as tilt. Exact counts, hashes and
validation provenance are in `validation.json`.

## GPU handoff: 200-step smoke, then full demo

Run on the configured GPU environment with real Isaac Lab/Arena and yam_arena.
From this checkout, first:

```bash
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/sort-blocks-go-no-go-gauge/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Gauge every aluminium block before putting acceptable parts in the green tray and rejects in the red tray. Accept a part only if it enters the GO gap and stays out of the NO-GO gap. Keep unchecked parts out of the finished trays, and leave all parts settled and released." \
  --scorer task_score --max-steps 200 --seed 0 --no-prompt --store-frames \
  --log-dir /tmp/gauge-smoke
```

Expect task_progress exactly at steps **1,20,40,...,200**. `gripper` must read
`ok`; both `pad_gaps_mm` must be finite and within 0–94.1 mm (nominal maximum
93.9 mm, 0.2 mm sensor allowance). At an open pose expect about **93.9 mm**; holding block
1 expect about **25 mm**. A nearby released block 1 requires ≥**30 mm**, block 2
≥**35 mm**, block 3 ≥**40 mm**. `failed` must be empty. At step 200 the first
GO check is not yet complete, so `credited=0` is expected; zero smoke score is
not a failed full demo. Confirm fingers stay ≥3 mm above the block bottom and
≥1 mm clear of fixed structures on the recorded views.

`tcp_m` reports both TCPs. `contact` reports each bar's minimum/maximum bottom
separation from base and anvil top, in mm, plus its instantaneous gauge readings.
For a credited supported pose, the relevant signed gap range must be inside
**[−0.2, +0.0000005] mm**. A +0.3 mm range must yield no reading. These raw ranges
are diagnostic; footprint clipping and COM checks are still required by scoring.

The exact public-reader path is GPU-unverified. If it reports unavailable, stop
automatic scoring and retain fail-closed zero credit. Use the allowed fallback:
object-state geometry plus **human video judgment of release** to evaluate the
trial, and record that as external manual judgment. Do not claim automatic 1.0
from poses alone. Implementing an automatic bypass is deliberately avoided.

Only after successful smoke:

```bash
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/sort-blocks-go-no-go-gauge/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Gauge every aluminium block before putting acceptable parts in the green tray and rejects in the red tray. Accept a part only if it enters the GO gap and stays out of the NO-GO gap. Keep unchecked parts out of the finished trays, and leave all parts settled and released." \
  --scorer task_score --max-steps 2600 --seed 0 --no-prompt --store-frames \
  --log-dir /tmp/gauge-full-demo
```

The final result must be 1.0 from continuous success within 2,600 steps, with no
failure latch. Inspect gauge photos: no positive gaps; both anvils must support
a blocked bar. A rigid-body positive gap, even 0.01 mm, intentionally fails.
Do not weaken that rule to obtain a score. This strictness may expose numeric
separation or imperfect seating that requires a new approved physical revision.
No GPU command, Dream-1 run, media upload or PR was executed in this work.

## Prior work, limitations and complete failure record

Snapshot source: `pr-crawl/snapshot-20261119T072321Z`, completed November 19 at
07:34:28 UTC. PR #1849 task.json and README only were read. Its last recorded
review was November 17 at 22:32:26 UTC; the head fetched on November 20 remained
`8369fe24691b5693350900b66e8d5f0dd35fdf07`, committed before that review. Its
48-hour hold had expired under the stated rule. The live task board required
sign-in; later private board changes were not verified.

| Closest PR | Distinction and relevant history |
| --- | --- |
| [#1849](https://github.com/robocurve/benchmark-tasks/pull/1849) | Same authorised block-inspection idea; importance passed. Its documented 0.5 mm support band was repeatedly rejected. This implementation uses geometric contact patches on every required support, COM containment, explicit release and adversarial tests. Three 90 mm bars, side-owned trays and two gauge handoffs replace its documented seven 50 mm bars. Physical full-score verification remains pending here. |
| [#658](https://github.com/robocurve/benchmark-tasks/pull/658) | Two-limit lot release/hold task rejected for repeating earlier inspection capability, fixed distinguishable layouts, and hidden-kind demo decisions. This task follows #1849's separately accepted importance framing and requires each individual bar's real reading before any placement credit. It has no lot decision. Known-layout and scripted-routing limitations still apply. |
| [#48](https://github.com/robocurve/benchmark-tasks/pull/48) | Pin-and-gauge-plate task rejected for repeating earlier inspection capability. This task is the #1849 rectangular-part two-gap inspection idea, whose importance was accepted. That history reduces the duplicate concern; it does not guarantee acceptance of this implementation. |

All remaining assumptions and problems:

- The first layout failed: left arm crossed to the red tray, with 83.429895 mm /
  9.378597° worst IK error, 212 expanded-mesh/fixture/frame collision witnesses,
  and confirmed unexpanded finger/floor intersection. This single repair addresses
  ownership and measured fingertip clearance; there was no second geometry repair.
- GPU release reads, real frictional grasps, off-centre grasp torque, dropped-bar
  centring, solver penetration sign, contact-force magnitude, and final physics
  success are unverified. CPU rest poses are synthetic, explicitly so.
- Support is a conservative rigid-geometry/COM certificate. It can reject real
  support when PhysX reports a small positive gap or an edge-supported tilted
  rest. Tilted/wedged gauge readings and stacked tray placements are not credited.
- The static demo uses known manufactured widths for release heights and routing.
  It is an open-loop mechanical demonstration, not a perception-driven policy.
  The scorer still requires actual inspections. Starts are fixed and visibly
  distinguishable; no randomized width assignment has been implemented. A reviewer
  could raise the memorized-layout concern that appeared in #1849's first review.
- Aluminium blocks are exact boxes; gauge chamfers, burrs, wear, compliance, fine
  manufacturing tolerances and mounting spacers are omitted. Shared framework
  friction is rubber-like, not calibrated aluminium/steel friction.
- Rest and dwell assume exactly 20 Hz; using the framework's default 10 Hz is
  unsupported. Object jitter must be zero. No run-to-run robustness claim is made.
- The initial report's first test attempt had 11 fixture errors because a sandbox
  denied the evidence copy. After approved copying it had 18 passes. The current
  repaired suite has no failures or skips. Existing system Python 3.9 lacked
  dependencies; bundled Python 3.12 and external existing pytest were used.
- Read-only public physics views are tested with an API-shaped fake and with no
  simulator, not with a GPU. API/version/path differences fail closed. No private
  API or hidden embodiment access was introduced to hide that limitation.
- No third-party model, texture or image is redistributed. There is no assets/
  directory, so image metadata stripping, licensing and asset path scans are
  inapplicable. All geometry is original primitive design; the robot's existing
  framework meshes are read for tests only, not copied into the task.
- Initial Git/DNS sandbox restrictions in the first session were overcome by
  approved retries. No secrets or authentication files were opened. Changes stay
  within this task directory; the requested external audit and Git excludes are
  the only ancillary writes.
- In this round a read-only process-list diagnostic was sandbox-denied; ordinary
  session polling still established test completion. An empty patch request was
  rejected without changing any file. No correct-code assertion failed in this
  repair round. A full collision replay passed in 76.93 s; adding per-hull AABB
  pruning preserved the checks and reduced the final suite to 26.09 s.

Likely reasons for rejection now: no GPU full-score evidence/media; conservative
support rule may not match solver contacts; unverified gripper reader; fixed
recognizable assignments and known-width demo; coarse geometry/friction. PR
#1849's accepted importance is the relevant precedent, rather than a claim that
#48/#658 already disqualify this idea.
