# Seat a gift-box lid squarely

Put a slip-over lid fully onto a packed gift box without wedging its skirt,
moving the base, or removing the gift. Category: Societal relatability. The
right YAM arm performs the nominal demo and the left parks; this task is not
claimed to require bimanual manipulation.

The idea comes from PR #2264, whose hold lapsed without revision; this implementation was written from scratch.

At commit `253f934`, the GPU scripted demo scored **1.0 in 533 steps** and
Dream-1 scored **0.0 after 1200 steps**. Uploaded media is **M-B336HN**.
Only seed 0 with object_jitter=0 has been evaluated; these results do not
establish robustness to other seeds, layouts or physical manufacturing error.

## Reproducible setup

Original printable PLA design files supply all dimensions; no physical sample
has been measured. Coordinates are metres, x forward, y left, z above the
station table. Both parts are movable, long axes along x, yaw zero.

| Quantity | Base | Lid |
| --- | --- | --- |
| Outside L × W × H | 100 × 66 × 40 mm | 112 × 78 × 28 mm |
| Cavity L × W × depth | 96 × 62 × 38 mm | 104 × 70 × 26 mm |
| Floor/top thickness | 2 mm | 2 mm |
| Four wall thicknesses | 2 mm | 4 mm |
| Solid volume | 37,824 mm³ | 55,328 mm³ |
| Mass, PLA density 1,240 kg/m³ | 46.90176 g | 68.60672 g |
| Start xy | (0.24, −0.10) | (0.20, −0.26) |
| Colour / pose | Burgundy, opening up | Matching burgundy, top up and skirt down |

Centred side clearance stays **2 mm** in x and y. The lid walls were thickened
outward, preserving the cavity and 26 mm skirt. Lid panel is 112×78×2 mm at
z=27 mm; long walls 112×4×26 mm at y=±37, z=13 mm; end walls 4×70×26 mm at
x=±54, z=13 mm. Metadata records all panel dimensions, COM, mass and hashes.
Each USD uses one rigid body and five independent Box colliders; `convex=False`
preserves the cavity. There is no whole-lid convex hull or mesh decomposition.

A plain oak gift, 60×38×20 mm and 31.92 g at 700 kg/m³, lies flat at the base's
XY centre. Base and lid spawn 1 mm above the table; the gift is 1 mm above the
initial base floor (2 mm total nominal travel to the settled floor). There is
no initial volume overlap. No object jitter is used. Table height is read from
`TABLE_TOP['station']` (0.7475 m in the tested framework).

Print PLA at 100% infill, 0.2 mm layers and a 0.4 mm nozzle; base floor down and
lid top down, then invert the lid. Inspect actual outside base, inside lid,
heights and all side clearances with calipers/feeler gauges before a physical
trial. A ±0.2 mm linear tolerance gives 1.8–2.2 mm centred clearance; warping,
corner buildup and roughness are unmodelled. No cardboard equivalence is claimed.

`build_assets.py` regenerates the original watertight union STLs and five-panel
USDA files. Lid exterior triangles use centroid refinement, so base.stl is
4,684 bytes and lid.stl is 13,884 bytes, with distinct hashes. This changes the
STL tessellation only; USD collision shapes remain the same five boxes.

The base/lid designs, STL meshes and USD/USDA representations were authored
from scratch for this task and released under **CC0 1.0 Universal**, as stated
in [assets/LICENSE.txt](assets/LICENSE.txt). No third-party mesh or texture is
included. Robot meshes remain external framework assets and are not
redistributed here. Dimensions are design-file values, not physical measurements.

## Continuous grip interface and revised motion

Real framework source declares:

```python
finger: float  # finger joint target (m): FINGER_OPEN .. FINGER_CLOSED
FINGER_OPEN = -0.04695
FINGER_CLOSED = 0.0
```

ArmPlan interpolates `prev_finger + s * (w.finger - prev_finger)`, then computes
`clip(finger / FINGER_OPEN, 0, 1)`. YamGripperAction multiplies that normalized
value by FINGER_OPEN and sends it to both prismatic joints (joint7/joint8).
Thus finger is a **single-finger linear displacement in metres**, not an angle
or the total gap. A 77.4 mm symmetric gap uses **−0.0387 m**, normalized
**0.82428115**. There is no Boolean open/close conversion.

The grasp stays symmetric at the centres of the long walls; no corner grasp.
TCP is 19 mm above the lid bottom. URDF d=12.842499535 mm leaves the lowest tip
6.157500465 mm above the table. The 78 mm lid fits under the nominal 93.9 mm
maximum opening. The commanded pad section is about 0.269 mm into each outer
wall and still 3.731 mm outside the inner cavity. This is a command-geometry
check, not a measured penetration/force. A 2 mm old wall fails this check.

| Right-arm phase | Duration | Control-step boundary |
| --- | ---: | ---: |
| Park | 2 s | 0–40 |
| Approach / descend | 1 / 1 s | 40–60 / 60–80 |
| Pick alignment hold | 1.5 s | 80–110 |
| Close to 77.4 mm | 2 s | 110–150 |
| Closed hold | 1.5 s | 150–180 |
| Lift | 2 s | 180–220 |
| Carry | 2.5 s | 220–270 |
| First lowering | 1.5 s | 270–300 |
| Place alignment hold | 1.5 s | 300–330 |
| Second lowering | 3 s | 330–390 |
| Pre-release hold | 1.5 s | 390–420 |
| Open | 2 s | 420–460 |
| Open hold | 1.5 s | 460–490 |
| Retreat / park / final hold | 2 / 2 / 2 s | 490–610 |

The shorter empty approach replaces the old redundant explicit settle wait;
the framework already settles the reset scene. A 200-step smoke covers complete
closing/closed hold and the first half of lifting, not all lifting or seating.

Before final lowering, lid bottom is 10 mm above the 40 mm box rim: TCP is
table +69 mm (world z=0.8165 m). After holding at least 1.5 s, diagnostic
acceptance requires |dx|, |dy|≤0.5 mm, relative yaw/tilt≤0.5°, and **all four
actual projected side clearances ≥1 mm**. Projection uses all rotated base
corners in the lid frame. The open-loop plan does not adapt to a failed check;
a failed check means the physical trial needs diagnosis, not automatic approval.

Final nominal lid bottom is table +15 mm, 1 mm above its seated bottom at
14 mm. Release TCP is table +34 mm (world z=0.7815 m). Hold 1.5 s, open over
2 s, wait open 1.5 s, then retreat. Fully seated lid top is table +42 mm.
Inherited contact offset remains 5 mm, rest offset zero; static/dynamic
friction 1.2/1.0. A 5 mm speculative contact offset is not a 5 mm physical gap.
The fd23a947 demo seated without persistent jamming through the 2 mm side
clearance; contact forces and physical calibration remain unverified.

## Scoring

Task instruction: “Put the lid fully onto the gift box, level and square. Keep
the gift inside and leave the box where it is. Let go when it is seated.”

### Real-trial measurements

Use a millimetre ruler, a straightedge and level side/top photos. Compare the
trial video with a marked initial base outline to confirm release, no dragging,
no base repositioning and the gift staying inside. Base movement over 20 mm is
a failure, not permission to deliberately move it. The base must remain upright
and resting on the table; the shared exclusions in the implementation below
still apply. Award only the highest satisfied final-state tier; other outcomes
score zero.

Let Hr be the current base-rim height above the table, Ht the mean height of
the four lid-top corners, C the horizontal centre distance measured to ±1 mm,
Dh the maximum minus minimum corner height, and G the two-end gap-skew
difference defined below. All dimensions in the following table are millimetres.

| Award | Ruler/photo conditions, after release and settling |
| --- | --- |
| 1.0 | Ht=Hr+2±1.5; C≤2; Dh≤2; G≤2.9; four visible, nonzero side gaps; held for 1 s |
| 0.75 | Ht=Hr+2±1.5; C≤2; Dh≤4; G≤5.8; four visible, nonzero side gaps; full award not met |
| 0.50 | Upright, partly engaged lid; Hr+0.5≤Ht≤Hr+18; C≤8; no higher award |
| 0.25 | Upright lid resting at the rim; Ht=Hr+28±2; C≤25; no higher award |

The lid is 112×78×28 mm, with a 26 mm skirt and 2 mm top panel; the base is
100×66×40 mm. For a level lid, Ht−Hr=28−engagement. Therefore the 0.50
engagement range [10,26+1.5] becomes Ht−Hr in [0.5,18]; rim rest becomes
28±2; full seating becomes 28−26=2 mm, with the existing ±1.5 mm allowance.

Over the 112 mm long edge, 112 sin(1°)=1.955 mm and 112 sin(2°)=3.909 mm,
giving the approximately 1°/2° ruler criteria Dh≤2/4 mm. The actual four-corner
range is **Dh=112|R31|+78|R32| mm**, for the lid-to-table rotation matrix R.
Its worst direction at tilt theta is sqrt(112²+78²) sin(theta), or
2.382/4.763 mm at 1°/2°. The four-corner limits are observable physical
criteria, not exact replacements for all 3-D orientations. The simulation
continues to use its unchanged normal-angle thresholds.

For yaw, measure the positive/negative long-side gaps g+/g− at both ends of
a **100 mm baseline along the base**. Extend the straight edge lines with a
ruler if needed. Set Dnear=(g+−g−)near and Dfar=(g+−g−)far. Translation cancels
in the two-end difference; normalize to the 112 mm lid length:

```
G = 112/(2×100) × |Dfar−Dnear| = 112 tan(|yaw|) mm
112 tan(1.5°) = 2.933 mm  -> conservative G ≤ 2.9 mm
112 tan(3°)   = 5.870 mm  -> conservative G ≤ 5.8 mm
```

A single opposed-gap difference at one cross-section also changes when the
lid translates; it must not be mistaken for this yaw measurement. The actual
100 mm base length, rather than an imaginary 112 mm overlap, sets the measuring
baseline. All four side gaps must remain visible and nonzero when seated.

A ruler or photo cannot reliably distinguish sub-millimetre centring. With
2 mm design clearance on each side, the seated skirt already constrains
offset to about 2 mm along each axis. The radial C≤2 mm condition is conservative
for diagonal offsets and confirms that the lid surrounds the base; it is not
a sub-millimetre centring requirement.

### Simulation implementation

All simulation thresholds and the demo trajectory are unchanged. Credit needs
valid initial/state data, no latched violation, still objects, readable gripper
states and both complete grippers at least 30 mm from every object. Unavailable
scoring reads fail closed. Both hands must be clear, but the task does not
require both arms to manipulate objects.

| Tier / rule | Unchanged numeric condition |
| --- | --- |
| 0.25 | No higher award; skirt bottom within 2 mm of rim; centre≤25 mm; tilt≤5° |
| 0.50 | No higher award; engagement≥10 mm, no deeper than 26+1.5 mm; centre≤8 mm; tilt≤10° |
| 0.75 | Full success not awarded; top within 1.5 mm of rim+2 mm; centre≤2 mm; yaw≤3°; tilt≤2° |
| 1.0 | Seated; centre≤2 mm; yaw≤1.5°; tilt≤1°; held for 1 s by framework |
| Final base | Height within 3 mm of table-resting height; tilt≤2° |
| Rest | Last 10 samples: position≤2 mm, angle≤0.5°; early speed≤10 mm/s, spin≤0.10 rad/s |
| Initial pose | XY/Z≤3 mm; angle≤2° |
| Irreversible failure | Base moved>20 mm; base/lid tilt>15/30°; box below table>10 mm; speed>0.75 m/s; gift outside interior with 1 mm margin; unexplained jump>100 mm; invalid data |
| Reset | >100 mm single-step jump plus complete nominal layout |
| Run limit | 1200 steps / 60 s at 20 Hz; 2.251× the measured 533-step completion |

The scorer's top/bottom heights are centre z±14 mm. Its top differs from the
actual mean top-corner height by 14(1−cos(tilt)) mm: 0.0021 mm at 1°, 0.0085 mm
at 2°, 0.0533 mm at 5° and 0.2127 mm at 10°. The physical height conversions
are exact when level; their small tilt-dependent difference is not hidden by
changing simulation code.

The 0.75 and full tiers share the 2 mm centre limit. A seated yaw of 2° or tilt
of 1.5° can earn 0.75, while the stricter full pose must also hold for 1 s.
Highest-tier precedence makes final awards exclusive; all tiers are reachable.
score() and distance() are read-only; score() caps at 0.75 and the framework
alone awards 1.0. In video review mode, object-only partial credit remains
capped at 0.75 and automatic full success is disabled until release is checked
manually. Slowly restoring a failed episode does not clear latches. Nothing
is consumed; restore all named initial poses to reset the real trial.

## Threshold evidence and remaining estimates

GPU observations have checked the nominal grasp, release and successful
seating conditions. **No threshold boundary has been fully calibrated** by
GPU boundary sweeps or physical measurements. The following distinction is
intentional: measured compatibility is not a claim of a calibrated tolerance.

| Check | GPU evidence | Basis and remaining limitation |
| --- | --- | --- |
| Centre ≤2 mm | demo3 measured 0.889511 mm; demo3/demo4 reached full score | Revised from 0.5 mm for the 2 mm design clearance and ruler/photo observability; no multi-seed boundary calibration |
| Seated height error ≤1.5 mm | demo3 measured −0.006421 mm | Nominal seating verified; the ±1.5 mm cutoff remains a development estimate |
| Full yaw ≤1.5°, tilt ≤1° | demo3 measured 0.065305° and 0.002152° | Nominal pose verified; angular cutoff boundaries remain development estimates |
| Both complete grippers ≥30 mm away, stillness, 1 s hold | demo3 logged hands_clear/still true and ended with framework success | Release and hold worked once at seed 0; 30 mm and rest-window cutoffs have not been calibrated at their boundaries |
| 77.4 mm grip command / measured opening ≥77 mm | Earlier revised-grip diagnostics measured about 78.0 mm without appreciable relative slip | Grip command is supported by this simulation; grip force, friction and slip limits are uncalibrated |
| Pre-descent ≥1 mm side gap, per-axis centring ≤0.5 mm, yaw/tilt ≤0.5° | demo3 minimum gap was **0.450874 mm**; diagnostic accepted=false | Development diagnostic target was not met; it is not a scoring condition or an enforced motion gate |

The partial-tier cutoffs, initial-pose tolerances, base drift/height/tilt limits,
toppling/fall/speed latches, 1 mm gift-containment margin, 100 mm reset/jump
threshold and stillness thresholds remain development estimates with CPU
boundary/counterexample tests. The 5 mm contact offset, zero rest offset and
1.2/1.0 friction coefficients are inherited simulation settings, not measured
PLA parameters. CPU tests verify implementation boundaries; they do not
establish physical validity of those boundaries.

## CPU verification and runtime logging

Run with the real installed yam_arena and designated Python environment:

```bash
"$YAM_PYTHON" tasks/seat-gift-box-lid-square/build_assets.py tasks/seat-gift-box-lid-square/assets
"$YAM_PYTHON" tasks/seat-gift-box-lid-square/audit.py tasks/seat-gift-box-lid-square "$HOME/yam-eval/gift-box-preflight.json"
PYTHONDONTWRITEBYTECODE=1 "$YAM_PYTHON" -m pytest -p no:cacheprovider tasks/seat-gift-box-lid-square/test_task.py -q
```

| Complete 20 Hz CPU audit | Left | Right |
| --- | ---: | ---: |
| Frames | 611 | 611 |
| Worst IK translation | 0.097262 mm | 0.097247 mm |
| Worst IK rotation | 0.007003° | 0.011742° |
| Joint limits | Pass | Pass |
| Robot to table/stand/base, 1 mm expanded shapes | No intersections | No intersections |
| Conservative minimum AABB gap | 70.017201 mm | 6.016520 mm |

This audit uses real URDF collision meshes and width-limited fingers. It does
not simulate grasps, contacts, dynamic lid/base forces or tracking lag. The
full tests also cover +10 mm upward reach reserve, STL volume/COM/watertightness,
USD collider dimensions, native float values, continuous grip conversion,
release height, scoring counterexamples, smoke coverage and diagnostics.

The required reverse test regenerates the 2 mm wall design and verifies that
the grasp-section test fails, then restores every asset and reruns all tests.
The centring revision also restores the former 0.5 mm full-credit threshold
temporarily: the 1.9 mm full-credit regression fails, then the source is restored
byte-for-byte and the full suite passes (126 passed, 1 live-GPU test skipped).
Exact test counts, environment, asset hashes and limitations are in validation.json.
One optional GPU-reader test is skipped in the CPU suite because it needs a
live Isaac Lab scene; its public API shape is tested separately on CPU.

`TASK_DEBUG=1` prints every step during descend, grip alignment, closing,
lifting, carrying, placement alignment/lowering/release and ±1 s; otherwise
step 1/every20. It records actual object pos/quat/velocities, TCPs, finger/link
poses, openings, lid-to-TCP transform/change, base drift/tilt, four-side fit,
and nearest convex distances plus intersecting finger/panel pairs. At step330
it marks second-descent entry. Added contact diagnostics are caught separately
and cannot latch scoring failure. Existing complete-gripper release sensing
remains part of scoring. Diagnostic GJK zero means contact/intersection, not a
signed penetration depth.

## Reproduce the evaluated runs

Use the installed yam_arena/Inspect Robots environment. Read instruction from
task.json, run long commands inside tmux, and save each run in a fresh log
directory. The evaluated runs used seed 0, object_jitter=0 and 20 Hz. Both
`--seed` and `--no-prompt` were confirmed in the installed CLI help.

```bash
T=seat-gift-box-lid-square
INS="$(python -c "import json; print(json.load(open('tasks/$T/task.json'))['instruction'])")"
unset TASK_DEBUG
set -o pipefail
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/$T/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$INS" --scorer task_score \
  --max-steps 1200 --seed 0 --no-prompt --store-frames \
  --log-dir "$HOME/yam-eval/$T-demo4" 2>&1 | tee "$HOME/yam-eval/$T-demo4.log"
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/$T/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$INS" --scorer task_score \
  --max-steps 1200 --seed 0 --no-prompt --store-frames \
  --log-dir "$HOME/yam-eval/$T-dream2" 2>&1 | tee "$HOME/yam-eval/$T-dream2.log"
```

Configure model credentials outside the repository. A 200-step diagnostic
smoke uses the scripted command with `TASK_DEBUG=1`, `--max-steps 200` and a
fresh smoke log path. Grip-slip checks use lid rotation relative to the
gripper, not its absolute tilt during transport. Judge the result JSON status,
score and termination reason; CLI exit code alone is insufficient.

## GPU results and diagnostic measurements

| Commit / run | Score | Steps | Termination | JSON status | Framework runtime |
| --- | ---: | ---: | --- | --- | ---: |
| 253f934 / demo4, yam_scripted | **1.0** | **533** | success | success | 78.19 s |
| 253f934 / dream2, dreamlabs/dream-1-beta | **0.0** | **1200/1200** | max_steps | success | 153.33 s |
| 9a03999 / demo3, TASK_DEBUG=1 | 1.0 | 533 | success | success | 120.65 s |

The submitted videos come from demo4 and dream2 at the same commit. Both ran
without TASK_DEBUG and reported error=null. The only scene change from demo3
to commit 253f934 was the lid's rendering colour; geometry, dynamics, scoring
and the scripted trajectory stayed unchanged. This does not turn older
diagnostic measurements into new measurements from demo4.

Demo3's last printed pose at step **520** was:

| Quantity | Measured value |
| --- | ---: |
| Lid top − (current base rim + 2 mm top panel) | −0.006421 mm |
| Lid tilt from vertical | 0.002152° |
| Lid yaw relative to base | 0.065305° |
| Horizontal centre offset | 0.889511 mm |

At that frame, full_candidate, hands_clear and still were true, with no latched
violation. Framework success was recorded at step 533; a separate precise
object-pose record was not printed for that terminal frame. Both gripper
readers and added diagnostics reported ok throughout the printed states.
The gift remained in the base and the lid did not follow the withdrawing hand.

At the start of second descent (step **330**), dx/dy were
**+0.184033/−1.131257 mm**, lid tilt was **0.336050°**, and relative yaw was
**0.056186°**. Four projected side gaps (−x,+x,−y,+y) were
**1.784577, 2.147171, 3.217484, 0.450874 mm**. The minimum was therefore only
**0.451 mm**, below the 1 mm diagnostic target; `alignment.accepted=false`.
The open-loop trajectory continued and seated successfully. This check is
diagnostic only and does not prove a safe clearance margin for other trials.

Media **M-B336HN** contains the top-camera demo4 and dream2 videos, encoded as
H.264/yuv420p with faststart. The setup image is the first top-camera frame of
a separate one-step scripted run at commit 253f934, requested at 1280×704.
The framework renders at 640×360 and resizes that output; it is not native
1280 rendering. The screenshot keeps the full scene and has no embedded
metadata. The videos remain 320×176.

Only **seed 0** has been tested. Threshold-boundary robustness, other layouts,
grip forces, actual cooked collision shapes, contact-force calibration and
physical printing/manufacturing quality remain unverified. No physical
specimen has been measured.
