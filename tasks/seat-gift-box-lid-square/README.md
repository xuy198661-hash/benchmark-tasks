# Seat a gift-box lid squarely

Put a slip-over lid fully onto a packed gift box without wedging its skirt,
moving the base, or removing the gift. Category: Societal relatability. The
right YAM arm performs the nominal demo and the left parks; this task is not
claimed to require bimanual manipulation.

本想法源自他队已失效占位的 PR #2264，本实现从头编写。

The revised design passes CPU checks. The earlier e40fccc GPU demo scored 0.0;
the revised physical demo, Dream-1, media and submission remain pending.

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
| Colour / pose | Burgundy, opening up | Pale, top up and skirt down |

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
Whether it prevents settling through the 2 mm side clearance remains a GPU test.

## Scoring

Task instruction: “Put the lid fully onto the gift box, level and square. Keep
the gift inside and leave the box where it is. Let go when it is seated.”

The final-state tiers are unchanged. Credit requires valid initial/state data,
no latched violation, still objects and both complete grippers ≥30 mm from all
objects, using actual public link transforms. Unavailable scoring reads fail
closed. `score()` and `distance()` do not mutate episode state.

| Tier / rule | Numeric condition |
| --- | --- |
| 0.25, rim rest | Bottom within 2 mm of rim; centre≤25 mm; tilt≤5° |
| 0.50, engagement | At least 10 mm below rim, no deeper than seated+1.5 mm; centre≤8 mm; tilt≤10° |
| 0.75, seated | Top within 1.5 mm of rim+2 mm; centre≤1 mm; yaw≤3°; tilt≤2° |
| Full success | Seated plus centre≤0.5 mm; yaw≤1.5°; tilt≤1°, held 1 s by framework |
| Final base | Height within 3 mm of table-resting height; tilt≤2° |
| Rest | Last 10 samples: position≤2 mm, angle≤0.5°; early speed≤10 mm/s, spin≤0.10 rad/s |
| Initial pose | XY/Z≤3 mm; angle≤2° |
| Irreversible failure | Base moved>20 mm; base/lid tilt>15/30°; box below table>10 mm; speed>0.75 m/s; gift outside interior with 1 mm margin; unexplained jump>100 mm; invalid data |
| Reset | >100 mm single-step jump plus complete nominal layout |
| Run limit | 1,200 steps / 60 s at 20 Hz, 1.97×610-step plan |

Highest tier only, not a sum; score() caps at 0.75, and the framework alone
awards 1.0 after its success hold. Video-review mode caps at 0.75 and disables
full success until manual release review. Slowly restoring a failed episode
does not clear latches. Nothing is consumed; restore all named initial poses.

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

## Subsequent physical runs

Read instruction from task.json. Confirm optional flags in --help. Run long
commands inside tmux, save logs with tee, and use a fresh directory per commit:

```bash
T=seat-gift-box-lid-square
INS="$(python -c "import json; print(json.load(open('tasks/$T/task.json'))['instruction'])")"
TASK_DEBUG=1 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/$T/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "$INS" --scorer task_score \
  --max-steps 200 --store-frames --log-dir "$HOME/yam-eval/$T-smoke" \
  2>&1 | tee "$HOME/yam-eval/$T-smoke.log"
```

Smoke must have good scoring reads, no Traceback, stable base/gift, and during
closing/holding/lifting actual gap≥77.0 mm, no obvious slip and lid tilt≤0.5°.
If it fails, stop before demo. For the full demo use `--max-steps 1200` and a separate demo log path. Judge JSON status,
score and termination, not CLI exit code. Before submission check run_limit is
above actual demo steps and at most 20× them. Dream-1 and uploads require approval.

## Known evidence and limits

Previous e40fccc demo: JSON status success, task score0, 530steps, policy_stop,
77.1299 s framework time. At step160, gap69.3527mm, tilt0.3082°, yaw3.6271°;
by180, gap58.8830mm and tilt4.4558°. Bias starts during closing, worsens during
lift. Nominal URDF reconstruction at those openings intersects sidewall/top
colliders, but old logs lacked absolute poses and cannot prove actual signed
penetration. The earlier base moved/toppled and the lid did not seat.

Unverified: real grip force/slip/torque, tracking and alignment convergence,
release/unhooking, actual cooked collision shapes, 5mm contact-offset effects,
settling/full score, debug runtime cost, actual completion limit ratio, physical
sample/manufacturing quality, Dream-1/media, and task-board competition since
the original reference. CPU command targets and deterministic geometry are not
claims of GPU success. A full current-PR duplication review is required before
submission; an expired original hold alone is insufficient evidence.
