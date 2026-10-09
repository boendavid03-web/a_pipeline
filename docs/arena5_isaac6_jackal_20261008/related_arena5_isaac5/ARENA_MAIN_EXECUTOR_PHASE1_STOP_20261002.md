# Arena Main Executor Phase 1 — bounded stop (2026-10-02)

## Result

`STOP_REASON=JACKAL_CONTACT_ISOLATED_GATE_FAILED_AFTER_TWO_JUSTIFIED_CHANGES`

The isolated Jackal control chain remains unsuitable for a formal ROS control probe. No formal compatibility source, installed runtime, URDF, Nav2 setting, HuNav source, or scenario was changed. Phase 2 (ROS), Phase 3 (HuNav time repair), and scenarios 1–5 were not run. The old `first_221` remains pre-fix evidence only.

## Decision table from A, B, and C

| Hypothesis | A evidence | B evidence | C relevance | Change tested / risk | Discriminating result |
| --- | --- | --- | --- | --- | --- |
| 229 diagnostic floor caused the contact failure | `FixedCuboid` had 0.8–1.0 m torsional patch; formal GroundPlane has zero | Official wheeled reference does not establish the Arena diagnostic floor | None | Recreate formal `add_ground_plane(size=100, z_position=0)` in isolated stage; low risk | 230/234 improved yaw over 229 but still failed wheel tracking and yaw consistency. The diagnostic floor amplified the problem but does not explain all of it. |
| Four wheel contact friction causes skid-steer scrub | A confirmed wheel geometry and noted rigid lateral scrub | Official Isaac 5.1 Jackal explicitly binds 0.2/0.2 wheel material | None | 235 bound 0.2/0.2 to four imported wheel Cylinder source prims; geometry, ground, drive, solver, commands fixed. Risk: importing a material value alone may not reproduce the official contact model. | Four composed robot collision instance proxies resolved to the new material. Wheel and yaw metrics improved modestly but failed the joint gate. |
| Velocity solver is causing contact/drive spikes | A measured velocity iteration count 1 and spikes | Official reference has higher velocity iterations | None | 236 changed count 1→16 while retaining 235's material. Risk: solver iteration count alone may not address drive/contact coupling. | Low-speed yaw and tracking worsened; high-speed tracking did not converge. |
| Force-type drive or a different skid-steer contact representation is needed | Still plausible after the bounded experiments | Official reference differs in drive and contact model | None | Not tested. This would be a new investigation, with drive limits/units and contact forces measured first. | Further parameter changes would exceed D's two-round stop rule. |
| HuNav integrates wall-stamped request intervals | Separate C forensic traced Task Generator wall/epoch stamp → HuNav `dt` → LightSFM integration | None | Direct root cause of pedestrian speed per simulation second | Time repair remains gated on isolated and formal Jackal control pass. | No HuNav test or repair was attempted here. |

`OFFICIAL_ADAPTER_MIGRATION_NOW=NO`.

## Experiment contract and observations

All valid new probes used the installed Jackal URDF, the same URDF import config, two Arena differential/articulation graphs, 60 Hz physics, a 1 s settling period, and command windows `-0.2 rad/s × 3 sim s → 0 × 1 s → -0.5 × 3 sim s → 0 × 1 s`. The diagnostic graph sets angular velocity directly, so these probes do **not** test ROS delivery, Nav2, or odometry publication.

The analysis uses a joint acceptance gate in each commanded window: correct left/right target signs; median absolute wheel velocity error relative to target ≤20%; opposite-sign actual velocity samples ≤5%; peak actual wheel speed ≤3× target; `|chassis yaw / wheel-model yaw|` between 0.70 and 1.30; and the `-0.5` yaw magnitude ≥1.5× the `-0.2` yaw magnitude. A single yaw increase was not sufficient. These numerical limits were formalized in the comparison after the runs; the large measured failures do not depend on a narrow cutoff choice. They are an operational gate for reliable downstream control, not a fitted physics parameter.

| Run | Ground | Wheel material | Velocity solver | yaw at -0.2 / -0.5 (rad, sampled 2.9 s) | Median relative wheel error at -0.2 / -0.5 | Opposite-sign fraction at -0.2 / -0.5 | Peak speed / target at -0.2 / -0.5 | Gate |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 229, prior | FixedCuboid | imported | 1 | -0.003 / -0.250 | 1.325 / 0.313 | 0.575 / 0.050 | 7.3 / 9.3 | FAIL; different floor |
| 230, Experiment 0 | formal GroundPlane | imported | 1 | -0.076 / -0.501 | 1.329 / 1.376 | 0.308 / 0.350 | 17.7 / 11.2 | FAIL |
| 234, repeat with stage inventory | formal GroundPlane | imported | 1 | -0.076 / -0.501 | 1.329 / 1.376 | 0.308 / 0.350 | 17.7 / 11.2 | FAIL; reproduces 230 |
| 235, justified change 1 | formal GroundPlane | 0.2/0.2 | 1 | -0.100 / -0.537 | 1.252 / 0.819 | 0.317 / 0.283 | 15.3 / 9.6 | FAIL |
| 236, justified change 2 | formal GroundPlane | 0.2/0.2 | 16 | -0.074 / -0.528 | 1.471 / 1.140 | 0.367 / 0.333 | 26.7 / 10.2 | FAIL |

Wheel-model/chassis yaw ratios in 236 were 0.27 and 0.50. Four wheels retained the intended displacement signs, but instantaneous wheel speed frequently reversed and spiked. The second change reduced, rather than improved, the low-speed response. Run 231 and 232 ended before stepping because diagnostic instrumentation initially searched under the robot instead of the wheel collider source prims; they are not counted as physics experiments. Run 233 established that the imported wheel collisions are USD instances. Run 234 confirmed all four composed `/World/jackal/*wheel_link/collisions/mesh_0/cylinder` paths are valid, correcting that instrumentation path error.

Formal GroundPlane was created with the exact `world.scene.add_ground_plane(size=100, z_position=0.0)` call used in the loaded `run_isaacsim.py`. Its authored material was static/dynamic friction 0.5/0.5 and restitution 0.8; its collision mesh had zero torsional patch radii. The probe's wheel material was authored at the four `/colliders/*wheel_link/mesh_0/cylinder` source prims; all four composed robot instance proxies reported the 0.2/0.2 material binding. The articulation root reported solver count 16 in run 236. These stage checks confirm that both experimental changes took effect.

Raw captures, executed source copies, SHA files and logs are under `[LOCAL_PATH]{230,231,232,233,234,235,236}/`. The computed comparison with explicit criteria, all metrics and probe hashes is `[LOCAL_PATH]` (SHA-256 `ac686cc641ac154a51231c6c7763ea0f04b0a7dfae5402cc0511ca433cdf8fd7`). The retained diagnostic script is `scripts/validation/arena_control_chain/probe.py` (SHA-256 `094aeb892017fcf07dd9bb81ece74b9884329a1bee2ff314a4f024adb511509f`). Its diff from the old 229 diagnostic source is `[LOCAL_PATH]` (SHA-256 `931b22c8bd36d5a8601347e0d478c43104f437a0e8f3c09aee87285025f99c0c`). It is diagnostic instrumentation only.

## Provenance and bounded stop

Branch: `experiment/scenario-topology-ab-e314f56`. Existing tracked files remained clean; the previous untracked reports, scripts, and large assets were preserved. The loaded formal `UrdfToUsd.py`, `control/__init__.py`, `control/differential.py`, and installed Jackal URDF hashes after the probes matched the before hashes respectively: `5258725c…`, `634beda0…`, `e209e266…`, `5c7cdb47…`. No unrelated Isaac/ROS processes or compute GPU applications were active at the end of the probe sequence.

`LAST_PASS_GATE=FORMAL_GROUND_STAGE_AND_WHEEL_MATERIAL_BINDING_VERIFIED`, not isolated locomotion acceptance. Further blind parameter changes have low value because the two evidence-backed changes failed the combined wheel/chassis gate, and the remaining drive/contact hypothesis needs direct force/contact measurements. The next decision is a new, separately scoped read-only investigation of effective wheel-ground contact impulses, drive effort and official four-wheel skid-steer implementation before selecting another isolated fix. Under the present D stop rule, no formal patch or downstream baseline run is justified.

`DWB_BASELINE_READY=NO`; `READY_FOR_MPPI=NO`.
