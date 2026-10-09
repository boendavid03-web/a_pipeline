# Arena Jackal angular control: bounded Phase A diagnosis (2026-10-02)

## Decision

```text
STOP_REASON: STOP B — PHASE_A_STATUS = BLOCKED_NONCONVERGENT
BLOCKER: A5 CONTACT_PHYSICS / articulation under ground contact
LAST_COMPLETED_PHASE: Phase A static chain audit and isolated command-response probe
ROOT_CAUSE_CONFIDENCE: high for the isolated contact-dependent failure layer; medium that the same layer explains first_221; low for the exact PhysX/material/geometry mechanism
PHASE_B: NOT_STARTED
PHASE_C: NOT_STARTED
PHASE_D: NOT_STARTED
```

The current compatibility adapter does not yet have a demonstrated faithful Jackal angular response. The old `first_221` bag remains a complete record of that episode, but it is **not** an accepted social-navigation baseline. No new Scenario 1–5 baseline runs were started.

## Current control chain, from live code and Scenario 1 bag

| Stage | Current implementation and evidence | Units / clock / limits |
|---|---|---|
| Nav2 | `/task_generator_node/jackal/cmd_vel_nav` is recorded in `first_221` with mostly negative `angular.z`; `/task_generator_node/jackal/cmd_vel` also records negative angular commands. The final topic has 440 messages in the bag. | Twist m/s and rad/s; Nav2 `use_sim_time=true`; controller 1 Hz. Exact internal relay between these two bag topics was not instrumented in this phase. |
| Isaac import | The active Task Generator `_spawn_robot()` in `isaac_simulator.py` sends its `service_namespace(robot.name, 'cmd_vel')` to `isaac/urdf_to_usd`. The installed child service `isaac_utils/services/UrdfToUsd.py` passes that exact topic to `Control.parse()`. | ROS bridge subscription is created by the imported robot graph; the isolated probe below bypasses this subscription, so original-run subscription receipt remains unmeasured. |
| Differential graphs | `isaac_utils/graphs/control/__init__.py` reads live Jackal `control.yaml`: `wheel_separation=0.36 m`, multiplier `1.5`, effective distance `0.54 m`; radius `0.098 m`. It makes two graphs, front left/right and rear left/right, from `isaac_utils/graphs/control/differential.py`. Each uses `OnPlaybackTick → ROS2SubscribeTwist → DifferentialController → IsaacArticulationController`. | Wheel targets rad/s; `maxWheelSpeed=10`. The parser passes negative configured minima as `maxLinearSpeed=-2` and `maxAngularSpeed=-4`, but Isaac 5.1 `OgnDifferentialController.cpp` applies `fabs` to nonzero maxima. This field reversal is a code defect to record, but does **not** explain this probe's mismatch. Graph execution follows playback ticks; the graph contains no explicit command timeout. |
| Joint and chassis | The same `UrdfToUsd` path imports the Jackal URDF with velocity drives; four expected wheel joints exist. Imported drive properties in the probe: damping `1000`, stiffness `0`, maxForce `3.4e38`, drive type `acceleration`, axis `Y`. | Probe physics step `1/60 s` simulation. Actual wheel joint position and velocity were sampled every `0.1 s` simulation. |
| Odom | Installed `isaac_utils/graphs/odom.py` uses `IsaacComputeOdometry` on `base_link`, then publishes `/task_generator_node/jackal/odom` with simulation-time stamps. | Isolated probe measured chassis pose directly; it did not start the odom publisher. The Scenario 1 bag measured only about `-0.148 rad` odom yaw against about `-1.107 rad` integrated final angular command, as recorded in the handoff. |

The relevant installed control files were read directly. SHA-256: `UrdfToUsd.py` `5258725cc075d2879abb248629fd3a59a6404823ffbd516d7d1c38a39c3ae38b`; control parser `634beda0130ce4596dfcbd6f275daf8a2c76c5e0fda767e386b302fedafae028`; differential graph `e209e26656bd63532335ec19a9c579d56d12567998a2b5eb6af1f1f0138da974`; live `control.yaml` `45b57a7787c5dca2de172f8f8ef1966c9d980c756d31d9a5d8456678f180b970`; installed Jackal URDF `5c7cdb473934cbbd3beb6e4ecfd29d09a4a4f1331930a3a8048e5c3752bbfb4b`.

## Probe evidence

The [probe script](scripts/validation/arena_control_chain/probe.py) imported the installed Jackal URDF using the same import settings as `UrdfToUsd`, built the same pairwise differential and articulation nodes, and applied `angular.z=-0.2 rad/s` for 3 simulation seconds, zero for 1 second, `-0.5 rad/s` for 3 seconds, then zero for 1 second. It sampled graph wheel targets, actual joint positions and velocities, chassis pose, simulation time, and wall time. This is **instrumentation only**: it directly sets the differential node input and does not exercise ROS command delivery or Nav2. Physics floor top was at `z=0`; Jackal root was placed at its formal `z=0.0635 m`. The probe source SHA-256 for both decisive runs was `dfa5592d154f4eb60ee977eb6c88f82d5114cf403afc4579c0b4cf4ecd53719d`.

| Probe | Command | Four graph targets (FL, FR, RL, RR), rad/s | Actual wheel displacement over sampled 2.9 sim s, rad | Chassis yaw change |
|---|---:|---|---|---:|
| Ground contact `229` | `-0.2` | `+0.551,-0.551,+0.551,-0.551` | `+0.243,-0.400,+0.328,-0.297` | `-0.00272 rad` |
| Ground contact `229` | `-0.5` | `+1.378,-1.378,+1.378,-1.378` | `+1.609,-2.965,+2.712,-1.432` | `-0.25003 rad` |
| No nearby ground `228` | `-0.2` | same | `+1.598,-1.598,+1.598,-1.598` | approximately `0`, because it was falling freely |
| No nearby ground `228` | `-0.5` | same | `+3.995,-3.995,+3.995,-3.995` | approximately `0`, because it was falling freely |

With contact, the wheel displacement implies about `-0.115` and `-0.791 rad` using the nominal no-slip differential model, while chassis yaw was `-0.00272` and `-0.25003 rad`. The first pair has very poor wheel-target tracking; both pairs have substantial wheel-to-chassis yaw loss. Without nearby ground, the wheel joints track their targets to about `1e-4 rad/s`, which localizes the failure to the contact/constraint-dependent response. The zero-ground run is an **articulation control** comparison only; its free-fall chassis yaw is not a locomotion measure. With ground, root height stayed approximately `0.06350 m` through both command windows, so the result is not explained by the earlier incorrect probe spawn height.

Raw data and logs: `[LOCAL_PATH]` (floor 100 m below robot) and `[LOCAL_PATH]` (ground at `z=0`). Each contains `probe/probe.json`, `analysis.json`, `probe.log`, `manifest.json`, and a copy of the executed probe script. `probe.json` SHA-256: `228` `6f236bbf13e4935997112e0a47fa2851cb26a9ba10300f408e6a30f915bf9861`; `229` `44178cd1ea0522c6033414452a178e846fd69dc107d27bc7597b02d3d739491d`. The [analyzer](scripts/validation/arena_control_chain/analyze.py) recomputes the table from those raw samples. Runs `222` and `223` had an inactive playback graph and are invalid; `224` had a low root spawn; `225` and `226` were preliminary ground checks; `227` failed to initialize an articulation with no physics floor. They are retained, not counted as successful probe evidence.

## Bounded diagnosis and stop

```text
CONFIRMED_FACTS:
- Current installed graph computes the expected wheel target signs and magnitudes.
- Wheel drives have nonzero target capacity; maxForce is not a small configured cap.
- In free fall, wheel joints track both command targets closely.
- Under ground contact, wheel tracking and chassis yaw have large mismatch.
- The formal Scenario 1 bag also shows a large angular command/odom yaw mismatch.

STRONG_INFERENCES:
- The first reproducible downstream failure is A5 CONTACT_PHYSICS, including joint/contact coupling.
- A pure DWB direction error or pure differential conversion bug cannot explain this isolated contrast.

UNKNOWN:
- Which contact/constraint property creates the loss: imported wheel collider orientation/contact patch, tire lateral traction, PhysX material binding, articulation solver behavior, or interaction among these.
- Actual wheel targets and ROS subscriber receipt during `first_221`; its bag has no graph target instrumentation.
- Whether odom exactly follows independently sampled chassis yaw under the formal runtime; probe did not publish odom.

LAYERS_ALREADY_RULED_OUT:
- A2 DIFF_DRIVE_CONVERSION for the isolated commands.
- A3 WHEEL_TARGET_APPLICATION without contact (the targets reach the articulation and joints track).
- The negative max-speed fields as the cause of this mismatch (Isaac node takes absolute value).

ATTEMPT_1:
CHANGE: none to formal compatibility runtime; diagnostic graph and samples only.
RESULT: contact-dependent mismatch localized; exact contact mechanism unresolved.

ATTEMPT_2:
CHANGE: not started; no single justified compatibility fix was identified.
RESULT: not applicable.

WHY_NOT_CONTINUE_BLINDLY:
Changing friction, wheel separation/radius, mass, or controller gain here would be parameter fitting without identifying the imported contact defect. It could make an isolated turn look better while corrupting the frozen baseline.

FILES_MODIFIED: only new diagnostic scripts and this report; no live adapter, Nav2, scenario, HuNav, or frozen configuration file changed.
PATCHES: none formal; diagnostic script is instrumentation only.
BASELINE_RESULTS_STILL_VALID: `first_221` is a valid raw episode record, but not final baseline-valid. Prior frozen short/default results retain their recorded status; no new acceptance is inferred.
```

All owned probe processes exited with code 0 for the decisive runs; no Isaac, rosbag, Task Generator, HuNav, or Arena process remained at final audit. GPU and process occupancy checks found no other active owners, and available disk space was about 396 GB. The isolated probes did not capture a pre-run ROS node list or a PID tree; this is recorded in each manifest and does not certify full-stack process isolation. No branch switch, commit, or push occurred.

## Recommended next decision

- **Option A (recommended):** authorize a separate Isaac contact-model investigation that records wheel-ground contact points/material bindings and tests one justified collider or material correction against the same two command windows, then against the formal ROS graph. Keep it outside the frozen baseline until the correction passes.
- **Option B:** replace the compatibility Jackal physics/control implementation under an explicit new architecture contract and requalify all navigation evidence.
- **Option C:** defer the DWB baseline and preserve the current run as pre-fix evidence. Official adapter migration remains a later decision and was not performed.

Reproduce the decisive isolated comparison with new output directories and unoccupied domains:

```bash
ROS_DOMAIN_ID=228 scripts/validation/arena_tf_throttle/python.sh scripts/validation/arena_control_chain/probe.py --no-floor --output [LOCAL_PATH]<unique_no_contact_dir>
ROS_DOMAIN_ID=229 scripts/validation/arena_tf_throttle/python.sh scripts/validation/arena_control_chain/probe.py --output [LOCAL_PATH]<unique_contact_dir>
python3 scripts/validation/arena_control_chain/analyze.py [LOCAL_PATH]<unique_contact_dir>/probe.json
```
