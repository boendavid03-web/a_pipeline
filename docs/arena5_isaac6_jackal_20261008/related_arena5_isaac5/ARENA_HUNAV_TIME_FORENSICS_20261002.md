# Arena + HuNav simulation-time forensics (C, 2026-10-02)

Scope: read existing source, launch files, parameter snapshot, `first_221` bag and logs. No scene launch, parameter change, or runtime repair was performed. This report concerns pedestrian timing only.

## Required handoff

```text
TIME_MISMATCH_CONFIRMED: YES; 0.299 m/s by message/wall time, about 0.610 m/s by Isaac /clock in the navigation interval.

HUNAV_INTEGRATION_CLOCK: Request Agents.header.stamp supplied by Task Generator. HuNav differences adjacent request stamps to form dt; LightSFM integrates position += velocity * dt. Thus the effective integration clock was wall/epoch time in first_221. HuNav itself does not independently sample wall time on this path.
HUNAV_TIMER_CLOCK: No independent HuNav compute timer. The service is driven by Task Generator's 0.1 s rclpy timer. With Task Generator use_sim_time=false, that timer uses the node's system clock; observed rate was about 10 Hz wall, about 20.4 calls per simulated second.
HUMAN_STATE_STAMP_SOURCE: HuNav publishes the incoming, pre-integration Agents message unchanged. Its header stamp is Task Generator's request stamp (epoch-like); /people uses that same stamp. The published pose corresponds to the prior compute response, so compare a position step with the preceding stamp interval.
TASK_GENERATOR_CLOCK: Effective use_sim_time=false in first_221 parameter dump; node.get_clock().now() therefore supplied epoch/system stamps.
ISAAC_APPLY_TRIGGER: Each Task Generator ComputeAgents response invokes /isaac/move_prim once per human; Isaac applies the supplied pose via geom.move. No simulation-time gate or velocity integration is present in that service. first_221 log has 317 distinct request stamps and 951 applied/read-back samples for three humans; logged pose error is 0 to six decimal places.
ISAAC_CLOCK_SOURCE: Isaac OnPlaybackTick -> IsaacReadSimulationTime -> ROS2PublishClock /clock. The navigation interval's /clock advanced about 10.526 s during 21.492 s wall time. /clock may publish duplicate values on render ticks; its elapsed time is still the simulation-time reference.

FIRST_LAYER_USING_WALL_TIME: Task Generator's use_sim_time=false node clock, used both for its 0.1 s update timer and the request header stamp. The stamp-derived dt is the direct integration error; the timer increases the number of updates per simulated second when RTF<1.

ROOT_CAUSE: Clock-domain mismatch between Task Generator's wall-clock HuNav request timestamps and Isaac's simulation clock. HuNav correctly uses the request's dt, but the requester provides wall time; Isaac applies each resulting pose while simulation time advances at roughly half wall rate.
CONFIDENCE: High for the first_221 mechanism (source plus per-step bag and Isaac read-back); an isolated timing probe is still required to validate the proposed repair across start, pause, reset and runtime loading.

MINIMAL_FIX_LOCATION: `isaac_sim/arena_ws/src/arena/arena-rosnav/task_generator/launch/task_generator.launch.py`, Task Generator node parameters, and/or the launch path that passes its effective `use_sim_time`; keep `task_generator/simulators/human/hunav/hunav.py::_move_entity_callback` as the verification point. The existing node launch omits `use_sim_time`, despite other launched nodes setting it true. Confirm effective loaded source/parameter before applying a fix.
MINIMAL_FIX_SEMANTICS: Make the Task Generator timer and `node.get_clock().now()` follow Isaac `/clock`, so adjacent ComputeAgents request stamps and HuNav dt are simulation-time deltas. Handle first clock availability, paused/unchanged time, and reset/backward jump without integrating stale or negative dt. Prefer one clock domain throughout; do not add an RTF multiplier.

DO_NOT_CHANGE: Original scenario, HuNav `max_vel`/desired speed (0.3 m/s), SFM force/speed parameters, Nav2/DWB settings, Isaac clock rate, or any 0.3 -> 0.15/RTF compensation. Do not require RTF=1 or treat `first_221` as a valid social-navigation baseline.

ISOLATED_TIMING_TEST: With no Nav2/full Arena scene, use one straight-route pedestrian at fixed 0.3 m/s and the same Task Generator -> ComputeAgents -> /isaac/move_prim path. Record monotonic wall time, /clock, request stamp, HuNav returned position/velocity, and Isaac applied/read-back position for each update. Deliberately run at two distinct wall RTFs and include a simulation pause and reset. Use a new domain and output directory.
PASS_CRITERIA: During steady straight motion, displacement / elapsed `/clock` agrees with configured 0.3 m/s within 5%; request stamp deltas track `/clock` deltas within one clock publication interval; pause causes no position drift, reset produces no jump or negative-step integration, and Isaac read-back agrees with HuNav returned XY within 1 mm. RTF may differ from 1; the speed criterion must hold at both RTFs.

HANDOFF_TO_MAIN_EXECUTOR: Change only the clock/timer/stamp contract at Task Generator after the Jackal gates in D. Run the isolated timing probe first. Retain the original 0.3 m/s scenario and existing bags. Do not infer a formal baseline pass from this forensics report.
```

## Provenance and measured reconstruction

- Evidence: `[LOCAL_PATH]` (`raw_rosbag_0.db3` SHA-256 `ee3b78c47463c8bfca2d863e97ff2f0acfa2c315cf1bb1d27c6ad9033ae1da8d`; `metadata.yaml` SHA-256 `b9ad0ac66c20fb7eec8b5268a4409201c72cdc00bf55c3214871362f15d7a689`). Parsed SQLite in read-only mode with the installed ROS Humble/HuNav message types. `/clock`: 2460 samples. `/task_generator_node/human_states` and `/task_generator_node/people`: 317 each, three people per message. `launch.log` contains 951 `HUNAV_ISAAC_SAMPLE` records, exactly three for each unique stamp.
- Effective Task Generator parameter snapshot: `use_sim_time: false` in `effective_task_generator_params.yaml`. The human launch passes `use_sim_time=true` to `hunav_agent_manager`, but no effective HuNav parameter dump was captured. Its service dt is taken from incoming stamps regardless of its own clock parameter. Nav2 controller's effective `use_sim_time: true` is unrelated to pedestrian integration.
- Runtime resolution check under `[LOCAL_PATH]`: `task_generator` resolves to this repository's `isaac_sim/arena_ws/build/task_generator`, whose HuNav file is a symlink to `src` and shares SHA-256 `8c221c1b3fa01c1c569700ff8f8be6301a6f11ca0e7a95a25f31b5dfe7d518b8`. The active Isaac child path is recorded in `preflight.json`; its installed `MovePrim.py` and `graphs/time.py` were read. Local `[LOCAL_PATH]` HuNav/Task Generator copies differ in hash, so the repository runtime copy is cited for the Task Generator behavior, and the actual host-overlay HuNav binary/service semantics are cross-checked against its local source and bag. This is not a binary-to-source identity proof for HuNav.
- Human 1 navigation subset: 215 samples, position `(13.988568, 2.011432)` to `(9.446432, 6.553568)`, path 6.423551 m. Bag receive elapsed 21.491576 s; message stamp elapsed 21.491940 s. Interpolating `/clock` at the two bag receive times gives 10.526219 s; `6.423551/10.526219 = 0.61024 m/s`. Using nearest preceding `/clock` samples instead gives 10.533334 s and 0.60983 m/s. Both show the same mismatch. `6.423551/21.491940 = 0.298882 m/s`.
- Stable subset (human 1, indices 20..199): 5.370060 m / 17.899962 stamp seconds = 0.300004 m/s, versus 5.370060 m / 8.933334 `/clock` seconds = 0.601126 m/s. The median absolute per-step residual for `distance - 0.3 * preceding_stamp_delta` is about `2.44e-8 m` (180 steps); using the same message's stamp interval gives about `8.64e-5 m`. The one-message lag is expected because `publish_agent_states()` sends the incoming state before `tree_tick(dt)` updates the response.
- Representative steady steps (human 1). Each row compares the **position change from sample i to i+1**; `/clock` is the nearest prior bag clock value:

  | i | bag wall delta (s) | header stamp delta (s) | `/clock` delta (s) | position delta (m) | reported linear speed (m/s) |
  |---:|---:|---:|---:|---:|---:|
  | 25 | 0.100573 | 0.100615 | 0.050000 | 0.029945 | 0.300000 |
  | 26 | 0.119480 | 0.119589 | 0.050000 | 0.030184 | 0.300000 |
  | 27 | 0.081642 | 0.081566 | 0.033333 | 0.035877 | 0.300000 |
  | 28 | 0.098470 | 0.098366 | 0.050000 | 0.024470 | 0.300000 |
  | 29 | 0.099818 | 0.099880 | 0.050000 | 0.029510 | 0.300000 |
  | 30 | 0.099984 | 0.100111 | 0.050000 | 0.029964 | 0.300000 |

- Human stamp absolute values span `1790934470.4334447` to `1790934502.11739`, matching bag receive epoch within 0.5–3.7 ms (median 0.71 ms before receive). `/clock` spans about 11.93 to 27.55 s over that same human capture. The bag's `/people` and `/human_states` messages have identical stamps for all 317 paired publications. Both are HuNav publications; neither is a Task Generator subscription. Task Generator drives HuNav by a synchronous service call, then drives Isaac by `/isaac/move_prim`. No recorded evidence indicates a `human_states` subscriber in this apply path.
- The navigation interval contains 948 `/clock` messages over roughly 21.49 wall seconds (~44.2 Hz publication); 631 adjacent values advance by about 1/60 s, 316 repeat. `/clock` advance rate is about 0.49 simulated second per wall second. This explains the factor of about two once wall-time integration is established; it is not a requirement to force Isaac to RTF=1.
- Isaac apply log: all 951 records say `applied=1`, with reported read-back XY error `0.000000` at six decimal places. The first update shows `stamp=1790934470.433444838`, `dt=0.116070`, HuNav position `(13.988568, 2.011432)` and Isaac read-back `(13.988568, 2.011431)`. Logs thus support direct pose application, not independent Isaac motion integration. Millimetre or submillimetre accuracy beyond printed precision remains unmeasured.

## Source chain and clock ownership

| Layer | Source evidence | Clock / timer / dt / stamp |
|---|---|
| Scenario speed | `isaac_sim/arena_ws/src/arena/arena-rosnav/arena_bringup/configs/hunav_agents/default.yaml:12`; Task Generator `simulators/human/hunav/__init__.py:181` and `hunav.py:722` | Default `max_vel=0.3` becomes `Agent.desired_velocity`. Scenario JSON gives routes, not a 0.15 m/s override. |
| Task Generator node | `task_generator/launch/task_generator.launch.py:132-153`; `hunav.py:401-416,479-485`; first_221 effective parameter dump | Node `/task_generator_node`, package `task_generator`, effective `use_sim_time=false`; 0.1 s `rclpy.create_timer` uses its node clock (`rclpy/node.py:1505-1510`). Each callback stamps the request with `node.get_clock().now()`. No fixed `dt=0.1` is passed. |
| HuNav service | `hunav_agent_manager/src/bt_node.cpp:284-342` in local upstream-style source; repository copy has the same stamp-delta logic | Node `/task_generator_node/hunav_agent_manager`, package `hunav_agent_manager`; launch config `use_sim_time=true`, effective value not captured. No compute timer. `dt = current_agents.header.stamp - prev_time_`; negative dt is clamped to zero on `ComputeAgents`. |
| SFM position | `hunav_agent_manager/src/bt_functions.cpp:103-120`, `agent_manager.cpp:830-871`; `[LOCAL_PATH]:562-576` | `dt` flows from BT to LightSFM; velocity is capped at desired velocity, then `position += velocity * dt`. LightSFM samples no clock here. |
| HuNav outputs | `hunav_agent_manager/src/bt_node.cpp:318-339,476-505` | `human_states` republishes incoming Agents unchanged before integration, including its request stamp; `/people` is stamped from that same request. Both observed at about 10 Hz wall. |
| Isaac apply | Repository `task_generator/simulators/human/hunav/hunav.py:426-469`, `simulators/sim/isaac_simulator.py:136-182`; installed Isaac `isaac_utils/services/MovePrim.py:13-35` | Each service response leads to one `/isaac/move_prim` per person; Isaac writes the returned pose and Task Generator reads it back. No independent Isaac time step is used for the pose. |
| Isaac `/clock` | Installed Isaac `isaac_utils/graphs/time.py:9-19`; `run_isaacsim.py` creates the time publisher | `OnPlaybackTick` publishes `IsaacReadSimulationTime` via `ROS2PublishClock`. Measured simulated elapsed time and frequency are above. |

## Competing hypotheses

| Hypothesis | Evidence for | Evidence against / limit | Discriminating observation |
|---|---|---|---|
| H1: HuNav position integration uses wall time | Movement scales almost exactly with epoch-like request-stamp deltas. | HuNav does not call a wall clock directly on this path; it consumes the caller's stamp. | Supply known simulated stamp sequence to isolated `ComputeAgents`, vary wall delay, verify displacement follows stamp delta. |
| H2: fixed 10 Hz wall timer with assumed `dt=0.1` | Task Generator is approximately 10 Hz wall in this run. | HuNav computes `dt` from stamps, not literal 0.1. Jittered position increments match jittered **preceding** stamp intervals. | Vary request intervals at fixed stamps and then vary stamps at fixed call intervals. |
| H3: correct HuNav integration, but Isaac applies wall updates during slow simulation | Isaac does apply every returned pose, and RTF is ~0.49. | HuNav integration is already wrong in simulation-time terms before Isaac apply: response/logged `output_xy` has wall-time-derived displacement, and Isaac merely copies it. | Compare HuNav response XY and Isaac read-back against `/clock` in same isolated probe. |
| H4: stamps wall-based, internal dynamics simulation-time correct | Stamps are epoch-like wall time. | HuNav source explicitly derives `dt` from those stamps; stable 5.370 m / 8.933 sim s = 0.601 m/s. | Compare controlled service response increments to explicit stamp deltas. |
| H5: `/clock` publication or RTF interpretation alone causes apparent discrepancy | Isaac publishes repeated `/clock` values; RTF<1. | Interpolated and previous-sample `/clock` estimates agree on ~0.61 m/s, while wall/stamp speed is ~0.30. `/clock` is sourced from Isaac simulation time. Duplicates do not explain the 2x elapsed-time ratio. | Cross-check Isaac physics-step counter and `/clock` in a small isolated run; retain expected speed per simulated second. |

## Limits

The bag does not contain `ComputeAgents` service payloads or a separate Isaac pedestrian truth topic. The Task Generator log's input/output/Isaac read-back records bridge that gap for this run; an instrumented isolated test should retain raw request/response stamps and actual Isaac pose. The effective HuNav manager `use_sim_time` value and binary-to-source hash mapping were not captured, although service behavior matches the read source and the observed stamp/dt/position relationship. This report recommends a minimal change but does not implement or validate it.
