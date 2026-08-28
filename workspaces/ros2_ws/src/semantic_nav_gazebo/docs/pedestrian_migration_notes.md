# Pedestrian Migration Notes

The ROS 1 launch chain for `semantic_cnn_nav_gazebo.launch` uses:

- `pedsim_simulator/launch/robot.launch` to start Gazebo Classic, spawn the robot, start `pedsim_simulator`, and publish the pedsim visualizer.
- `pedsim_gazebo_plugin/scripts/spawn_pedsim_agents.py` to wait for the first `/pedsim_simulator/simulated_agents` message, spawn one SDF model per agent through `/gazebo/spawn_sdf_model`, then exit.
- `pedsim_gazebo_plugin/src/actor_poses_plugin.cpp` as a Gazebo Classic world plugin. It subscribes to `/pedsim_simulator/simulated_agents` and updates each spawned model pose by matching the Gazebo model name with the pedsim agent id.
- `pedsim_simulator/scenarios/lobby/eng_hall_15.xml` as the default scenario. The XML defines static obstacles, waypoints, queues, and agent clusters.

For ROS 2 + Gazebo Sim, the clean equivalent is:

- Port or replace the pedsim simulator so it publishes ROS 2 agent states.
- Spawn each pedestrian in Gazebo Sim with `ros_gz_sim create` or the Gazebo Sim entity factory service instead of ROS 1 `gazebo_msgs/SpawnModel`.
- Replace `ActorPosesPlugin` with either a Gazebo Sim system plugin or a ROS 2 node that bridges agent states into Gazebo Sim pose/entity updates.
- Keep the Gazebo world independent from the robot URDF. The robot can be spawned later from a separate ROS 2 launch file.

The migrated worlds in this package intentionally remove the ROS 1 `ActorPosesPlugin` reference so the static maps can launch in Gazebo Sim before the pedestrian port is implemented.

## Current ROS 2 Spawn Smoke Test

`semantic_cnn_nav_gazebo.launch.py` has a `spawn_scene_pedestrians` argument. When enabled, it starts `scenario_pedestrian_controller.py`, waits a few seconds after Gazebo Sim starts, parses the configured scenario XML, spawns one primitive `simple_pedestrian` model for each non-robot agent, then moves all pedestrians with Gazebo Sim's `/world/default/set_pose_vector` service.

Example:

```bash
ros2 launch semantic_nav_gazebo semantic_cnn_nav_gazebo.launch.py \
  gui:=false \
  spawn_scene_pedestrians:=true
```

The default scene is `scenarios/lobby/eng_hall_15.xml`. It is parsed like the ROS 1 `ScenarioReader`: `waypoint` elements are indexed by id, each `agent` cluster is expanded using `x`, `y`, `n`, `dx`, `dy`, and `type`, `type=2` robot agents are skipped, and each pedestrian cycles through the cluster's `addwaypoint` targets in XML order.

This validates the Gazebo Sim entity creation and XML-driven pose update path. It does not yet reproduce pedsim's full social-force interaction model; the controller currently uses straight-line waypoint following and waypoint-radius switching.
