# nav2py_drl_vo

This repository integrates the ROS 2 Navigation Stack (Nav2) with a Deep Reinforcement Learning-based Velocity Obstacle (DRL-VO) control policy, enabling autonomous navigation through environments with static and dynamic obstacles.

## Installation

1. **Install nav2py and deps**

   ```bash
      git clone https://github.com/voshch/nav2py
      git clone https://github.com/voshch/ament_cmake_venv
      git clone https://github.com/voshch/ament_cmake_venv_uv
   ```
   
2. **Clone the Repository**:

   ```bash
   git clone https://github.com/Arena-Rosnav/nav2py_drlvo

3. Build workspace.

4. **Run** (in this directory):
   ```bash
   ros2 launch nav2_bringup tb4_simulation_launch.py params_file:=nav2_params.yaml headless:=True
   ```