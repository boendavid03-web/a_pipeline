#!/usr/bin/env python3

import rclpy
import rclpy.logging
import sys
print(sys.executable)
from stable_baselines3 import PPO
from . import custom_cnn_full
import numpy as np
sys.modules['custom_cnn_full'] = custom_cnn_full
def main():
    print(sys.executable)
    logger = rclpy.logging.get_logger('controller_server')
    print(logger)
    model = PPO.load('/home/kien/colcon_ws/src/nav2_pyif/drl_vo_controller/model/drl_vo.zip')
    ped_pos = np.zeros((80, 80, 2), dtype=np.float32)
    scan = np.zeros((80, 80), dtype=np.float32)
    goal = np.zeros((2, 1), dtype=np.float32)
    observation = np.concatenate((ped_pos.flatten(), scan.flatten(), goal.flatten()))
    action, _ = model.predict(observation)
    print(observation.shape)
    print("Predicted action:", action)
if __name__ == '__main__':
    main()