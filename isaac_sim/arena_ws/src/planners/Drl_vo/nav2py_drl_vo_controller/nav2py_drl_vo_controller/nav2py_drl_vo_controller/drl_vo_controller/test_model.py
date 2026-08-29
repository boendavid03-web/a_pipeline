#!/usr/bin/env python3

import io

import numpy as np
from stable_baselines3 import PPO


def main():
    # Path to your trained PPO model
    model_path = '/home/kien/colcon_ws/src/nav2_pyif/drl_vo_controller/model/drl_vo.zip'

    # Load the PPO model
    model = PPO.load(model_path)
    print("Model loaded successfully.")

    # Create a dummy observation:
    # The observation is a concatenation of:
    # 1. Pedestrian map: shape (80, 80, 2) → 80*80*2 = 12800 elements
    # 2. Lidar scan: shape (80, 80) → 80*80 = 6400 elements
    # 3. Goal position: shape (2, 1) → 2 elements
    ped_pos = np.zeros((80, 80, 2), dtype=np.float32)
    scan = np.zeros((80, 80), dtype=np.float32)
    goal = np.zeros((2, 1), dtype=np.float32)

    # Flatten and concatenate to build the observation vector
    observation = np.concatenate((ped_pos.flatten(), scan.flatten(), goal.flatten()))
    print("Observation shape:", observation.shape)

    # Use the model to predict an action from the observation
    action, _ = model.predict(observation)
    print("Predicted action:", action)


if __name__ == '__main__':
    main()
