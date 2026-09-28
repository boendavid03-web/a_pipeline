import os
import torch
import numpy as np
import logging
from collections import deque
from omegaconf import OmegaConf

# Import the previous ModelRunner (contains normalization logic and model structure)
from .model_runner2 import ModelRunner

logger = logging.getLogger(__name__)

class LidarVLAInference:
    """
    LiDAR VLA Inference Engine (Flow Matching)
    Features:
    1. Dynamically load config files and normalization parameters
    2. Provide Flow Matching ODE Solver for action sampling
    3. Maintain action execution queue (Receding Horizon Control)
    """
    def __init__(
        self, 
        config_path: str, 
        checkpoint_path: str, 
        norm_stats_path: str,
        device: str = "cuda",
        dtype: torch.dtype = torch.bfloat16,
        num_inference_steps: int = 5,
        action_execution_horizon: int = 8  # Predict 16 steps each time, but only execute the first 8 steps
    ):
        self.device = device
        self.dtype = dtype
        self.num_inference_steps = num_inference_steps
        self.action_execution_horizon = action_execution_horizon
        
        self.action_queue = deque()
        
        # 1. Load config
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config not found: {config_path}")
        self.config = OmegaConf.load(config_path)
        
        self.action_dim = self.config.common.action_dim
        self.action_chunk_size = self.config.common.action_chunk_size
        
        # 2. Initialize model and load weights
        logger.info(f"Loading Model from {checkpoint_path}...")
        self.runner = ModelRunner(self.config, norm_stats_path, device, dtype)
        
        checkpoint = torch.load(checkpoint_path, map_location='cpu', weights_only=False)
        state_dict = checkpoint.get('model_state_dict', checkpoint)
        
        self.runner.model.load_state_dict(state_dict, strict=True)
        self.runner.eval()
        self.runner.to(device)
        logger.info("✅ Inference Engine Ready.")

    def reset(self):
        """Reset the action queue"""
        self.action_queue.clear()

    @torch.no_grad()
    def predict_chunk(
        self, 
        scan_01_np: np.ndarray, 
        scan_02_np: np.ndarray,
        current_tf_np: np.ndarray = None,
        nav_goal_np: np.ndarray = None
    ) -> np.ndarray:
        """
        Core inference module: Use Flow Matching Euler method for action generation
        :param scan_01_np: shape [1, L] (Main LiDAR data)
        :param scan_02_np: shape [1, L] (Secondary LiDAR data)
        :param current_tf_np: Current robot absolute TF [3] (Optional)
        :param nav_goal_np: Navigation goal features [3] (Optional)
        :return: Predicted Action Chunk, shape [chunk_size, 2]
        """
        # 1. Convert to Tensor and add Batch and Time dimensions -> [1, 1, L]
        # (Since it was reshaped to (1, -1) externally, the original shape is [1, L]. After unsqueeze, it becomes [B=1, T=1, L])
        scan_01_t = torch.from_numpy(scan_01_np).to(self.device, self.dtype).unsqueeze(0)
        scan_02_t = torch.from_numpy(scan_02_np).to(self.device, self.dtype).unsqueeze(0)
        
        # Convert TF and goal to Tensor and add Batch dimension -> [1, 3]
        current_tf_t = None
        if current_tf_np is not None:
            current_tf_t = torch.from_numpy(current_tf_np).to(self.device, self.dtype).unsqueeze(0)
            
        nav_goal_t = None
        if nav_goal_np is not None:
            nav_goal_t = torch.from_numpy(nav_goal_np).to(self.device, self.dtype).unsqueeze(0)

        # 2. Normalize both LiDARs separately -> [1, 1, 1, L] (i.e., [B, C, T, L])
        lidar_feat_01 = self.runner.normalize_lidar(scan_01_t)
        lidar_feat_02 = self.runner.normalize_lidar(scan_02_t)
        
        # ==========================================================
        # 3. Flow Matching ODE Solver (Euler Method)
        # ==========================================================
        B = 1
        # Initialize pure noise x_0 ~ N(0, I)
        x_t = torch.randn((B, self.action_chunk_size, self.action_dim), device=self.device, dtype=self.dtype)
        
        # Time step division t: 0 -> 1
        dt = 1.0 / self.num_inference_steps
        
        for i in range(self.num_inference_steps):
            # Construct the current time step scalar
            t_val = i * dt
            t_tensor = torch.full((B,), t_val, device=self.device, dtype=self.dtype)
            
            # Predict Vector Field (velocity v)
            # [Core modification] Pass dual LiDAR features independently to the network
            pred_v = self.runner.model(
                t=t_tensor, 
                noisy_actions=x_t, 
                lidar_feat_01=lidar_feat_01,
                lidar_feat_02=lidar_feat_02,
                current_tf=current_tf_t,
                nav_goal=nav_goal_t
            )
            
            # Euler step: x_{t+dt} = x_t + v * dt
            x_t = x_t + pred_v * dt
            
        # 4. Denormalize to get physical velocity
        action_seq = self.runner.denormalize_action(x_t) # [1, chunk_size, action_dim]
        
        return action_seq[0].float().cpu().numpy()

    def step(self, scan_01_np: np.ndarray, scan_02_np: np.ndarray, current_tf_np: np.ndarray = None, nav_goal_np: np.ndarray = None) -> np.ndarray:
        """Public interface: Receding Horizon Control (RHC) step"""
        if len(self.action_queue) == 0:
            full_chunk = self.predict_chunk(scan_01_np, scan_02_np, current_tf_np, nav_goal_np)
            valid_actions = full_chunk[:self.action_execution_horizon]
            for act in valid_actions:
                self.action_queue.append(act)
                
        return self.action_queue.popleft()
