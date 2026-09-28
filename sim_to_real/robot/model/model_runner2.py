import json
import torch
import torch.nn as nn
import logging
from .model_dit_fm2 import LidarVLAModel, calc_flow_matching_loss

logger = logging.getLogger(__name__)

class ModelRunner(nn.Module):
    """Unified management of LiDAR VLA model and data normalization"""
    def __init__(self, config, norm_stats_path, device, dtype):
        super().__init__()
        self.device = device
        self.dtype = dtype
        self.config = config
        
        logger.info("Initializing Lidar VLA Model...")
        self.model = LidarVLAModel(config)
        self.model.to(device, dtype=dtype)
        
        self.load_norm_stats(norm_stats_path)

    def load_norm_stats(self, path):
        try:
            with open(path, 'r') as f:
                stats = json.load(f)
                
            scan_max = torch.tensor(stats["lidar_range"]["norm_threshold_95"], dtype=torch.float32)
            vx_max = torch.tensor(stats["velocity_linear_x"]["norm_bound_99"], dtype=torch.float32)
            vw_max = torch.tensor(stats["velocity_angular_w"]["norm_bound_99"], dtype=torch.float32)
            act_max = torch.stack([vx_max, vw_max])
            
            # 🌟 Fix: Prevent division by zero caused by zero statistic values
            scan_max = torch.clamp(scan_max, min=1e-3)
            vx_max = torch.clamp(vx_max, min=1e-3)
            vw_max = torch.clamp(vw_max, min=1e-3)
            
            act_max = torch.stack([vx_max, vw_max])

            self.register_buffer('scan_max', scan_max)
            self.register_buffer('action_max', act_max)
            logger.info("Normalization stats successfully loaded.")
        except Exception as e:
            logger.error(f"Failed to load norm stats: {e}")
            raise e

    def normalize_lidar(self, scan_raw):
        """
        Raw scan shape: [Batch, Time, Length]
        Normalize LiDAR data to [0, 1] and add a channel dimension to fit 1D convolution requirements
        """
        scan_safe = torch.where(scan_raw < 0.05, self.scan_max, scan_raw)
        scan_norm = torch.clamp(scan_safe, 0.0, self.scan_max.item()) / self.scan_max
        # Add channel dimension -> [Batch, 1, Time, Length]
        return scan_norm.unsqueeze(1)

    def normalize_action(self, action_raw):
        max_v = self.action_max.to(action_raw.device, action_raw.dtype)
        return torch.clamp(action_raw, -max_v, max_v) / max_v

    def denormalize_action(self, action_norm):
        max_v = self.action_max.to(action_norm.device, action_norm.dtype)
        return action_norm * max_v

    def forward(self, batch):
        act_raw = batch['action'].to(self.device, self.dtype)
        
        # [Batch, 1081] -> Add dimension to become [Batch, 1, 1081]
        scan_01 = batch['scan_01'].to(self.device, self.dtype).unsqueeze(1)
        scan_02 = batch['scan_02'].to(self.device, self.dtype).unsqueeze(1)
        
        current_tf = batch.get('current_tf')
        if current_tf is not None:
            current_tf = current_tf.to(self.device, self.dtype)
            
        nav_goal = batch.get('nav_goal')
        if nav_goal is not None:
            nav_goal = nav_goal.to(self.device, self.dtype)
            
        # ================= 3. Data Normalization =================
        x1 = self.normalize_action(act_raw)
        
        # Normalize front and rear LiDARs separately -> [Batch, 1, 1, 1081]
        lidar_feat_01 = self.normalize_lidar(scan_01) 
        lidar_feat_02 = self.normalize_lidar(scan_02) 
        
        # ================= 4. Calculate Loss =================
        loss = calc_flow_matching_loss(
            model=self.model,
            x1=x1,
            lidar_feat_01=lidar_feat_01,  # Pass dual LiDAR features separately
            lidar_feat_02=lidar_feat_02,
            current_tf=current_tf,
            nav_goal=nav_goal,
            time_sampler=self.config.training.time_sampler,
            time_mu=self.config.training.time_mu,
            time_sigma=self.config.training.time_sigma
        )
        
        return loss
