import math
import torch
import torch.nn as nn
import torch.nn.functional as F

def get_1d_sincos_pos_embed(embed_dim, length):
    if embed_dim % 2 != 0:
        raise ValueError("Embed dim must be divisible by 2")
    pos = torch.arange(length, dtype=torch.float32)
    grid = torch.arange(embed_dim // 2, dtype=torch.float32)
    omega = 1.0 / (10000 ** (grid / (embed_dim // 2)))
    out = torch.einsum('m,d->md', pos, omega)
    emb = torch.cat([torch.sin(out), torch.cos(out)], dim=1)
    return emb.unsqueeze(0)

class SinusoidalPosEmb(nn.Module):
    def __init__(self, dim):
        super().__init__()
        self.dim = dim
    def forward(self, x):
        device = x.device
        half_dim = self.dim // 2
        emb = math.log(10000) / (half_dim - 1)
        emb = torch.exp(torch.arange(half_dim, device=device) * -emb)
        emb = emb.to(dtype=x.dtype)
        emb = x[:, None] * emb[None, :]
        return torch.cat((emb.sin(), emb.cos()), dim=-1)

def modulate(x, shift, scale):
    return x * (1 + scale.unsqueeze(1)) + shift.unsqueeze(1)

class PerceptionBlock(nn.Module):
    def __init__(self, hidden_size, num_heads, mlp_ratio=4.0, dropout=0.):
        super().__init__()
        self.norm1 = nn.LayerNorm(hidden_size)
        self.attn = nn.MultiheadAttention(hidden_size, num_heads, dropout=dropout, batch_first=True)
        self.norm2 = nn.LayerNorm(hidden_size)
        mlp_hidden = int(hidden_size * mlp_ratio)
        self.mlp = nn.Sequential(
            nn.Linear(hidden_size, mlp_hidden),
            nn.GELU(),
            nn.Linear(mlp_hidden, hidden_size)
        )

    def forward(self, x):
        x_norm = self.norm1(x)
        x = x + self.attn(x_norm, x_norm, x_norm)[0]
        x = x + self.mlp(self.norm2(x))
        return x

class DiTBlock(nn.Module):
    def __init__(self, hidden_size, num_heads, mlp_ratio=4.0, dropout=0.):
        super().__init__()
        self.norm1 = nn.LayerNorm(hidden_size, elementwise_affine=False)
        self.attn = nn.MultiheadAttention(hidden_size, num_heads, dropout=dropout, batch_first=True)
        self.norm2 = nn.LayerNorm(hidden_size, elementwise_affine=False)
        mlp_hidden = int(hidden_size * mlp_ratio)
        self.mlp = nn.Sequential(
            nn.Linear(hidden_size, mlp_hidden),
            nn.GELU(),
            nn.Linear(mlp_hidden, hidden_size)
        )
        self.adaLN_modulation = nn.Sequential(
            nn.SiLU(),
            nn.Linear(hidden_size, 6 * hidden_size, bias=True)
        )
        nn.init.constant_(self.adaLN_modulation[-1].weight, 0)
        nn.init.constant_(self.adaLN_modulation[-1].bias, 0)

    def forward(self, x, emb_t):
        (shift_msa, scale_msa, gate_msa, shift_mlp, scale_mlp, gate_mlp) = \
            self.adaLN_modulation(emb_t).chunk(6, dim=1)

        x_norm = modulate(self.norm1(x), shift_msa, scale_msa)
        x = x + gate_msa.unsqueeze(1) * self.attn(x_norm, x_norm, x_norm)[0]

        x_norm = modulate(self.norm2(x), shift_mlp, scale_mlp)
        x = x + gate_mlp.unsqueeze(1) * self.mlp(x_norm)
        return x

class LidarVLAModel(nn.Module):
    def __init__(self, config):
        super().__init__()
        self.action_dim = config.common.action_dim
        self.action_chunk_size = config.common.action_chunk_size
        self.hidden_dim = config.model.hidden_dim
        self.obs_steps = config.common.get('obs_steps', 1)  
        
        # --- 1. Dual LiDAR Shared Encoder (Siamese Encoder) ---
        perc_cfg = config.model.perception
        self.lidar_stem = nn.Conv1d(
            in_channels=1, 
            out_channels=self.hidden_dim, 
            kernel_size=perc_cfg.conv_kernel, 
            stride=perc_cfg.conv_stride
        )
        
        # 🌟 Extra large safety buffer to completely resolve dimension mismatch errors
        safe_calc_len = 2048 
        self.register_buffer('lidar_pos_emb', get_1d_sincos_pos_embed(self.hidden_dim, safe_calc_len))
        
        self.temporal_pos_emb = nn.Parameter(torch.zeros(1, self.obs_steps, 1, self.hidden_dim))
        nn.init.normal_(self.temporal_pos_emb, std=0.02)
        
        self.num_cls = perc_cfg.num_cls_tokens
        self.cls_tokens = nn.Parameter(torch.randn(1, self.num_cls, self.hidden_dim))
        nn.init.normal_(self.cls_tokens, std=0.02)
        
        self.perception_blocks = nn.ModuleList([
            PerceptionBlock(self.hidden_dim, perc_cfg.num_heads, dropout=perc_cfg.dropout) 
            for _ in range(perc_cfg.depth)
        ])
        
        self.use_tf = config.common.get('use_tf_pose', False)
        self.use_goal = config.common.get('use_nav_goal', False)
        
        if self.use_tf:
            self.tf_proj = nn.Sequential(nn.Linear(3, self.hidden_dim), nn.SiLU(), nn.Linear(self.hidden_dim, self.hidden_dim))
        if self.use_goal:
            self.goal_proj = nn.Sequential(nn.Linear(3, self.hidden_dim), nn.SiLU(), nn.Linear(self.hidden_dim, self.hidden_dim))

        # --- 2. Action Prediction Head (Diffusion Transformer Architecture) ---
        act_cfg = config.model.action_head
        self.time_mlp = nn.Sequential(
            SinusoidalPosEmb(self.hidden_dim),
            nn.Linear(self.hidden_dim, self.hidden_dim * 2),
            nn.SiLU(),
            nn.Linear(self.hidden_dim * 2, self.hidden_dim),
        )
        self.action_proj = nn.Linear(self.action_dim, self.hidden_dim)
        self.register_buffer('action_pos_emb', get_1d_sincos_pos_embed(self.hidden_dim, self.action_chunk_size))
        
        self.action_blocks = nn.ModuleList([
            DiTBlock(self.hidden_dim, act_cfg.num_heads, dropout=act_cfg.dropout) for _ in range(act_cfg.depth)
        ])
        self.final_norm = nn.LayerNorm(self.hidden_dim)
        self.output_proj = nn.Linear(self.hidden_dim, self.action_dim)

    def encode_lidar(self, lidar_features):
        B, C, T_steps, L = lidar_features.shape
        x_lidar = lidar_features.permute(0, 2, 1, 3).reshape(B * T_steps, C, L)
        x_lidar = self.lidar_stem(x_lidar)
        L_out = x_lidar.shape[-1]
        
        x_lidar = x_lidar.permute(0, 2, 1).view(B, T_steps, L_out, self.hidden_dim) 
        
        # Dynamically slice to extract the required positional embeddings
        x_lidar = x_lidar + self.lidar_pos_emb[:, :L_out, :].unsqueeze(1)
        x_lidar = x_lidar + self.temporal_pos_emb[:, :T_steps, :, :]
        x_lidar = x_lidar.view(B, T_steps * L_out, self.hidden_dim)
        
        cls_tokens = self.cls_tokens.expand(B, -1, -1)
        x_perc = torch.cat([cls_tokens, x_lidar], dim=1)
        for block in self.perception_blocks:
            x_perc = block(x_perc)
        return x_perc[:, :self.num_cls, :]

    def forward(self, t, noisy_actions, lidar_feat_01, lidar_feat_02, current_tf=None, nav_goal=None):
        cond_01 = self.encode_lidar(lidar_feat_01)
        cond_02 = self.encode_lidar(lidar_feat_02)
        
        extra_tokens = [cond_01, cond_02]
        
        if self.use_tf and current_tf is not None:
            tf_emb = self.tf_proj(current_tf).unsqueeze(1)
            extra_tokens.append(tf_emb)
        if self.use_goal and nav_goal is not None:
            goal_emb = self.goal_proj(nav_goal).unsqueeze(1) 
            extra_tokens.append(goal_emb)
            
        t_emb = self.time_mlp(t)
        x_act = self.action_proj(noisy_actions) + self.action_pos_emb
        
        x = torch.cat([x_act] + extra_tokens, dim=1)
        for block in self.action_blocks:
            x = block(x, t_emb)
            
        x = self.final_norm(x)
        x_out = x[:, :self.action_chunk_size, :]
        return self.output_proj(x_out)

def calc_flow_matching_loss(model, x1, lidar_feat_01, lidar_feat_02, current_tf=None, nav_goal=None, time_sampler="uniform", time_mu=0.0, time_sigma=1.0):
    device = x1.device
    bs = x1.shape[0]
    
    x0 = torch.randn_like(x1)
    if time_sampler == "uniform":
        t = torch.rand(bs, device=device)
    elif time_sampler == "logit_normal":
        normal_samples = torch.randn(bs, device=device) * time_sigma + time_mu
        t = torch.sigmoid(normal_samples)
    else:
        raise ValueError("Unsupported time_sampler")
    
    t_expand = t.view(bs, 1, 1)
    x_t = (1 - t_expand) * x0 + t_expand * x1
    target_v = x1 - x0 
    
    pred_v = model(t, noisy_actions=x_t, lidar_feat_01=lidar_feat_01, lidar_feat_02=lidar_feat_02, current_tf=current_tf, nav_goal=nav_goal)
    return F.mse_loss(pred_v, target_v)
