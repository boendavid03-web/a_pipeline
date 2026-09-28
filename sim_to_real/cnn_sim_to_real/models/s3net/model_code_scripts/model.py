#!/usr/bin/env python
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/angles_lidar/, /intensities_lidar/, /scans_lidar/, /semantic_label/, /source_sensor/, /valid_mask_lidar/
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：NPY, TXT
# 可能使用的关键环境变量：BASELINE_ANGLE_MAX, BASELINE_ANGLE_MIN, DEFAULT_NORMALIZATION_STATS, FEATURE_MODES, IGNORE_LABEL, NEW_LINE, POINTS, S3NET_IGNORE_CLASS_IDS, S3NET_STATS_JSON, SEED1
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/model.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.734305629 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:18:51.815067236 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/render_semantic_cnn_offline_demo.py（导入其函数、类或模型）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/smoke_fixed_dual_training.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（ros2 launch 启动该场景）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/s3net_fixed_dual_inference_node.py（导入其函数、类或模型）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_fixed_dual_inference_node.py（导入其函数、类或模型）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/model.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/render_semantic_cnn_offline_demo.py; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/smoke_fixed_dual_training.py; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/s3net_fixed_dual_inference_node.py; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_fixed_dual_inference_node.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
#
# file: $ISIP_EXP/SOGMP/scripts/model.py
#
# revision history: xzt
#  20220824 (TE): first version
#
# usage:
#
# This script hold the model architecture
#------------------------------------------------------------------------------

# import pytorch modules
#
from __future__ import print_function
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from collections import OrderedDict

# import modules
#
import json
import os
import random

# for reproducibility, we seed the rng
#
SEED1 = 1337
NEW_LINE = "\n"

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

#-----------------------------------------------------------------------------
#
# helper functions are listed here
#
#-----------------------------------------------------------------------------

# function: set_seed
#
# arguments: seed - the seed for all the rng
#
# returns: none
#
# this method seeds all the random number generators and makes
# the results deterministic
#
def set_seed(seed):
    #torch.manual_seed(seed)
    #torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    #random.seed(seed)
    #os.environ['PYTHONHASHSEED'] = str(seed)
#
# end of method

# calculate the angle of incidence of the lidar ray:
def angle_incidence_calculation(b, c, alpha, last_ray=False):
    '''
    # remove invalid values:
    if(last_ray): # the last ray
        if(np.isnan(b) or np.isinf(b)):
            b = 60.
        if(np.isnan(c) or np.isinf(c)):
            c = 60.
    else:
        b[np.isnan(b)] = 60.
        b[np.isinf(b)] = 60.
        c[np.isnan(c)] = 60.
        c[np.isinf(c)] = 60.
    '''
    # the law of cosines:
    a = np.sqrt(b*b + c*c - 2*b*c*np.cos(alpha))
    if(last_ray): # the last ray
        with np.errstate(invalid='ignore', divide='ignore'):
            cos_beta = (a*a + c*c - b*b)/(2*a*c)
            cos_beta = np.clip(cos_beta, -1.0, 1.0)
            beta = np.arccos([cos_beta])
        theta = np.abs(np.pi/2 - beta)
    else:
        with np.errstate(invalid='ignore', divide='ignore'):
            cos_gamma = (a*a + b*b - c*c)/(2*a*b)
            cos_gamma = np.clip(cos_gamma, -1.0, 1.0)
            gamma = np.arccos([cos_gamma])
        theta = np.abs(np.pi/2 - gamma)

    return theta

# function: get_data
#
# arguments: fp - file pointer
#            num_feats - the number of features in a sample
#
# returns: data - the signals/features
#          labels - the correct labels for them
#
# this method takes in a fp and returns the data and labels
POINTS = 1081
IGNORE_LABEL = -1
BASELINE_ANGLE_MIN = -2.356194496154785
BASELINE_ANGLE_MAX = 2.356194496154785
FEATURE_MODES = {
    "range_only": 1,
    "range_incidence": 2,
    "range_intensity_incidence": 3,
}

DEFAULT_NORMALIZATION_STATS = {
    "scan": {"mean": 4.518406, "std": 8.2914915},
    "intensity": {"mean": 3081.8167, "std": 1529.4413},
    "angle_incidence": {"mean": 0.5959513, "std": 0.4783924},
}


def parse_class_ids(value):
    if value is None:
        return []
    ids = []
    for item in str(value).split(","):
        item = item.strip()
        if item:
            ids.append(int(item))
    return ids


def safe_std(value, floor=1e-6):
    return max(float(value), floor)


def load_normalization_stats(stats_path=None):
    stats = DEFAULT_NORMALIZATION_STATS.copy()
    stats_path = stats_path or os.environ.get("S3NET_STATS_JSON")
    if not stats_path:
        return stats

    with open(stats_path, "r") as fp:
        loaded = json.load(fp)
    loaded = loaded.get("normalization", loaded)

    for key in ("scan", "intensity", "angle_incidence"):
        if key in loaded:
            stats[key] = {
                "mean": float(loaded[key].get("mean", stats[key]["mean"])),
                "std": safe_std(loaded[key].get("std", stats[key]["std"])),
            }
    return stats


def load_ignored_class_ids(stats_path=None):
    env_ids = parse_class_ids(os.environ.get("S3NET_IGNORE_CLASS_IDS"))
    if env_ids:
        return env_ids
    stats_path = stats_path or os.environ.get("S3NET_STATS_JSON")
    if not stats_path:
        return []

    with open(stats_path, "r") as fp:
        loaded = json.load(fp)
    return parse_class_ids(",".join(str(v) for v in loaded.get("ignored_class_ids", [])))


def baseline_lidar_angles(num_points):
    return np.linspace(BASELINE_ANGLE_MIN, BASELINE_ANGLE_MAX, num=num_points, dtype=np.float32)


def feature_mode_num_channels(feature_mode):
    if feature_mode not in FEATURE_MODES:
        raise ValueError(
            "S3-Net feature mode must be one of {}, got {!r}".format(
                sorted(FEATURE_MODES), feature_mode
            )
        )
    return FEATURE_MODES[feature_mode]


def feature_mode_from_channels(input_channels):
    matches = [name for name, channels in FEATURE_MODES.items() if channels == input_channels]
    if not matches:
        raise ValueError("unsupported S3-Net input channel count: {}".format(input_channels))
    return matches[0]


def angle_incidence_from_scan(scan, angles=None, source_sensor=None):
    scan = np.asarray(scan, dtype=np.float32).reshape(-1)
    if source_sensor is not None:
        source_sensor = np.asarray(source_sensor).reshape(-1)
        if source_sensor.shape != scan.shape:
            raise ValueError("source_sensor shape must match scan shape")
        if angles is not None:
            angles = np.asarray(angles, dtype=np.float32).reshape(-1)
            if angles.shape != scan.shape:
                raise ValueError("angles_lidar shape must match scan shape")
        incidence = np.zeros(scan.shape, dtype=np.float32)
        if scan.size == 0:
            return incidence
        boundaries = np.flatnonzero(source_sensor[1:] != source_sensor[:-1]) + 1
        starts = np.concatenate(([0], boundaries))
        ends = np.concatenate((boundaries, [scan.size]))
        for start, end in zip(starts, ends):
            segment_angles = None if angles is None else angles[start:end]
            incidence[start:end] = angle_incidence_from_scan(
                scan[start:end], segment_angles, source_sensor=None
            )
        return incidence
    if scan.size < 2:
        return np.zeros(scan.shape, dtype=np.float32)
    scan = np.nan_to_num(scan, nan=0.0, posinf=0.0, neginf=0.0)

    b = scan[:-1]
    c = scan[1:]
    if angles is None:
        alpha = np.ones(scan.size - 1) * ((270*np.pi / 180) / (scan.size - 1))
        alpha_last = (270*np.pi / 180) / (scan.size - 1)
    else:
        angles = np.asarray(angles, dtype=np.float32).reshape(-1)
        if angles.shape[0] != scan.shape[0]:
            raise ValueError("angles_lidar shape must match scan shape")
        alpha = np.abs(np.diff(angles))
        alpha_last = float(alpha[-1]) if alpha.size else 0.0

    theta = angle_incidence_calculation(b, c, alpha)
    theta_last = angle_incidence_calculation(scan[-2], scan[-1], alpha_last, last_ray=True)
    return np.concatenate((theta[0], theta_last), axis=0).astype(np.float32)


class VaeTestDataset(torch.utils.data.Dataset):
    def __init__(self, img_path, file_name, stats_path=None):
        if not img_path.endswith(os.sep):
            img_path = img_path + os.sep
        # initialize the data and labels
        # read the names of image data:
        self.scan_file_names = []
        self.intensity_file_names = []
        self.angle_file_names = []
        self.valid_mask_file_names = []
        self.source_sensor_file_names = []
        self.sample_sensor_ids = []
        self.native_sample_flags = []
        #self.vel_file_names = []
        self.label_file_names = []
        # parameters: data mean/std for scan, intensity, and angle of incidence.
        stats_path_used = stats_path or os.environ.get("S3NET_STATS_JSON")
        stats = load_normalization_stats(stats_path)
        self.s_mu = stats["scan"]["mean"]
        self.s_std = stats["scan"]["std"]
        self.i_mu = stats["intensity"]["mean"]
        self.i_std = stats["intensity"]["std"]
        self.a_mu = stats["angle_incidence"]["mean"]
        self.a_std = stats["angle_incidence"]["std"]
        self.ignore_class_ids = load_ignored_class_ids(stats_path)
        # open train.txt or dev.txt:
        fp_folder = open(img_path+'dataset.txt','r')
        
        # for each line of the file:
        for folder_line in fp_folder.read().split(NEW_LINE):
            if('-' in folder_line): 
                folder_path = folder_line
                fp_file = open(img_path+folder_path+'/'+file_name+'.txt', 'r')
                for line in fp_file.read().split(NEW_LINE):
                    if('.npy' in line): 
                        sample_root = img_path+folder_path
                        angle_name = sample_root+'/angles_lidar/'+line
                        valid_mask_name = sample_root+'/valid_mask_lidar/'+line
                        is_native = os.path.exists(angle_name) and os.path.exists(valid_mask_name)
                        source_sensor_name = sample_root+'/source_sensor/'+line
                        if os.path.exists(source_sensor_name):
                            source_values = np.load(source_sensor_name).reshape(-1)
                            sensor_ids = sorted(int(value) for value in np.unique(source_values))
                            if not is_native:
                                raise ValueError("source_sensor requires native angles/mask: {}".format(sample_root))
                        else:
                            source_sensor_name = None
                            sensor_ids = [None]
                        for sensor_id in sensor_ids:
                            self.scan_file_names.append(sample_root+'/scans_lidar/'+line)
                            self.intensity_file_names.append(sample_root+'/intensities_lidar/'+line)
                            self.angle_file_names.append(angle_name if is_native else None)
                            self.valid_mask_file_names.append(valid_mask_name if is_native else None)
                            self.source_sensor_file_names.append(source_sensor_name)
                            self.sample_sensor_ids.append(sensor_id)
                            self.native_sample_flags.append(is_native)
                            #self.vel_file_names.append(img_path+folder_path+'/velocities/'+line)
                            self.label_file_names.append(sample_root+'/semantic_label/'+line)
                # close txt file:
                fp_file.close()

        # close txt file:
        fp_folder.close()

        self.length = len(self.scan_file_names)
        self.native_lidar = any(self.native_sample_flags)
        self.native_samples = sum(1 for flag in self.native_sample_flags if flag)
        if self.native_lidar and not stats_path_used:
            self.i_mu = 0.0
            self.i_std = 1.0

        print("dataset length: ", self.length)
        print("native lidar samples: ", self.native_samples)
        if self.ignore_class_ids:
            print("ignoring semantic class ids: ", self.ignore_class_ids)


    def __len__(self):
        return self.length

    def __getitem__(self, idx):
        # get the scan data:
        intensity_name = self.intensity_file_names[idx]
        intensity = np.load(intensity_name).astype(np.float32).reshape(-1)

        # get the scan data:
        scan_name = self.scan_file_names[idx]
        scan = np.load(scan_name).astype(np.float32).reshape(-1)

        # get the semantic label data:
        label_name = self.label_file_names[idx]
        label = np.nan_to_num(np.load(label_name), nan=IGNORE_LABEL, posinf=IGNORE_LABEL, neginf=IGNORE_LABEL)
        label = label.astype(np.int64).reshape(-1)
        if self.ignore_class_ids:
            label[np.isin(label, self.ignore_class_ids)] = IGNORE_LABEL

        if intensity.shape[0] != scan.shape[0]:
            intensity = np.zeros(scan.shape, dtype=np.float32)
        if label.shape[0] != scan.shape[0]:
            raise ValueError("semantic_label shape must match scan shape: {}".format(label_name))

        # Native datasets carry true beam angles and validity. Old 1081 datasets
        # fall back to the historical 270 degree field of view.
        if self.native_sample_flags[idx]:
            angles = np.load(self.angle_file_names[idx]).astype(np.float32).reshape(-1)
            valid_mask = np.load(self.valid_mask_file_names[idx]).astype(np.bool_).reshape(-1)
            if angles.shape[0] != scan.shape[0] or valid_mask.shape[0] != scan.shape[0]:
                raise ValueError("angles_lidar/valid_mask_lidar shape must match scan shape")
        else:
            angles = baseline_lidar_angles(scan.shape[0])
            valid_mask = np.isfinite(scan)

        source_sensor_id = self.sample_sensor_ids[idx]
        if source_sensor_id is not None:
            source_sensor = np.load(self.source_sensor_file_names[idx]).reshape(-1)
            if source_sensor.shape != scan.shape:
                raise ValueError("source_sensor shape must match scan shape")
            sensor_mask = source_sensor == source_sensor_id
            if not np.any(sensor_mask):
                raise ValueError("source_sensor id {} has no beams".format(source_sensor_id))
            scan = scan[sensor_mask]
            intensity = intensity[sensor_mask]
            label = label[sensor_mask]
            angles = angles[sensor_mask]
            valid_mask = valid_mask[sensor_mask]

        # get the angle of incidence of the ray:
        angle_incidence = angle_incidence_from_scan(scan, angles if self.native_sample_flags[idx] else None)

        # initialize:
        valid_mask = valid_mask & np.isfinite(scan) & np.isfinite(intensity) & np.isfinite(angles)
        scan[np.isnan(scan)] = 0.
        scan[np.isinf(scan)] = 0.

        intensity[np.isnan(intensity)] = 0.
        intensity[np.isinf(intensity)] = 0.

        angle_incidence[np.isnan(angle_incidence)] = 0.
        angle_incidence[np.isinf(angle_incidence)] = 0.

        label[np.isnan(label)] = 0.
        label[np.isinf(label)] = 0.
        label[~valid_mask] = IGNORE_LABEL

        scan[~valid_mask] = 0.
        intensity[~valid_mask] = 0.
        angle_incidence[~valid_mask] = 0.

        # data normalization: 
        # standardization: scan
        # mu: 4.518406, std: 8.2914915
        scan = (scan - self.s_mu) / self.s_std

        # standardization: intensity
        # mu: 3081.8167, std: 1529.4413
        intensity = (intensity - self.i_mu) / self.i_std

        # standardization: angle_incidence
        # mu: 0.5959513, std: 0.4783924
        angle_incidence = (angle_incidence - self.a_mu) / self.a_std

        # Invalid beams must be numerically neutral before entering the model.
        scan[~valid_mask] = 0.0
        intensity[~valid_mask] = 0.0
        angle_incidence[~valid_mask] = 0.0
        if not (
            np.all(np.isfinite(scan))
            and np.all(np.isfinite(intensity))
            and np.all(np.isfinite(angle_incidence))
        ):
            raise ValueError("non-finite S3-Net input after invalid-beam cleanup")

        # transfer to pytorch tensor:
        scan_tensor = torch.FloatTensor(scan)
        intensity_tensor = torch.FloatTensor(intensity)
        angle_incidence_tensor = torch.FloatTensor(angle_incidence)
        angles_tensor = torch.FloatTensor(angles)
        valid_mask_tensor = torch.BoolTensor(valid_mask)
        label_tensor =  torch.LongTensor(label)

        data = {
                'scan': scan_tensor,
                'intensity': intensity_tensor,
                'angle_incidence': angle_incidence_tensor, 
                'angles': angles_tensor,
                'valid_mask': valid_mask_tensor,
                'label': label_tensor,
                'source_sensor_id': -1 if source_sensor_id is None else int(source_sensor_id),
                }

        return data

#
# end of function


#------------------------------------------------------------------------------
#
# the model is defined here
#
#------------------------------------------------------------------------------

# define the PyTorch VAE model
#
# define a VAE
# Residual blocks: 
class Residual(nn.Module):
    def __init__(self, in_channels, num_hiddens, num_residual_hiddens):
        super(Residual, self).__init__()
        self._block = nn.Sequential(
            nn.ReLU(False),
            nn.Conv1d(in_channels=in_channels,
                      out_channels=num_residual_hiddens,
                      kernel_size=3, stride=1, padding=1, bias=False),
            nn.BatchNorm1d(num_residual_hiddens),
            nn.ReLU(False),
            nn.Conv1d(in_channels=num_residual_hiddens,
                      out_channels=num_hiddens,
                      kernel_size=1, stride=1, bias=False),
            nn.BatchNorm1d(num_hiddens)
        )
    
    def forward(self, x):
        return x + self._block(x)

class ResidualStack(nn.Module):
    def __init__(self, in_channels, num_hiddens, num_residual_layers, num_residual_hiddens):
        super(ResidualStack, self).__init__()
        self._num_residual_layers = num_residual_layers
        self._layers = nn.ModuleList([Residual(in_channels, num_hiddens, num_residual_hiddens)
                             for _ in range(self._num_residual_layers)])

    def forward(self, x):
        for i in range(self._num_residual_layers):
            x = self._layers[i](x)
        return F.relu(x)

# Encoder & Decoder Architecture:
# Encoder:
class Encoder(nn.Module):
    def __init__(self, in_channels, num_hiddens, num_residual_layers, num_residual_hiddens):
        super(Encoder, self).__init__()
        self._conv_1 = nn.Sequential(*[
                                        nn.Conv1d(in_channels=in_channels,
                                                  out_channels=num_hiddens//2,
                                                  kernel_size=4,
                                                  stride=2, 
                                                  padding=1),
                                        nn.BatchNorm1d(num_hiddens//2),
                                        nn.ReLU(False)
                                    ])
        self._conv_2 = nn.Sequential(*[
                                        nn.Conv1d(in_channels=num_hiddens//2,
                                                  out_channels=num_hiddens,
                                                  kernel_size=4,
                                                  stride=2, 
                                                  padding=1),
                                        nn.BatchNorm1d(num_hiddens)
                                        #nn.ReLU(True)
                                    ])
        self._residual_stack = ResidualStack(in_channels=num_hiddens,
                                             num_hiddens=num_hiddens,
                                             num_residual_layers=num_residual_layers,
                                             num_residual_hiddens=num_residual_hiddens)

    def forward(self, inputs):
        x = self._conv_1(inputs)
        x = self._conv_2(x)
        x = self._residual_stack(x)
        return x

# Decoder:
class Decoder(nn.Module):
    def __init__(self, out_channels, num_hiddens, num_residual_layers, num_residual_hiddens):
        super(Decoder, self).__init__()
        
        self._residual_stack = ResidualStack(in_channels=num_hiddens,
                                             num_hiddens=num_hiddens,
                                             num_residual_layers=num_residual_layers,
                                             num_residual_hiddens=num_residual_hiddens)

        self._conv_trans_2 = nn.Sequential(*[
                                            nn.ReLU(False),
                                            nn.ConvTranspose1d(in_channels=num_hiddens,
                                                              out_channels=num_hiddens//2,
                                                              kernel_size=4,
                                                              stride=2,
                                                              padding=1),
                                            nn.BatchNorm1d(num_hiddens//2),
                                            nn.ReLU(False)
                                        ])

        self._conv_trans_1 = nn.Sequential(*[
                                            nn.ConvTranspose1d(in_channels=num_hiddens//2,
                                                              out_channels=num_hiddens//2,
                                                              kernel_size=4,
                                                              stride=2,
                                                              padding=1,
                                                              output_padding=1),
                                            nn.BatchNorm1d(num_hiddens//2),
                                            nn.ReLU(False),                  
                                            nn.Conv1d(in_channels=num_hiddens//2,
                                                      out_channels=out_channels,
                                                      kernel_size=3,
                                                      stride=1,
                                                      padding=1),
                                            #nn.Sigmoid()
                                        ])

    def forward(self, inputs):
        x = self._residual_stack(inputs)
        x = self._conv_trans_2(x)
        x = self._conv_trans_1(x)
        return x

class VAE_Encoder(nn.Module):
    def __init__(self, input_channel, num_hiddens, num_residual_layers, num_residual_hiddens, embedding_dim):
        super(VAE_Encoder, self).__init__()
        # parameters:
        self.input_channels = input_channel
        '''
        # Constants
        num_hiddens = 128 #128
        num_residual_hiddens = 64 #32
        num_residual_layers = 2
        embedding_dim = 2 #64
        '''

        # encoder:
        in_channels = input_channel
        self._encoder = Encoder(in_channels, 
                                num_hiddens,
                                num_residual_layers, 
                                num_residual_hiddens)

        # z latent variable: 
        self._encoder_z_mu = nn.Conv1d(in_channels=num_hiddens, 
                                    out_channels=embedding_dim,
                                    kernel_size=1, 
                                    stride=1)
        self._encoder_z_log_sd = nn.Conv1d(in_channels=num_hiddens, 
                                    out_channels=embedding_dim,
                                    kernel_size=1, 
                                    stride=1)  
        
    def forward(self, x):
        # input reshape:
        num_points = x.shape[-1]
        x = x.reshape(-1, self.input_channels, num_points)
        # Encoder:
        encoder_out = self._encoder(x)
        # get `mu` and `log_var`:
        z_mu = self._encoder_z_mu(encoder_out)
        z_log_sd = self._encoder_z_log_sd(encoder_out)
        return z_mu, z_log_sd

# our proposed model:
class S3Net(nn.Module):
    def __init__(self, input_channels, output_channels, feature_mode=None):
        super(S3Net, self).__init__()
        # parameters:
        self.input_channels = input_channels
        self.output_channels = output_channels
        self.feature_mode = feature_mode or feature_mode_from_channels(input_channels)
        expected_channels = feature_mode_num_channels(self.feature_mode)
        if expected_channels != input_channels:
            raise ValueError(
                "feature mode {} requires {} channels, got {}".format(
                    self.feature_mode, expected_channels, input_channels
                )
            )

        # Constants
        num_hiddens = 64 #128 
        num_residual_hiddens = 32 #64 
        num_residual_layers = 2
        embedding_dim = 1 #2 
    
        # prediction encoder:
        self._encoder = VAE_Encoder(self.input_channels, 
                                    num_hiddens, 
                                    num_residual_layers, 
                                    num_residual_hiddens, 
                                    embedding_dim)

        # decoder:
        self._decoder_z_mu = nn.ConvTranspose1d(in_channels=embedding_dim, 
                                    out_channels=num_hiddens,
                                    kernel_size=1, 
                                    stride=1)
        self._decoder = Decoder(self.output_channels,
                                num_hiddens, 
                                num_residual_layers, 
                                num_residual_hiddens)

        self.softmax = nn.Softmax(dim=1)

        

    def vae_reparameterize(self, z_mu, z_log_sd):
        """
        :param mu: mean from the encoder's latent space
        :param log_sd: log standard deviation from the encoder's latent space
        :output: reparameterized latent variable z, Monte carlo KL divergence
        """
        # reshape dynamic lidar length latent sequence to [batch, length, channel].
        batch_size = z_mu.size(0)
        z_mu = z_mu.reshape(batch_size, -1, 1)
        z_log_sd = z_log_sd.reshape(batch_size, -1, 1)
        # define the z probabilities (in this case Normal for both)
        # p(z): N(z|0,I)
        pz = torch.distributions.Normal(loc=torch.zeros_like(z_mu), scale=torch.ones_like(z_log_sd))
        # q(z|x,phi): N(z|mu, z_var)
        qz_x = torch.distributions.Normal(loc=z_mu, scale=torch.exp(z_log_sd))

        # repameterization trick: z = z_mu + xi (*) z_log_var, xi~N(xi|0,I)
        z = qz_x.rsample()
        # Monte Carlo KL divergence: MCKL(p(z)||q(z|x,phi)) = log(p(z)) - log(q(z|x,phi))
        # sum over weight dim, leaves the batch dim 
        kl_divergence = (pz.log_prob(z) - qz_x.log_prob(z)).sum(dim=1)
        kl_loss = -kl_divergence.mean()

        return z, kl_loss 

    def _match_output_length(self, x, target_length):
        current_length = x.size(-1)
        if current_length > target_length:
            return x[..., :target_length]
        if current_length < target_length:
            return F.pad(x, (0, target_length - current_length))
        return x

    def forward(self, x_s, x_i, x_a):
        """
        Forward pass `input_img` through the network
        """
        # reconstruction: 
        # encode:
        # input reshape:
        batch_size = x_s.shape[0]
        num_points = x_s.shape[-1]
        x_s = x_s.reshape(batch_size, 1, num_points)
        x_i = x_i.reshape(batch_size, 1, num_points)
        x_a = x_a.reshape(batch_size, 1, num_points)
        if not (torch.isfinite(x_s).all() and torch.isfinite(x_a).all()):
            raise ValueError("non-finite range/incidence input reached S3-Net")
        if self.feature_mode == "range_only":
            features = [x_s]
        elif self.feature_mode == "range_incidence":
            features = [x_s, x_a]
        else:
            if not torch.isfinite(x_i).all():
                raise ValueError("non-finite intensity input reached S3-Net")
            features = [x_s, x_i, x_a]
        # concatenate along channel axis
        x = torch.cat(features, dim=1)
          
        # encode: 
        z_mu, z_log_sd = self._encoder(x)

        # get the latent vector through reparameterization:
        z, kl_loss = self.vae_reparameterize(z_mu, z_log_sd)
    
        # decode:
        # reshape:
        z = z.transpose(1, 2).contiguous()
        x_d = self._decoder_z_mu(z)
        semantic_channels = self._decoder(x_d)
        semantic_channels = self._match_output_length(semantic_channels, num_points)

        # semantic grid: 10 channels
        semantic_scan = self.softmax(semantic_channels)

        return semantic_scan, semantic_channels, kl_loss

#
# end of class

#
# end of file
