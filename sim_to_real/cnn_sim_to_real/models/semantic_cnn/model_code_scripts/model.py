#!/usr/bin/env python
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, NPY, TXT
# 可能使用的关键环境变量：IGNORE_LABEL, IMG_SIZE, NEW_LINE, POINTS, POOL_ANGLE_MAX, POOL_ANGLE_MIN, POOL_MODES, PYTHONHASHSEED, SEED1, SEMANTIC_CNN_POOL_MODE, SEQ_LEN
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/baselines/semantic_cnn/training/scripts/model.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.784307748 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:00.811228313 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/render_semantic_cnn_offline_demo.py（导入其函数、类或模型）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/smoke_fixed_dual_training.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/control/run_model_demo.sh（ros2 launch 启动该场景）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/launch/s3net_fixed_dual_perception_demo.launch.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/s3net_fixed_dual_inference_node.py（导入其函数、类或模型）; /home/user/navigation_project/a_pipeline/workspaces/ros2_ws/src/semantic_nav_gazebo/scripts/semantic_cnn_fixed_dual_inference_node.py（导入其函数、类或模型）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/baselines/semantic_cnn/training/scripts/model.py
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
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np

# import modules
#
import os
import random
import json

# for reproducibility, we seed the rng
#
SEED1 = 1337
NEW_LINE = "\n"

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
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    torch.backends.cudnn.deterministic = True
    torch.backends.cudnn.benchmark = False
    random.seed(seed)
    os.environ['PYTHONHASHSEED'] = str(seed)
#
# end of method


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
IMG_SIZE = 80
SEQ_LEN = 10
IGNORE_LABEL = -1
LEGACY_SUB_GOAL_MEAN = np.asarray([0.30655652, 0.30655652], dtype=np.float32)
LEGACY_SUB_GOAL_STD = np.asarray([0.5378557, 0.5378557], dtype=np.float32)


def load_sub_goal_normalization(stats_path=None):
    """Load two-dimensional sub-goal statistics with legacy fallback."""
    stats_path = stats_path or os.environ.get("SEMANTIC_CNN_STATS_JSON")
    if not stats_path:
        return {
            "path": None,
            "source": "legacy_constants",
            "mean": LEGACY_SUB_GOAL_MEAN.copy(),
            "std": LEGACY_SUB_GOAL_STD.copy(),
        }
    stats_path = os.path.abspath(os.path.expanduser(str(stats_path)))
    if not os.path.isfile(stats_path):
        raise FileNotFoundError(
            "SEMANTIC_CNN_STATS_JSON does not exist: {}".format(stats_path)
        )
    with open(stats_path, "r") as stream:
        payload = json.load(stream)
    sub_goal = payload.get("sub_goal_local_xy")
    if not isinstance(sub_goal, dict):
        raise ValueError(
            "SemanticCNN stats must contain sub_goal_local_xy: {}".format(stats_path)
        )
    mean = np.asarray(sub_goal.get("mean"), dtype=np.float32).reshape(-1)
    std_values = sub_goal.get("std_population", sub_goal.get("std"))
    std = np.asarray(std_values, dtype=np.float32).reshape(-1)
    if mean.shape != (2,) or std.shape != (2,):
        raise ValueError(
            "SemanticCNN sub-goal mean/std must both have shape (2,): {}".format(
                stats_path
            )
        )
    if not np.all(np.isfinite(mean)) or not np.all(np.isfinite(std)):
        raise ValueError("SemanticCNN sub-goal mean/std must be finite")
    if np.any(std <= 0.0):
        raise ValueError("SemanticCNN sub-goal std must be positive")
    return {
        "path": stats_path,
        "source": "stats_json",
        "mean": mean,
        "std": std,
    }


def _majority_label(labels):
    labels = np.asarray(labels, dtype=np.int64).reshape(-1)
    labels = labels[labels >= 0]
    if labels.size == 0:
        return 0
    return int(np.bincount(labels, minlength=256).argmax())


POOL_MODES = ("global_virtual_angle_80", "sensor_split_40x2")
POOL_ANGLE_MIN = -np.pi / 2.0
POOL_ANGLE_MAX = np.pi / 2.0


def _pool_virtual_angles(ranges, angles, semantic, valid_mask, num_bins, range_max):
    ranges = np.asarray(ranges, dtype=np.float32).reshape(-1)
    angles = np.asarray(angles, dtype=np.float32).reshape(-1)
    semantic = np.nan_to_num(
        semantic, nan=IGNORE_LABEL, posinf=IGNORE_LABEL, neginf=IGNORE_LABEL
    ).astype(np.int64).reshape(-1)
    valid_mask = np.asarray(valid_mask, dtype=np.bool_).reshape(-1)
    if not (ranges.shape == angles.shape == semantic.shape == valid_mask.shape):
        raise ValueError("virtual range/angle, semantic label, and valid mask shapes must match")
    if not np.isfinite(range_max) or range_max <= 0.0:
        raise ValueError("pool range_max must be positive and finite")

    valid = (
        valid_mask
        & np.isfinite(ranges)
        & np.isfinite(angles)
        & (angles >= POOL_ANGLE_MIN)
        & (angles < POOL_ANGLE_MAX)
    )
    normalized_ranges = np.clip(ranges, 0.0, range_max) / range_max
    mins = np.ones(num_bins, dtype=np.float32)
    means = np.ones(num_bins, dtype=np.float32)
    sem_nearest = np.zeros(num_bins, dtype=np.float32)
    sem_majority = np.zeros(num_bins, dtype=np.float32)
    bin_valid = np.zeros(num_bins, dtype=np.bool_)
    if not np.any(valid):
        return mins, means, sem_nearest, sem_majority, bin_valid

    scale = num_bins / float(POOL_ANGLE_MAX - POOL_ANGLE_MIN)
    bin_ids = np.full(angles.shape, -1, dtype=np.int64)
    bin_ids[valid] = np.floor(
        (angles[valid] - POOL_ANGLE_MIN) * scale
    ).astype(np.int64)
    for bin_id in range(num_bins):
        indices = np.flatnonzero(valid & (bin_ids == bin_id))
        if indices.size == 0:
            continue
        values = normalized_ranges[indices]
        mins[bin_id] = float(values.min())
        means[bin_id] = float(values.mean())
        bin_valid[bin_id] = True
        labeled = indices[semantic[indices] >= 0]
        if labeled.size:
            nearest = labeled[int(np.argmin(normalized_ranges[labeled]))]
            sem_nearest[bin_id] = float(semantic[nearest])
            sem_majority[bin_id] = float(_majority_label(semantic[labeled]))
    return mins, means, sem_nearest, sem_majority, bin_valid


def _native_lidar_maps(
    virtual_ranges,
    virtual_angles,
    semantic,
    valid_mask,
    source_sensor,
    pool_mode="global_virtual_angle_80",
    range_max=50.0,
):
    source_sensor = np.asarray(source_sensor).reshape(-1)
    virtual_ranges = np.asarray(virtual_ranges).reshape(-1)
    if source_sensor.shape != virtual_ranges.shape:
        raise ValueError("source_sensor shape must match virtual LiDAR arrays")
    if pool_mode == "global_virtual_angle_80":
        return _pool_virtual_angles(
            virtual_ranges, virtual_angles, semantic, valid_mask, IMG_SIZE, range_max
        )
    if pool_mode != "sensor_split_40x2":
        raise ValueError("unsupported SemanticCNN pool mode: {}".format(pool_mode))

    arrays = (
        np.asarray(virtual_ranges),
        np.asarray(virtual_angles),
        np.asarray(semantic),
        np.asarray(valid_mask),
    )
    parts = []
    for sensor_id in (0, 1):
        sensor_mask = source_sensor == sensor_id
        if not np.any(sensor_mask):
            raise ValueError("sensor_split_40x2 requires source sensor {}".format(sensor_id))
        parts.append(
            _pool_virtual_angles(
                arrays[0][sensor_mask],
                arrays[1][sensor_mask],
                arrays[2][sensor_mask],
                arrays[3][sensor_mask],
                IMG_SIZE // 2,
                range_max,
            )
        )
    return tuple(np.concatenate((parts[0][index], parts[1][index])) for index in range(5))


class NavDataset(torch.utils.data.Dataset):
    def __init__(self, img_path, file_name, pooling_mode=None, stats_path=None):
        if not img_path.endswith(os.sep):
            img_path += os.sep
        self.pooling_mode = pooling_mode or os.environ.get(
            "SEMANTIC_CNN_POOL_MODE", "global_virtual_angle_80"
        )
        if self.pooling_mode not in POOL_MODES:
            raise ValueError("SemanticCNN pool mode must be one of {}".format(POOL_MODES))
        self.windows = []
        self.session_count = 0
        # parameters: data mean std: scan, sub_goal, intensity, angle of incidence: 
        #  [[4.518406, 8.2914915], [0.30655652, 0.5378557], [3081.8167, 1529.4413], [0.5959513, 0.4783924]]
        self.s_mu = 4.518406
        self.s_std = 8.2914915
        dataset_stats_path = os.path.join(img_path, "train_normalization_stats.json")
        if stats_path is None and not os.environ.get("SEMANTIC_CNN_STATS_JSON"):
            stats_path = dataset_stats_path if os.path.isfile(dataset_stats_path) else None
        goal_normalization = load_sub_goal_normalization(stats_path)
        self.normalization_stats_path = goal_normalization["path"]
        self.normalization_source = goal_normalization["source"]
        self.g_mu = goal_normalization["mean"]
        self.g_std = goal_normalization["std"]
        self.i_mu = 3081.8167
        self.i_std = 1529.4413
        self.a_mu = 0.5959513
        self.a_std = 0.4783924
        
        with open(img_path+'dataset.txt','r') as fp_folder:
            folder_lines = fp_folder.read().split(NEW_LINE)
        for folder_line in folder_lines:
            folder_path = folder_line.strip().rstrip("/")
            if not folder_path:
                continue
            if '-' not in folder_path:
                raise ValueError(
                    "dataset session names must contain '-': {}".format(folder_path)
                )
            sample_root = os.path.join(img_path, folder_path)
            split_path = os.path.join(sample_root, file_name + '.txt')
            if not os.path.isfile(split_path):
                raise FileNotFoundError(
                    "dataset session is missing split: {}".format(split_path)
                )
            required_dirs = (
                "virtual_ranges_lidar",
                "virtual_angles_lidar",
                "valid_mask_lidar",
                "semantic_label",
                "source_sensor",
                "cmd_velocities",
                "sub_goals_local",
            )
            for directory in required_dirs:
                path = os.path.join(sample_root, directory)
                if not os.path.isdir(path):
                    raise FileNotFoundError(
                        "{} is required for fixed-dual SemanticCNN training: {}".format(
                            directory + '/', sample_root
                        )
                    )
            metadata_path = os.path.join(sample_root, "metadata.json")
            if not os.path.isfile(metadata_path):
                raise FileNotFoundError("metadata.json is required: {}".format(sample_root))
            with open(metadata_path, "r") as stream:
                metadata = json.load(stream)
            range_values = (
                float(metadata.get("range_max_01", float("nan"))),
                float(metadata.get("range_max_02", float("nan"))),
                float(metadata.get("pool_range_max", float("nan"))),
            )
            if any(not np.isfinite(value) or value <= 0.0 for value in range_values):
                raise ValueError(
                    "range_max_01, range_max_02, and pool_range_max must be positive; "
                    "got {} for {}".format(range_values, sample_root)
                )
            if range_values[2] + 1e-9 < max(range_values[0], range_values[1]):
                raise ValueError(
                    "pool_range_max must cover both sensor range maxima; got {} for {}".format(
                        range_values, sample_root
                    )
                )
            if not np.isclose(float(metadata.get("pool_angle_min")), POOL_ANGLE_MIN, atol=1e-6):
                raise ValueError("metadata pool_angle_min must be -pi/2")
            if not np.isclose(float(metadata.get("pool_angle_max")), POOL_ANGLE_MAX, atol=1e-6):
                raise ValueError("metadata pool_angle_max must be pi/2")

            with open(split_path, 'r') as fp_file:
                split_names = [
                    line.strip() for line in fp_file.read().split(NEW_LINE)
                    if line.strip().endswith('.npy')
                ]
            selected = set(split_names)
            ordered_records = metadata.get("frames", [])
            frame_records = {record["name"]: record for record in ordered_records}
            frame_positions = {
                record["name"]: index
                for index, record in enumerate(ordered_records)
            }
            expected_period = float(metadata.get("expected_frame_period_ms", 100.0))
            tolerance = float(metadata.get("frame_period_tolerance_ms", 20.0))
            if not np.isfinite(expected_period) or expected_period <= 0.0:
                raise ValueError("metadata expected_frame_period_ms must be positive and finite")
            if not np.isfinite(tolerance) or tolerance <= 0.0:
                raise ValueError("metadata frame_period_tolerance_ms must be positive and finite")
            for end_name in split_names:
                if end_name not in frame_positions:
                    raise ValueError("split name is missing from metadata.frames")
                end_index = frame_positions[end_name]
                start_index = end_index - SEQ_LEN + 1
                if start_index < 0:
                    continue
                records = ordered_records[start_index : end_index + 1]
                names = [record["name"] for record in records]
                if any(name not in selected for name in names):
                    continue
                episode_ids = {
                    int(record.get("episode_id", -1)) for record in records
                }
                if len(episode_ids) != 1 or -1 in episode_ids:
                    continue
                stamps = [int(frame_records[name]["scan_01_stamp_ns"]) for name in names]
                deltas = [
                    (right - left) / 1_000_000.0
                    for left, right in zip(stamps, stamps[1:])
                ]
                if not all(abs(delta - expected_period) <= tolerance for delta in deltas):
                    continue
                self.windows.append(
                    {"root": sample_root, "names": names, "range_max": range_values[2]}
                )
            self.session_count += 1

        self.length = len(self.windows)
        print("dataset windows: ", self.length)
        print("fixed-dual sessions: ", self.session_count)
        print("SemanticCNN pool mode: ", self.pooling_mode)
        print("SemanticCNN normalization source: ", self.normalization_source)
        print("SemanticCNN sub-goal mean/std: ", self.g_mu.tolist(), self.g_std.tolist())


    def __len__(self):
        return self.length

    def __getitem__(self, idx):
        window = self.windows[idx]
        npy_path = window["root"]
        names = window["names"]
        end_str = names[-1]

        sub_goal = np.load(f"{npy_path}/sub_goals_local/{end_str}")
        target = np.load(f"{npy_path}/cmd_velocities/{end_str}").astype(np.float32).reshape(-1)
        if target.shape != (3,):
            raise ValueError(f"cmd_velocities target must have shape (3,): {npy_path}/cmd_velocities/{end_str}")
        velocity = target[[0, 2]]

        scan_avg = np.zeros((SEQ_LEN * 2, IMG_SIZE), dtype=np.float32)
        semantic_avg = np.zeros((SEQ_LEN * 2, IMG_SIZE), dtype=np.float32)
        bin_valid_history = np.zeros((SEQ_LEN, IMG_SIZE), dtype=np.bool_)

        for n, frame_idx in enumerate(names):
            virtual_ranges = np.load(f"{npy_path}/virtual_ranges_lidar/{frame_idx}")
            virtual_angles = np.load(f"{npy_path}/virtual_angles_lidar/{frame_idx}")
            semantic = np.load(f"{npy_path}/semantic_label/{frame_idx}")
            valid_mask = np.load(f"{npy_path}/valid_mask_lidar/{frame_idx}")
            source_sensor = np.load(f"{npy_path}/source_sensor/{frame_idx}")
            mins, means, sem_min, sem_mode, bin_valid = _native_lidar_maps(
                virtual_ranges,
                virtual_angles,
                semantic,
                valid_mask,
                source_sensor,
                pool_mode=self.pooling_mode,
                range_max=window["range_max"],
            )
            scan_avg[2 * n] = mins
            semantic_avg[2 * n] = sem_min
            scan_avg[2 * n + 1] = means
            semantic_avg[2 * n + 1] = sem_mode
            bin_valid_history[n] = bin_valid

        row_repeat = IMG_SIZE // (SEQ_LEN * 2)
        if row_repeat * (SEQ_LEN * 2) != IMG_SIZE:
            raise ValueError("IMG_SIZE must be divisible by SEQ_LEN*2 for row expansion")
        scan_map = np.repeat(scan_avg, row_repeat, axis=0)
        semantic_map = np.repeat(semantic_avg, row_repeat, axis=0)

        sub_goal[np.isnan(sub_goal)] = 0.
        sub_goal[np.isinf(sub_goal)] = 0.
        velocity[np.isnan(velocity)] = 0.
        velocity[np.isinf(velocity)] = 0.

        sub_goal = (sub_goal - self.g_mu) / self.g_std
        if not (
            np.all(np.isfinite(scan_map))
            and np.all(np.isfinite(semantic_map))
            and np.all(np.isfinite(sub_goal))
            and np.all(np.isfinite(velocity))
        ):
            raise ValueError("non-finite SemanticCNN input or target")

        scan_tensor = torch.FloatTensor(scan_map)
        semantic_tensor = torch.FloatTensor(semantic_map)
        sub_goal_tensor = torch.FloatTensor(sub_goal)
        velocity_tensor = torch.FloatTensor(velocity)
        bin_valid_tensor = torch.BoolTensor(bin_valid_history)

        data = {
                'scan_map': scan_tensor,
                'semantic_map': semantic_tensor,
                'sub_goal': sub_goal_tensor,
                'velocity': velocity_tensor,
                'target': velocity_tensor.clone(),
                'bin_valid_mask': bin_valid_tensor,
                }

        return data

#
# end of function


#------------------------------------------------------------------------------
#
# ResNet blocks
#
#------------------------------------------------------------------------------
def conv3x3(in_planes, out_planes, stride=1, groups=1, dilation=1):
    """3x3 convolution with padding"""
    return nn.Conv2d(in_planes, out_planes, kernel_size=3, stride=stride,
                     padding=dilation, groups=groups, bias=False, dilation=dilation)

def conv1x1(in_planes, out_planes, stride=1):
    """1x1 convolution"""
    return nn.Conv2d(in_planes, out_planes, kernel_size=1, stride=stride, bias=False)

class Bottleneck(nn.Module):
    # Bottleneck in torchvision places the stride for downsampling at 3x3 convolution(self.conv2)
    # while original implementation places the stride at the first 1x1 convolution(self.conv1)
    # according to "Deep residual learning for image recognition"https://arxiv.org/abs/1512.03385.
    # This variant is also known as ResNet V1.5 and improves accuracy according to
    # https://ngc.nvidia.com/catalog/model-scripts/nvidia:resnet_50_v1_5_for_pytorch.

    expansion = 2 #4

    def __init__(self, inplanes, planes, stride=1, downsample=None, groups=1,
                 base_width=64, dilation=1, norm_layer=None):
        super(Bottleneck, self).__init__()
        if norm_layer is None:
            norm_layer = nn.BatchNorm2d
        width = int(planes * (base_width / 64.)) * groups
        # Both self.conv2 and self.downsample layers downsample the input when stride != 1
        self.conv1 = conv1x1(inplanes, width)
        self.bn1 = norm_layer(width)
        self.conv2 = conv3x3(width, width, stride, groups, dilation)
        self.bn2 = norm_layer(width)
        self.conv3 = conv1x1(width, planes * self.expansion)
        self.bn3 = norm_layer(planes * self.expansion)
        self.relu = nn.ReLU(inplace=False)
        self.downsample = downsample
        self.stride = stride

    def forward(self, x):
        identity = x

        out = self.conv1(x)
        out = self.bn1(out)
        out = self.relu(out)

        out = self.conv2(out)
        out = self.bn2(out)
        out = self.relu(out)

        out = self.conv3(out)
        out = self.bn3(out)

        if self.downsample is not None:
            identity = self.downsample(x)

        out = out + identity
        out = self.relu(out)

        return out
#
# end of ResNet blocks


#------------------------------------------------------------------------------
#
# the model is defined here
#
#------------------------------------------------------------------------------

# define the PyTorch MLP model
#
class SemanticCNN(nn.Module):

    # function: init
    #
    # arguments: input_size - int representing size of input
    #            hidden_size - number of nodes in the hidden layer
    #            num_classes - number of classes to classify
    #
    # return: none
    #
    # This method is the main function.
    #
    def __init__(self, block, layers, num_classes=2, zero_init_residual=True,
                 groups=1, width_per_group=64, replace_stride_with_dilation=None,
                 norm_layer=None):

        # inherit the superclass properties/methods
        #
        super(SemanticCNN, self).__init__()
        # define the model
        #
        ################## ped_pos net model: ###################
        if norm_layer is None:
            norm_layer = nn.BatchNorm2d
        self._norm_layer = norm_layer

        self.inplanes = 64
        self.dilation = 1
        if replace_stride_with_dilation is None:
            # each element in the tuple indicates if we should replace
            # the 2x2 stride with a dilated convolution instead
            replace_stride_with_dilation = [False, False, False]
        if len(replace_stride_with_dilation) != 3:
            raise ValueError("replace_stride_with_dilation should be None "
                             "or a 3-element tuple, got {}".format(replace_stride_with_dilation))
        self.groups = groups
        self.base_width = width_per_group
        self.conv1 = nn.Conv2d(2, self.inplanes, kernel_size=3, stride=1, padding=1,
                               bias=False)
        self.bn1 = norm_layer(self.inplanes)
        self.relu = nn.ReLU(inplace=False)
        self.maxpool = nn.MaxPool2d(kernel_size=3, stride=1, padding=1)
        self.layer1 = self._make_layer(block, 64, layers[0])
        self.layer2 = self._make_layer(block, 128, layers[1], stride=2,
                                       dilate=replace_stride_with_dilation[0])
        self.layer3 = self._make_layer(block, 256, layers[2], stride=2,
                                       dilate=replace_stride_with_dilation[1])

        self.conv2_2 = nn.Sequential(
            nn.Conv2d(in_channels=256, out_channels=128, kernel_size=(1, 1), stride=(1,1), padding=(0, 0)),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=False),

            nn.Conv2d(in_channels=128, out_channels=128, kernel_size=(3, 3), stride=(1,1), padding=(1, 1)),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=False),

            nn.Conv2d(in_channels=128, out_channels=256, kernel_size=(1, 1), stride=(1,1), padding=(0, 0)),
            nn.BatchNorm2d(256)
        )
        self.downsample2 = nn.Sequential(
            nn.Conv2d(in_channels=128, out_channels=256, kernel_size=(1, 1), stride=(2,2), padding=(0, 0)),
            nn.BatchNorm2d(256)
        )
        self.relu2 = nn.ReLU(inplace=False)

        self.conv3_2 = nn.Sequential(
            nn.Conv2d(in_channels=512, out_channels=256, kernel_size=(1, 1), stride=(1,1), padding=(0, 0)),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=False),

            nn.Conv2d(in_channels=256, out_channels=256, kernel_size=(3, 3), stride=(1,1), padding=(1, 1)),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=False),

            nn.Conv2d(in_channels=256, out_channels=512, kernel_size=(1, 1), stride=(1,1), padding=(0, 0)),
            nn.BatchNorm2d(512)
        )
        self.downsample3 = nn.Sequential(
            nn.Conv2d(in_channels=64, out_channels=512, kernel_size=(1, 1), stride=(4,4), padding=(0, 0)),
            nn.BatchNorm2d(512)
        )
        self.relu3 = nn.ReLU(inplace=False)

        # self.layer4 = self._make_layer(block, 512, layers[3], stride=2,
        #                               dilate=replace_stride_with_dilation[2])
        self.avgpool = nn.AdaptiveAvgPool2d((1, 1))
        self.fc = nn.Linear(256 * block.expansion + 2, num_classes)

        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')
            elif isinstance(m, (nn.BatchNorm2d, nn.GroupNorm)):
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0)
            elif isinstance(m, nn.BatchNorm1d): # add by xzt
                nn.init.constant_(m.weight, 1)
                nn.init.constant_(m.bias, 0) 
            elif isinstance(m, nn.Linear):
                nn.init.xavier_normal_(m.weight)

        # Zero-initialize the last BN in each residual branch,
        # so that the residual branch starts with zeros, and each residual block behaves like an identity.
        # This improves the model by 0.2~0.3% according to https://arxiv.org/abs/1706.02677
        if zero_init_residual:
            for m in self.modules():
                if isinstance(m, Bottleneck):
                    nn.init.constant_(m.bn3.weight, 0)           

    def _make_layer(self, block, planes, blocks, stride=1, dilate=False):
        norm_layer = self._norm_layer
        downsample = None
        previous_dilation = self.dilation
        if dilate:
            self.dilation *= stride
            stride = 1
        if stride != 1 or self.inplanes != planes * block.expansion:
            downsample = nn.Sequential(
                conv1x1(self.inplanes, planes * block.expansion, stride),
                norm_layer(planes * block.expansion),
            )

        layers = []
        layers.append(block(self.inplanes, planes, stride, downsample, self.groups,
                            self.base_width, previous_dilation, norm_layer))
        self.inplanes = planes * block.expansion
        for _ in range(1, blocks):
            layers.append(block(self.inplanes, planes, groups=self.groups,
                                base_width=self.base_width, dilation=self.dilation,
                                norm_layer=norm_layer))

        return nn.Sequential(*layers)

    def _forward_impl(self, scan, semantics, goal):
        ###### Start of fusion net ######
        scan_in = scan.reshape(-1,1,80,80)
        semantics_in = semantics.reshape(-1,1,80,80)
        fusion_in = torch.cat((scan_in, semantics_in), dim=1)

        # See note [TorchScript super()]
        x = self.conv1(fusion_in)
        x = self.bn1(x)
        x = self.relu(x)
        x = self.maxpool(x)

        identity3 = self.downsample3(x)

        x = self.layer1(x)

        identity2 = self.downsample2(x)

        x = self.layer2(x)

        x = self.conv2_2(x)
        x = x + identity2
        x = self.relu2(x)


        x = self.layer3(x)
        # x = self.layer4(x)

        x = self.conv3_2(x)
        x = x + identity3
        x = self.relu3(x)

        x = self.avgpool(x)
        fusion_out = torch.flatten(x, 1)
        ###### End of fusion net ######

        ###### Start of goal net #######
        goal_in = goal.reshape(-1,2)
        goal_out = torch.flatten(goal_in, 1)
        ###### End of goal net #######
        # Combine
        fc_in = torch.cat((fusion_out, goal_out), dim=1)
        x = self.fc(fc_in)  

        return x

    def forward(self, scan, semantics, goal):
        return self._forward_impl(scan, semantics, goal)
    #
    # end of method
#
# end of class

#
# end of file
