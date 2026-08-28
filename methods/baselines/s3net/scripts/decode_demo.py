#!/usr/bin/env python
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JPG, TXT
# 可能使用的关键环境变量：AGNLE_MAX, AGNLE_MIN, CLASSES, DEFAULT_LABEL_NAMES, EVAL_SET, MDL_PATH, NUM_ARGS, NUM_CLASSES, NUM_OUTPUT_CHANNELS, ODIR, RANGE_MAX, S3NET_FEATURE_MODE, S3NET_STATS_JSON, S3NET_STATS_PATH, SEED1, SPACE
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/decode_demo.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.733305586 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:18:51.814067218 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/run_eval_demo.sh（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/09_train_s3net_native_stats.sh（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/s3net/20260727_162813_s3net_native_stats_301epoch/work/s3_net_v7/run_eval_demo.sh（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/training/s3net/20260717_172858_s3net_native_stats_301epoch/work/s3_net_v7/run_eval_demo.sh（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/decode_demo.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/run_eval_demo.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/09_train_s3net_native_stats.sh; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/s3net/20260727_162813_s3net_native_stats_301epoch/work/s3_net_v7/run_eval_demo.sh; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/training/s3net/20260717_172858_s3net_native_stats_301epoch/work/s3_net_v7/run_eval_demo.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
#
# file: $ISIP_EXP/tuh_dpath/exp_0074/scripts/decode.py
#
# revision history:
#  20190925 (TE): first version
#
# usage:
#  python decode.py odir mfile data
#
# arguments:
#  odir: the directory where the hypotheses will be stored
#  mfile: input model file
#  data: the input data list to be decoded
#
# This script decodes data using a simple MLP model.
#------------------------------------------------------------------------------

# import pytorch modules
#
import torch
import torch.nn as nn
from tqdm import tqdm

# visualize:
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


import matplotlib
matplotlib.style.use('ggplot')

# import the model and all of its variables/functions
#
from model import *
# import modules
#
import sys
import os



#-----------------------------------------------------------------------------
#
# global variables are listed here
#
#-----------------------------------------------------------------------------

# general global values
#
NUM_ARGS = 3
SPACE = " "      

# Constants
DEFAULT_LABEL_NAMES = ['_background_', 'Chair', 'Door', 'Elevator', 'Person', 'Pillar', 'Sofa', 'Table', 'Trash bin', 'Wall']
NUM_CLASSES = len(DEFAULT_LABEL_NAMES)
NUM_OUTPUT_CHANNELS = NUM_CLASSES
S3NET_STATS_PATH = os.environ.get("S3NET_STATS_JSON")
CLASSES = list(DEFAULT_LABEL_NAMES)

# Hokuyo UTM-30LX-EW:
AGNLE_MIN = -2.356194496154785
AGNLE_MAX = 2.356194496154785
RANGE_MAX = 60.0

# for reproducibility, we seed the rng
#
set_seed(SEED1)        


def load_label_names(dataset_root):
    path = os.path.join(dataset_root, 'label_names.txt')
    if os.path.isfile(path):
        with open(path, encoding='utf-8') as stream:
            names = [line.strip() for line in stream if line.strip()]
        if len(names) >= 2 and names[0] == '_background_':
            return names
        raise ValueError(f'invalid label names file: {path}')
    return list(DEFAULT_LABEL_NAMES)


def plot_semantic_scan(ax, theta, r, labels):
    valid = (
        np.isfinite(theta)
        & np.isfinite(r)
        & (r > 0.0)
        & (labels >= 0)
        & (labels < NUM_CLASSES)
    )
    theta = theta[valid]
    r = r[valid]
    labels = labels[valid].astype(int)
    label_val = np.unique(labels)

    scatter = ax.scatter(
        theta,
        r,
        c=labels,
        s=6,
        cmap='nipy_spectral',
        vmin=0,
        vmax=NUM_CLASSES - 1,
        alpha=0.95,
        linewidth=10,
    )
    ax.set_xticks(np.linspace(AGNLE_MIN, AGNLE_MAX, 8, endpoint='true'))
    ax.set_thetamin(-135)
    ax.set_thetamax(135)
    ax.set_yticklabels([])
    plt.xticks(fontsize=16)
    plt.yticks(fontsize=16)
    if label_val.size > 0:
        plt.legend(
            handles=scatter.legend_elements(num=[j for j in label_val])[0],
            labels=[CLASSES[j] for j in label_val],
            bbox_to_anchor=(0.5, -0.08),
            loc='lower center',
            fontsize=18,
        )
    ax.grid(False)
    ax.set_theta_offset(np.pi/2)


def save_side_by_side(left_path, right_path, output_path):
    with Image.open(left_path) as left_image, Image.open(right_path) as right_image:
        left_image = left_image.convert('RGB')
        right_image = right_image.convert('RGB')
        height = max(left_image.height, right_image.height)
        comparison = Image.new(
            'RGB',
            (left_image.width + right_image.width, height),
            color='white',
        )
        comparison.paste(left_image, (0, 0))
        comparison.paste(right_image, (left_image.width, 0))
        comparison.save(output_path, quality=95)


#------------------------------------------------------------------------------
#
# the main program starts here
#
#------------------------------------------------------------------------------

# function: main
#
# arguments: none
#
# return: none
#
# This method is the main function.
#
def main(argv):
    global NUM_CLASSES, NUM_OUTPUT_CHANNELS, CLASSES
    # ensure we have the correct number of arguments:
    if(len(argv) != NUM_ARGS):
        print("usage: python nedc_decode_mdl.py [ODIR] [MDL_PATH] [EVAL_SET]")
        exit(-1)

    # define local variables:
    odir = argv[0]
    mdl_path = argv[1]
    fImg = argv[2]

    # if the odir doesn't exist, we make it:
    if not os.path.exists(odir):
        os.makedirs(odir)


    # set the device to use GPU if available:
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # get array of the data
    # data: [[0, 1, ... 26], [27, 28, ...] ...]
    # labels: [0, 0, 1, ...]
    #
    #[ped_pos_e, scan_e, goal_e, vel_e] = get_data(fname)
    eval_dataset = VaeTestDataset(fImg, 'dev', stats_path=S3NET_STATS_PATH)
    valid_files = [
        (scan_name, intensity_name, angle_name, valid_mask_name, native_flag, label_name)
        for scan_name, intensity_name, angle_name, valid_mask_name, native_flag, label_name in zip(
            eval_dataset.scan_file_names,
            eval_dataset.intensity_file_names,
            eval_dataset.angle_file_names,
            eval_dataset.valid_mask_file_names,
            eval_dataset.native_sample_flags,
            eval_dataset.label_file_names,
        )
        if os.path.exists(scan_name)
        and os.path.exists(intensity_name)
        and os.path.exists(label_name)
    ]
    skipped_files = eval_dataset.length - len(valid_files)
    if skipped_files > 0:
        print("Warning: skipping", skipped_files, "samples with missing scan/intensity/label files")
        eval_dataset.scan_file_names = [item[0] for item in valid_files]
        eval_dataset.intensity_file_names = [item[1] for item in valid_files]
        eval_dataset.angle_file_names = [item[2] for item in valid_files]
        eval_dataset.valid_mask_file_names = [item[3] for item in valid_files]
        eval_dataset.native_sample_flags = [item[4] for item in valid_files]
        eval_dataset.label_file_names = [item[5] for item in valid_files]
        eval_dataset.length = len(valid_files)
        print("usable dataset length:", eval_dataset.length)
    eval_dataloader = torch.utils.data.DataLoader(eval_dataset, batch_size=1, \
                                                   shuffle=False, drop_last=False) #, pin_memory=True)

    checkpoint = torch.load(mdl_path, map_location=device)
    CLASSES = load_label_names(fImg)
    NUM_CLASSES = int(checkpoint.get('num_output_channels', len(CLASSES)))
    NUM_OUTPUT_CHANNELS = NUM_CLASSES
    if len(CLASSES) != NUM_CLASSES:
        raise ValueError(
            f'checkpoint expects {NUM_CLASSES} classes but dataset defines {len(CLASSES)}'
        )
    feature_mode = checkpoint.get(
        'feature_mode', os.environ.get('S3NET_FEATURE_MODE', 'range_intensity_incidence')
    )
    # instantiate a model:
    model = S3Net(input_channels=feature_mode_num_channels(feature_mode),
                 output_channels=NUM_OUTPUT_CHANNELS,
                 feature_mode=feature_mode)
    # moves the model to device (cpu in our case so no change):
    model.to(device)

    # set the model to evaluate
    #
    model.eval()

    # set the loss criterion:
    criterion = nn.MSELoss(reduction='sum') #, weight=class_weights)
    criterion.to(device)

    # load the weights
    #
    model.load_state_dict(checkpoint.get('model_state_dict', checkpoint['model']))

    # for each batch in increments of batch size:
    counter = 0
    num_samples = 32
    # get the number of batches (ceiling of train_data/batch_size):
    num_batches = int(len(eval_dataset)/eval_dataloader.batch_size)
    with torch.no_grad():
        for i, batch in tqdm(enumerate(eval_dataloader), total=num_batches):
        #for i, batch in enumerate(dataloader, 0):
            if(i % 100 == 0):
                counter += 1
                # collect the samples as a batch:
                scans = batch['scan']
                scans = scans.to(device)
                intensities = batch['intensity']
                intensities = intensities.to(device)
                angle_incidence = batch['angle_incidence']
                angle_incidence = angle_incidence.to(device)
                labels = batch['label']
                labels = labels.to(device)

                # feed the batch to the network:
                inputs_samples = scans.repeat(num_samples,1,1)
                intensity_samples = intensities.repeat(num_samples,1,1)
                angle_incidence_samples = angle_incidence.repeat(num_samples,1,1)

                # feed the batch to the network:
                semantic_scan, semantic_channels, kl_loss = model(inputs_samples, intensity_samples, angle_incidence_samples)

                semantic_scans = semantic_scan.cpu().detach().numpy()
                semantic_scans_mx = semantic_scans.argmax(axis=1)

                # majority vote:
                semantic_scans_mx_mean = np.apply_along_axis(
                    lambda x: np.bincount(x, minlength=NUM_CLASSES).argmax(),
                    axis=0,
                    arr=semantic_scans_mx,
                )

                # plot:
                r = scans.cpu().detach().numpy().reshape(-1)
                r = r * eval_dataset.s_std + eval_dataset.s_mu
                points = r.shape[0]
                theta = batch['angles'].cpu().detach().numpy().reshape(points)

                ## plot semantic label:
                fig = plt.figure(figsize=(12, 12))
                ax = fig.add_subplot(1,1,1, projection='polar', facecolor='seashell')
                smap = labels.cpu().detach().numpy().reshape(points)
                plot_semantic_scan(ax, theta, r, smap)

                ground_truth_img_name = os.path.join(
                    odir, "semantic_ground_truth_" + str(i) + ".jpg"
                )
                plt.savefig(ground_truth_img_name, bbox_inches='tight')
                plt.close(fig)
                #plt.show()

                ## plot s3-net semantic seg,ementation:
                fig = plt.figure(figsize=(12, 12))
                ax = fig.add_subplot(1,1,1, projection='polar', facecolor='seashell')
                r = scans.cpu().detach().numpy().reshape(-1)
                r = r * eval_dataset.s_std + eval_dataset.s_mu
                points = r.shape[0]
                theta = batch['angles'].cpu().detach().numpy().reshape(points)
                smap = semantic_scans_mx_mean.reshape(points)
                valid_mask = batch['valid_mask'].cpu().detach().numpy().reshape(points)
                smap = np.where(valid_mask, smap, -1)
                plot_semantic_scan(ax, theta, r, smap)

                prediction_img_name = os.path.join(
                    odir, "semantic_s3net_" + str(i) + ".jpg"
                )
                plt.savefig(prediction_img_name, bbox_inches='tight')
                plt.close(fig)

                comparison_img_name = os.path.join(
                    odir, "semantic_comparison_" + str(i) + ".jpg"
                )
                save_side_by_side(
                    ground_truth_img_name,
                    prediction_img_name,
                    comparison_img_name,
                )

                print(i)
        
    
    # exit gracefully
    #
    return True
#
# end of function


# begin gracefully
#
if __name__ == '__main__':
    main(sys.argv[1:])
#
# end of file
