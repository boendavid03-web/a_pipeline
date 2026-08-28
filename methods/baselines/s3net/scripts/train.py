#!/usr/bin/env python
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：PT
# 可能使用的关键环境变量：BATCH_SIZE, BETA, BETAS, CHECKPOINT_INTERVAL, DEV_MASK_PATH, DEV_PATH, IGNORE_LABEL, LEARNING_RATE, MDL_PATH, NUM_ARGS, NUM_EPOCHS, NUM_INPUT_CHANNELS, NUM_OUTPUT_CHANNELS, S3NET_FEATURE_MODE, S3NET_NUM_CLASSES, S3NET_STATS_JSON, S3NET_STATS_PATH, SEED1, TRAIN_MASK_PATH, TRAIN_PATH
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/train.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.734305629 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:00.811228313 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/s3net/20260727_162813_s3net_native_stats_301epoch/model_code_scripts/train.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/s3net/20260727_162813_s3net_native_stats_301epoch/work/s3_net_v7/scripts/train.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/training/s3net/20260717_172858_s3net_native_stats_301epoch/model_code_scripts/train.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/training/s3net/20260717_172858_s3net_native_stats_301epoch/work/s3_net_v7/scripts/train.py（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/train.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/s3net/20260727_162813_s3net_native_stats_301epoch/model_code_scripts/train.py; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/s3net/20260727_162813_s3net_native_stats_301epoch/work/s3_net_v7/scripts/train.py; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/training/s3net/20260717_172858_s3net_native_stats_301epoch/model_code_scripts/train.py; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/training/s3net/20260717_172858_s3net_native_stats_301epoch/work/s3_net_v7/scripts/train.py
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
#
# file: $ISIP_EXP/SOGMP/scripts/train.py
#
# revision history: xzt
#  20220824 (TE): first version
#
# usage:
#  python train.py mdir train_data val_data
#
# arguments:
#  mdir: the directory where the output model is stored
#  train_data: the directory of training data
#  val_data: the directory of valiation data
#
# This script trains a S3-Net model
#------------------------------------------------------------------------------

# import pytorch modules
#
import torch
import torch.nn as nn
from torch.optim import Adam
from tqdm import tqdm
import torch.nn.functional as F

# visualize:
from tensorboardX import SummaryWriter
import numpy as np

# import the model and all of its variables/functions
#
from model import *
import lovasz_losses as L

# import modules
#
import sys
import os
import json
from datetime import datetime


#-----------------------------------------------------------------------------
#
# global variables are listed here
#
#-----------------------------------------------------------------------------

# general global values
#
model_dir = './model/s3_net_model.pth'  # the path of model storage
NUM_ARGS = 3
NUM_EPOCHS = 20000
BATCH_SIZE = 1024
CHECKPOINT_INTERVAL = 10
LEARNING_RATE = "lr"
BETAS = "betas"
EPS = "eps"
WEIGHT_DECAY = "weight_decay"

# Constants
S3NET_FEATURE_MODE = os.environ.get("S3NET_FEATURE_MODE", "range_intensity_incidence")
NUM_INPUT_CHANNELS = feature_mode_num_channels(S3NET_FEATURE_MODE)
NUM_OUTPUT_CHANNELS = int(os.environ.get("S3NET_NUM_CLASSES", "10"))
if NUM_OUTPUT_CHANNELS < 2:
    raise ValueError("S3NET_NUM_CLASSES must include background plus at least one semantic class")
BETA = 0.01
S3NET_STATS_PATH = os.environ.get("S3NET_STATS_JSON")

# for reproducibility, we seed the rng
#
set_seed(SEED1)

# adjust_learning_rate
#
def adjust_learning_rate(optimizer, epoch):
    lr = 1e-4
    if epoch > 50000:
        lr = 2e-5
    if epoch > 480000:
       # lr = 5e-8
       lr = lr * (0.1 ** (epoch // 110000))
    #  if epoch > 8300:
    #      lr = 1e-9
    for param_group in optimizer.param_groups:
        param_group['lr'] = lr


# train function:
def train(model, dataloader, dataset, device, optimizer, ce_criterion, lovasz_criterion, class_weights, epoch, epochs):
    # set model to training mode:
    model.train()
    # for each batch in increments of batch size:
    running_loss = 0.0
    # kl_divergence:
    kl_avg_loss = 0.0
    # CE loss:
    ce_avg_loss = 0.0

    counter = 0
    # get the number of batches (ceiling of train_data/batch_size):
    num_batches = len(dataloader)
    for i, batch in tqdm(enumerate(dataloader), total=num_batches):
    #for i, batch in enumerate(dataloader, 0):
        counter += 1
        # collect the samples as a batch:
        scans = batch['scan']
        scans = scans.to(device)
        intensities = batch['intensity']
        intensities = intensities.to(device)
        angle_incidence = batch['angle_incidence']
        angle_incidence = angle_incidence.to(device)
        if not (
            torch.isfinite(scans).all()
            and torch.isfinite(intensities).all()
            and torch.isfinite(angle_incidence).all()
        ):
            raise ValueError("non-finite S3-Net training input before model forward")
        labels = batch['label']
        labels = labels.to(device).long()

        batch_size = scans.size(0)

        # set all gradients to 0:
        optimizer.zero_grad()

        # feed the batch to the network:
        semantic_scan, semantic_channels, kl_loss = model(scans, intensities, angle_incidence)
        # calculate the semantic ce loss:
        ce_loss = ce_criterion(semantic_channels, labels).div(batch_size)
        lovasz_loss, _ = lovasz_criterion(semantic_channels, labels)
        lovasz_loss = lovasz_loss.mul(class_weights.to(device)).sum()
        # beta-vae:
        loss = ce_loss + BETA*kl_loss + lovasz_loss
        # perform back propagation:
        loss.backward(torch.ones_like(loss))
        optimizer.step()
        # get the loss:
        # multiple GPUs:
        if torch.cuda.device_count() > 1:
            loss = loss.mean()
            ce_loss = ce_loss.mean()
            kl_loss = lovasz_loss.mean() #kl_loss.mean()

        running_loss += loss.item()
        # kl_divergence:
        kl_avg_loss += lovasz_loss.item() #kl_loss.item()
        # CE loss:
        ce_avg_loss += ce_loss.item()

        # display informational message:
        if(i % 512 == 0):
            print('Epoch [{}/{}], Step[{}/{}], Loss: {:.4f}, CE_Loss: {:.4f}, Lovasz_Loss: {:.4f}'
                    .format(epoch, epochs, i + 1, num_batches, loss.item(), ce_loss.item(), lovasz_loss.item()))

    train_loss = running_loss / max(counter, 1)
    train_kl_loss = kl_avg_loss / max(counter, 1)
    train_ce_loss = ce_avg_loss / max(counter, 1)

    return train_loss, train_kl_loss, train_ce_loss

# validate function:
def validate(model, dataloader, dataset, device, ce_criterion, lovasz_criterion, class_weights):
    # set model to evaluation mode:
    model.eval()
    # for each batch in increments of batch size:
    running_loss = 0.0
    # kl_divergence:
    kl_avg_loss = 0.0
    # CE loss:
    ce_avg_loss = 0.0

    counter = 0
    # get the number of batches (ceiling of train_data/batch_size):
    num_batches = len(dataloader)
    with torch.no_grad():
        for i, batch in tqdm(enumerate(dataloader), total=num_batches):
        #for i, batch in enumerate(dataloader, 0):
            counter += 1
            # collect the samples as a batch:
            scans = batch['scan']
            scans = scans.to(device)
            intensities = batch['intensity']
            intensities = intensities.to(device)
            angle_incidence = batch['angle_incidence']
            angle_incidence = angle_incidence.to(device)
            if not (
                torch.isfinite(scans).all()
                and torch.isfinite(intensities).all()
                and torch.isfinite(angle_incidence).all()
            ):
                raise ValueError("non-finite S3-Net validation input before model forward")
            labels = batch['label']
            labels = labels.to(device).long()

            batch_size = scans.size(0)

            # feed the batch to the network:
            semantic_scan, semantic_channels, kl_loss = model(scans, intensities, angle_incidence)
            # calculate the semantic ce loss:
            ce_loss = ce_criterion(semantic_channels, labels).div(batch_size)
            lovasz_loss, _ = lovasz_criterion(semantic_channels, labels)
            lovasz_loss = lovasz_loss.mul(class_weights.to(device)).sum()
            # beta-vae:
            loss = ce_loss + BETA*kl_loss + lovasz_loss
            # multiple GPUs:
            if torch.cuda.device_count() > 1:
                loss = loss.mean()
                ce_loss = ce_loss.mean()
                kl_loss = lovasz_loss.mean() #kl_loss.mean()

            running_loss += loss.item()
            # kl_divergence:
            kl_avg_loss += lovasz_loss.item() #kl_loss.item()
            # CE loss:
            ce_avg_loss += ce_loss.item()

    val_loss = running_loss / max(counter, 1)
    val_kl_loss = kl_avg_loss / max(counter, 1)
    val_ce_loss = ce_avg_loss / max(counter, 1)

    return val_loss, val_kl_loss, val_ce_loss


def _state_dict(model):
    """Return model state dict, handling DataParallel wrapper."""
    if torch.cuda.device_count() > 1:
        return model.module.state_dict()
    return model.state_dict()


def _make_checkpoint(model, optimizer, epoch, train_loss, dev_loss, best_dev_loss,
                     dataset_root, stats_json, batch_size, num_epochs):
    """Build a checkpoint dict with metadata and backward-compatible keys."""
    model_state_dict = _state_dict(model)
    optimizer_state_dict = optimizer.state_dict()
    return {
        # New-style metadata keys
        'model_state_dict': model_state_dict,
        'optimizer_state_dict': optimizer_state_dict,
        'epoch': epoch,
        'train_loss': train_loss,
        'dev_loss': dev_loss,
        'best_dev_loss': best_dev_loss,
        'dataset_root': dataset_root,
        'stats_json': stats_json,
        'batch_size': batch_size,
        'num_epochs': num_epochs,
        'feature_mode': S3NET_FEATURE_MODE,
        'input_channels': NUM_INPUT_CHANNELS,
        'num_output_channels': NUM_OUTPUT_CHANNELS,
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        # Backward-compatible keys for existing loaders/resume code
        'model': model_state_dict,
        'optimizer': optimizer_state_dict,
    }


def _save_checkpoint(path, checkpoint):
    """Save checkpoint, creating parent directories if needed."""
    odir = os.path.dirname(path)
    if odir and not os.path.exists(odir):
        os.makedirs(odir)
    torch.save(checkpoint, path)


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
    # ensure we have the correct amount of arguments:
    #global cur_batch_win
    if(len(argv) != NUM_ARGS):
        print("usage: python train.py [MDL_PATH] [TRAIN_PATH] [DEV_PATH] [TRAIN_MASK_PATH] [DEV_MASK_PATH]")
        exit(-1)

    # define local variables:
    mdl_path = argv[0]
    pTrain = argv[1]
    pDev = argv[2]

    # get the output directory name:
    odir = os.path.dirname(mdl_path)

    # if the odir doesn't exits, we make it:
    if not os.path.exists(odir):
        os.makedirs(odir)

    # set the device to use GPU if available:
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    stats_path = S3NET_STATS_PATH
    if stats_path:
        print("using S3NET_STATS_JSON:", stats_path)

    print('...Start reading data...')
    ### training data ###
    # training set and training data loader
    train_dataset = VaeTestDataset(pTrain, 'train', stats_path=stats_path)
    train_dataloader = torch.utils.data.DataLoader(train_dataset, batch_size=BATCH_SIZE, num_workers=4, \
                                                   shuffle=True, drop_last=False, pin_memory=True)

    ### validation data ###
    # validation set and validation data loader
    dev_dataset = VaeTestDataset(pDev, 'dev', stats_path=stats_path)
    dev_dataloader = torch.utils.data.DataLoader(dev_dataset, batch_size=BATCH_SIZE, num_workers=2, \
                                                 shuffle=True, drop_last=False, pin_memory=True)

    # calculate the class weights:
    class_weights = np.ones(NUM_OUTPUT_CHANNELS, dtype=np.float32)
    if NUM_OUTPUT_CHANNELS == 10:
        class_weights = np.array([2.514399, 1.4917144, 0.51608694, 0.659483, 1.0900991, 1.6461798, 0.32852992, 1.5633508, 0.9236576, 0.10251398])
    if stats_path:
        with open(stats_path, "r") as fp:
            stats = json.load(fp)
        if "class_weights" in stats:
            class_weights = np.array(stats["class_weights"], dtype=np.float32)
    if class_weights.shape != (NUM_OUTPUT_CHANNELS,):
        raise ValueError(
            f"class_weights shape {class_weights.shape} does not match "
            f"S3NET_NUM_CLASSES={NUM_OUTPUT_CHANNELS}"
        )

    #class_weights = np.array([1.4222778, 2.1834621, 40.17538]) # inverse log class_probability
    class_weights = torch.Tensor(class_weights)
    print("class weights: ", class_weights)
    class_weights = class_weights.to(device)
    print('...Finish reading data...')

    # instantiate a model:
    model = S3Net(input_channels=NUM_INPUT_CHANNELS,
                 output_channels=NUM_OUTPUT_CHANNELS,
                 feature_mode=S3NET_FEATURE_MODE)
    # moves the model to device (cpu in our case so no change):
    model.to(device)

    # set the adam optimizer parameters:
    opt_params = { LEARNING_RATE: 0.001,
                   BETAS: (.9,0.999),
                   EPS: 1e-08,
                   WEIGHT_DECAY: .001 }
    # set the loss criterion and optimizer:
    ce_criterion = nn.CrossEntropyLoss(reduction='sum', weight=class_weights, ignore_index=IGNORE_LABEL)
    ce_criterion.to(device)
    lovasz_criterion = L.LovaszSoftmax(reduction='sum', ignore_index=IGNORE_LABEL)
    lovasz_criterion.to(device)
    # create an optimizer, and pass the model params to it:
    optimizer = Adam(model.parameters(), **opt_params)

    # get the number of epochs to train on:
    epochs = NUM_EPOCHS

    # if there are trained models, continue training:
    if os.path.exists(mdl_path):
        checkpoint = torch.load(mdl_path)
        model.load_state_dict(checkpoint.get('model_state_dict', checkpoint['model']))
        optimizer.load_state_dict(checkpoint.get('optimizer_state_dict', checkpoint['optimizer']))
        start_epoch = checkpoint['epoch']
        print('Load epoch {} success'.format(start_epoch))
    else:
        start_epoch = 0
        #pre_path = "./model/model_segnet_weight.pth"
        #pretrained_model = torch.load(pre_path)
        #model.load_state_dict(pretrained_model['model'])
        print('No trained models, restart training')

    # multiple GPUs:
    if torch.cuda.device_count() > 1:
        print("Let's use 2 of total", torch.cuda.device_count(), "GPUs!")
        # dim = 0 [30, xxx] -> [10, ...], [10, ...], [10, ...] on 3 GPUs
        model = nn.DataParallel(model) #, device_ids=[0, 1])
    # moves the model to device (cpu in our case so no change):
    model.to(device)

    # tensorboard writer: write to RESULT_DIR/runs, not work-dir runs/
    runs_dir = os.path.join(odir, 'runs')
    os.makedirs(runs_dir, exist_ok=True)
    writer = SummaryWriter(runs_dir)

    # Checkpoint paths
    latest_path = os.path.join(odir, 's3net_native_stats_latest.pth')
    best_dev_path = os.path.join(odir, 's3net_native_stats_best_dev.pth')
    final_path = os.path.join(odir, 's3net_native_stats_final.pth')
    best_dev_loss = float('inf')

    # Stats path for metadata
    stats_json_path = stats_path or ''

    epoch_num = start_epoch
    for epoch in range(start_epoch+1, epochs+1):
        # adjust learning rate:
        adjust_learning_rate(optimizer, epoch)
        ################################## Train #####################################
        # for each batch in increments of batch size
        #
        train_epoch_loss, train_kl_epoch_loss, train_ce_epoch_loss = train(
            model, train_dataloader, train_dataset, device, optimizer, ce_criterion, lovasz_criterion, class_weights, epoch, epochs
        )
        valid_epoch_loss, valid_kl_epoch_loss, valid_ce_epoch_loss = validate(
            model, dev_dataloader, dev_dataset, device, ce_criterion, lovasz_criterion, class_weights
        )

        # log the epoch loss
        writer.add_scalar('training loss',
                        train_epoch_loss,
                        epoch)
        writer.add_scalar('training kl loss',
                        train_kl_epoch_loss,
                        epoch)
        writer.add_scalar('training ce loss',
                train_ce_epoch_loss,
                epoch)

        writer.add_scalar('validation loss',
                        valid_epoch_loss,
                        epoch)
        writer.add_scalar('validation kl loss',
                        valid_kl_epoch_loss,
                        epoch)
        writer.add_scalar('validation ce loss',
                        valid_ce_epoch_loss,
                        epoch)

        print('Train set: Average loss: {:.4f}'.format(train_epoch_loss))
        print('Validation set: Average loss: {:.4f}'.format(valid_epoch_loss))

        if not np.isfinite(train_epoch_loss) or not np.isfinite(valid_epoch_loss):
            raise ValueError(
                'non-finite loss at epoch {}: train={}, dev={}'.format(
                    epoch, train_epoch_loss, valid_epoch_loss
                )
            )

        # Track best dev loss
        saved_best_dev = valid_epoch_loss < best_dev_loss
        if saved_best_dev:
            best_dev_loss = valid_epoch_loss

        # Build checkpoint
        state = _make_checkpoint(
            model, optimizer, epoch, train_epoch_loss, valid_epoch_loss,
            best_dev_loss, pTrain.rstrip(os.sep), stats_json_path, BATCH_SIZE, epochs
        )

        # Save latest and best_dev every epoch
        _save_checkpoint(latest_path, state)
        if saved_best_dev:
            _save_checkpoint(best_dev_path, state)

        # Save periodic checkpoint
        periodic_path = ''
        if CHECKPOINT_INTERVAL > 0 and epoch % CHECKPOINT_INTERVAL == 0:
            periodic_path = os.path.join(
                odir, 's3net_native_stats_epoch_{:04d}.pth'.format(epoch)
            )
            _save_checkpoint(periodic_path, state)

        print('Epoch [{}/{}]'.format(epoch, epochs))
        print('Best dev loss: {:.4f}'.format(best_dev_loss))
        print('Saved latest: {}'.format(latest_path))
        print('Saved best_dev: {}'.format('yes' if saved_best_dev else 'no'))
        print('Saved periodic checkpoint: {}'.format(periodic_path if periodic_path else 'no'))

        epoch_num = epoch

    # Save the final model (both as the main model path and as final.pth)
    state = _make_checkpoint(
        model, optimizer, epoch_num, train_epoch_loss, valid_epoch_loss,
        best_dev_loss, pTrain.rstrip(os.sep), stats_json_path, BATCH_SIZE, epochs
    )
    _save_checkpoint(mdl_path, state)
    _save_checkpoint(final_path, state)
    print('Saved final: {}'.format(final_path))
    if mdl_path != final_path:
        print('Saved run model: {}'.format(mdl_path))

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
