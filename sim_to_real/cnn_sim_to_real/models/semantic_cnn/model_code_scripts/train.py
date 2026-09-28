#!/usr/bin/env python
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：PT
# 可能使用的关键环境变量：BATCH_SIZE, BETAS, CHECKPOINT_INTERVAL, DEV_PATH, LEARNING_RATE, MDL_PATH, NUM_ARGS, NUM_EPOCHS, SEED1, SEMANTIC_CNN_STOP_LOSS_WEIGHT, STOP_LOSS_WEIGHT, STOP_TARGET_EPSILON, TARGET_NAME, TRAIN_PATH, WEIGHT_DECAY
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Python 工具/训练/转换脚本
# 推荐运行方式：python3 /home/user/navigation_project/a_pipeline/methods/baselines/semantic_cnn/training/scripts/train.py
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.784307748 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:00.812228331 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/train.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/s3net/20260727_162813_s3net_native_stats_301epoch/model_code_scripts/train.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/s3net/20260727_162813_s3net_native_stats_301epoch/work/s3_net_v7/scripts/train.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/training/s3net/20260717_172858_s3net_native_stats_301epoch/model_code_scripts/train.py（正文中引用该脚本路径/名称）; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/training/s3net/20260717_172858_s3net_native_stats_301epoch/work/s3_net_v7/scripts/train.py（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/baselines/semantic_cnn/training/scripts/train.py
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/train.py; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/s3net/20260727_162813_s3net_native_stats_301epoch/model_code_scripts/train.py; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/datasets/20260727_three_bag_online_seed_split_v1/training/s3net/20260727_162813_s3net_native_stats_301epoch/work/s3_net_v7/scripts/train.py; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/training/s3net/20260717_172858_s3net_native_stats_301epoch/model_code_scripts/train.py; /home/user/navigation_project/a_pipeline/runs/20260717_042135_v7_dual/training/s3net/20260717_172858_s3net_native_stats_301epoch/work/s3_net_v7/scripts/train.py
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
# This script trains a Semantic CNN model
#------------------------------------------------------------------------------

# import pytorch modules
#
import torch
import torch.nn as nn
from torch.optim import Adam
from tqdm import tqdm

# visualize:
from tensorboardX import SummaryWriter
import numpy as np

# import the model and all of its variables/functions
#
from model import *

# import modules
#
import sys
import os
from datetime import datetime


#-----------------------------------------------------------------------------
#
# global variables are listed here
#
#-----------------------------------------------------------------------------

# general global values
#
model_dir = './model/semantic_cnn_model.pth'  # the path of model storage 
NUM_ARGS = 3
NUM_EPOCHS = 4000 
BATCH_SIZE = 64 
CHECKPOINT_INTERVAL = 10
TARGET_NAME = "cmd_velocities/[linear_x, angular_z]"
STOP_LOSS_WEIGHT = float(os.environ.get("SEMANTIC_CNN_STOP_LOSS_WEIGHT", "1.0"))
STOP_TARGET_EPSILON = 1e-4
LEARNING_RATE = "lr"
BETAS = "betas"
EPS = "eps"
WEIGHT_DECAY = "weight_decay"

# for reproducibility, we seed the rng
#
set_seed(SEED1)       

# adjust_learning_rate
#　
def adjust_learning_rate(optimizer, epoch):
    lr = 1e-3
    if epoch > 40:
        lr = 2e-4
    if epoch > 2000:
        lr = 2e-5
    if epoch > 21000:
        lr = 1e-5
    if epoch > 32984:
        lr = 1e-6
    if epoch > 48000:
       # lr = 5e-8
       lr = lr * (0.1 ** (epoch // 110000))
    #  if epoch > 8300:
    #      lr = 1e-9
    for param_group in optimizer.param_groups:
        param_group['lr'] = lr



# train function:
def control_regression_loss(output, velocities):
    """Sum squared control error with an optional weight for stop targets."""
    if STOP_LOSS_WEIGHT < 0.0:
        raise ValueError("STOP_LOSS_WEIGHT must be non-negative")
    per_sample = torch.sum((output - velocities) ** 2, dim=1)
    is_stop = torch.all(torch.abs(velocities) < STOP_TARGET_EPSILON, dim=1)
    weights = torch.ones_like(per_sample)
    weights[is_stop] = STOP_LOSS_WEIGHT
    return torch.sum(per_sample * weights)


def train(model, dataloader, dataset, device, optimizer, epoch, epochs):
    ################################## Train #####################################
    # Set model to training mode
    model.train()  
    # for each batch in increments of batch size
    #
    running_loss = 0
    counter = 0
    # get the number of batches (ceiling of train_data/batch_size):
    num_batches = (len(dataset) + dataloader.batch_size - 1) // dataloader.batch_size
    for i, batch in tqdm(enumerate(dataloader), total=num_batches):
    #for i, batch in enumerate(dataloader, 0):
        counter += 1
        # collect the samples as a batch:
        scan_maps = batch['scan_map']
        scan_maps = scan_maps.to(device)
        semantic_maps = batch['semantic_map']
        semantic_maps = semantic_maps.to(device)
        sub_goals = batch['sub_goal']
        sub_goals = sub_goals.to(device)
        velocities = batch['velocity']
        velocities = velocities.to(device)

        # set all gradients to 0:
        optimizer.zero_grad()
        # feed the network the batch
        #
        
        output = model(scan_maps, semantic_maps, sub_goals)
        #writer.add_graph(model,[batch_ped_pos_t, batch_scan_t, batch_goal_t])    
        loss = control_regression_loss(output, velocities)

        # perform back propagation:
        loss.backward(torch.ones_like(loss))
        optimizer.step()
        # get the loss:
        # multiple GPUs:
        if torch.cuda.device_count() > 1:
            loss = loss.mean()  

        running_loss += loss.item()
        
        # display informational message
        #
        if(i % 1280 == 0):
            print('Epoch [{}/{}], Step[{}/{}], Loss: {:.4f}'
                    .format(epoch, epochs, i + 1, num_batches, loss.item()))

    train_loss = running_loss / len(dataset) #counter 

    return train_loss

# validate function:
def validate(model, dataloader, dataset, device):
    ################################## Train #####################################
    # set model to evaluation mode:
    model.eval()
    # for each batch in increments of batch size
    #
    running_loss = 0
    counter = 0
    # get the number of batches (ceiling of train_data/batch_size):
    num_batches = (len(dataset) + dataloader.batch_size - 1) // dataloader.batch_size
    for i, batch in tqdm(enumerate(dataloader), total=num_batches):
    #for i, batch in enumerate(dataloader, 0):
        counter += 1
        # collect the samples as a batch:
        scan_maps = batch['scan_map']
        scan_maps = scan_maps.to(device)

        semantic_maps = batch['semantic_map']
        semantic_maps = semantic_maps.to(device)
 
        sub_goals = batch['sub_goal']
        sub_goals = sub_goals.to(device)
        velocities = batch['velocity']
        velocities = velocities.to(device)

        # feed the network the batch
        #
        output = model(scan_maps, semantic_maps, sub_goals)
        #writer.add_graph(model,[batch_ped_pos_t, batch_scan_t, batch_goal_t])    
        loss = control_regression_loss(output, velocities)
            
        # get the loss:
        # multiple GPUs:
        if torch.cuda.device_count() > 1:
            loss = loss.mean()  

        running_loss += loss.item()

    val_loss = running_loss / len(dataset) #counter 

    return val_loss


def _state_dict(model):
    if torch.cuda.device_count() > 1:
        return model.module.state_dict()
    return model.state_dict()


def _make_checkpoint(model, optimizer, epoch, train_loss, dev_loss, best_dev_loss,
                     dataset_root, batch_size, num_epochs, normalization):
    model_state_dict = _state_dict(model)
    optimizer_state_dict = optimizer.state_dict()
    return {
        'model_state_dict': model_state_dict,
        'optimizer_state_dict': optimizer_state_dict,
        'epoch': epoch,
        'train_loss': train_loss,
        'dev_loss': dev_loss,
        'best_dev_loss': best_dev_loss,
        'dataset_root': dataset_root,
        'target': TARGET_NAME,
        'batch_size': batch_size,
        'num_epochs': num_epochs,
        'normalization': normalization,
        'timestamp': datetime.now().isoformat(timespec='seconds'),
        # Keep the original checkpoint keys for existing loaders/resume code.
        'model': model_state_dict,
        'optimizer': optimizer_state_dict,
    }


def _save_checkpoint(path, checkpoint):
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

    # ensure we have the correct amount of arguments
    #
    #global cur_batch_win
    if(len(argv) != NUM_ARGS):
        print("usage: python nedc_train_mdl.py [MDL_PATH] [TRAIN_PATH] [DEV_PATH]")
        exit(-1)

    # define local variables
    #
    mdl_path = argv[0]
    pTrain = argv[1]
    pDev = argv[2]

    # get the output directory name
    #
    odir = os.path.dirname(mdl_path)

    # if the odir doesn't exits, we make it
    #
    if not os.path.exists(odir):
        os.makedirs(odir)

    # set the device to use GPU if available
    #
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    ### train:
    print('...Start reading data...')
    # get array of the data
    # data: [[0, 1, ... 26], [27, 28, ...] ...]
    # labels: [0, 0, 1, ...]
    #
    #[ped_pos_t, scan_t, goal_t, vel_t] = get_data(pTrain)
    train_dataset = NavDataset(pTrain, 'train')
    if len(train_dataset) == 0:
        raise ValueError("SemanticCNN train split has no valid sequence windows")
    train_dataloader = torch.utils.data.DataLoader(train_dataset, batch_size=BATCH_SIZE, \
                                                   shuffle=True, drop_last=False, pin_memory=True)
    #train_data = train_data - np.mean(train_data, axis=0)
    
    ### dev:

    # get array of the data
    # data: [[0, 1, ... 26], [27, 28, ...] ...]
    # labels: [0, 0, 1, ...]
    #
    #[ped_pos_d, scan_d, goal_d, vel_d] = get_data(pDev)
    dev_dataset = NavDataset(pDev, 'dev')
    if len(dev_dataset) == 0:
        raise ValueError("SemanticCNN dev split has no valid sequence windows")
    dev_dataloader = torch.utils.data.DataLoader(dev_dataset, batch_size=BATCH_SIZE, \
                                                   shuffle=True, drop_last=False, pin_memory=True)
    if not np.allclose(train_dataset.g_mu, dev_dataset.g_mu, rtol=0.0, atol=0.0):
        raise ValueError("train/dev SemanticCNN sub-goal means differ")
    if not np.allclose(train_dataset.g_std, dev_dataset.g_std, rtol=0.0, atol=0.0):
        raise ValueError("train/dev SemanticCNN sub-goal standard deviations differ")
    normalization = {
        'source': train_dataset.normalization_source,
        'stats_json': train_dataset.normalization_stats_path,
        'sub_goal_mean': train_dataset.g_mu.tolist(),
        'sub_goal_std': train_dataset.g_std.tolist(),
    }
    #dev_data = dev_data - np.mean(dev_data, axis=0)
    print('...Finish reading data...')

    # instantiate a model
    #
    model = SemanticCNN(Bottleneck, [2, 1, 1])

    # moves the model to device (cpu in our case so no change)
    #
    model.to(device)

    # set the adam optimizer parameters
    #
    opt_params = { LEARNING_RATE: 0.001,
                   BETAS: (.9,0.999),
                   EPS: 1e-08,
                   WEIGHT_DECAY: .001 }

    # create an optimizer, and pass the model params to it
    #
    optimizer = Adam(model.parameters(), **opt_params)

    # get the number of epochs to train on
    #
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
        print('No trained models, restart training')

    # multiple GPUs:
    if torch.cuda.device_count() > 1:
        print("Let's use 2 of total", torch.cuda.device_count(), "GPUs!")
        # dim = 0 [30, xxx] -> [10, ...], [10, ...], [10, ...] on 3 GPUs
        model = nn.DataParallel(model) #, device_ids=[0, 1])

    # moves the model to device (cpu in our case so no change)
    #
    model.to(device)
    print('Stop target loss weight: {}'.format(STOP_LOSS_WEIGHT))

    # tensorboard writer:
    writer = SummaryWriter('runs')
    latest_path = os.path.join(odir, 'semantic_cnn_native_cmd_latest.pth')
    best_dev_path = os.path.join(odir, 'semantic_cnn_native_cmd_best_dev.pth')
    final_path = os.path.join(odir, 'semantic_cnn_native_cmd_final.pth')
    best_dev_loss = float('inf')

    # for each epoch
    #
    #loss_train = []
    #loss_vector = []
    epoch_num = start_epoch
    for epoch in range(start_epoch + 1, epochs + 1):

        # adjust learning rate:
        adjust_learning_rate(optimizer, epoch)
        ################################## Train #####################################
        # for each batch in increments of batch size
        #
        train_epoch_loss = train(
            model, train_dataloader, train_dataset, device, optimizer, epoch, epochs
        )
        
        ################################## Test #####################################
        valid_epoch_loss = validate(
            model, dev_dataloader, dev_dataset, device
        )

        # log the epoch loss
        writer.add_scalar('training loss',
                        train_epoch_loss,
                        epoch)
        writer.add_scalar('validation loss',
                        valid_epoch_loss,
                        epoch)

        print('Train set: Average loss: {:.4f}'.format(train_epoch_loss))
        print('Validation set: Average loss: {:.4f}'.format(valid_epoch_loss))

        if not np.isfinite(train_epoch_loss) or not np.isfinite(valid_epoch_loss):
            raise ValueError(
                'non-finite loss at epoch {}: train={}, dev={}'.format(
                    epoch, train_epoch_loss, valid_epoch_loss
                )
            )

        saved_best_dev = valid_epoch_loss < best_dev_loss
        if saved_best_dev:
            best_dev_loss = valid_epoch_loss

        state = _make_checkpoint(
            model, optimizer, epoch, train_epoch_loss, valid_epoch_loss,
            best_dev_loss, pTrain.rstrip(os.sep), BATCH_SIZE, epochs,
            normalization
        )
        _save_checkpoint(latest_path, state)
        if saved_best_dev:
            _save_checkpoint(best_dev_path, state)

        periodic_path = ''
        if CHECKPOINT_INTERVAL > 0 and epoch % CHECKPOINT_INTERVAL == 0:
            periodic_path = os.path.join(
                odir, 'semantic_cnn_native_cmd_epoch_{:04d}.pth'.format(epoch)
            )
            _save_checkpoint(periodic_path, state)

        print('Epoch [{}/{}]'.format(epoch, epochs))
        print('Best dev loss: {:.4f}'.format(best_dev_loss))
        print('Saved latest: {}'.format(latest_path))
        print('Saved best_dev: {}'.format('yes' if saved_best_dev else 'no'))
        print('Saved periodic checkpoint: {}'.format(periodic_path if periodic_path else 'no'))
        
        epoch_num = epoch

    # save the final model
    state = _make_checkpoint(
        model, optimizer, epoch_num, train_epoch_loss, valid_epoch_loss,
        best_dev_loss, pTrain.rstrip(os.sep), BATCH_SIZE, epochs,
        normalization
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
