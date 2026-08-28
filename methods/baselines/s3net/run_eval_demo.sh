#!/bin/sh
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：PT
# 可能使用的关键环境变量：DL_EXP, DL_MDL_PATH, DL_OUT, DL_SCRIPTS, DL_TRAIN_ODIR, NARGS
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/methods/baselines/s3net/run_eval_demo.sh
# 输入来源：命令行参数、数据文件、模型文件或 ROS 2 状态。
# 输出结果：检查报告、转换数据、训练模型、可视化结果或日志。
# 前置条件：需要对应 Python 环境、输入文件和依赖库。
# 后续步骤：输出由上层 pipeline 或人工分析继续使用。
# 副作用与安全：通常写入输出文件；运行前确认不会覆盖重要数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.733305586 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:18:51.813067201 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/decode_demo.py（正文中引用该脚本路径/名称）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/run_eval_demo.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/methods/baselines/s3net/scripts/decode_demo.py
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
#
# file: run_demo.sh
#
# This is a simple driver script that runs training and then decoding
# on the training set and the val set.
#
# To run this script, execute the following line:
#
#  run_demo.sh train.dat val.dat
#
# The first argument ($1) is the training data. The last two arguments,
# test data ($2) and evaluation data ($3) are optional.
#
# An example of how to run this is as follows:
#
# xzt: echo $PWD
# /home/xzt/SOGMP
# xzt: sh run_demo.sh ~/semantic2d_data/
#

# decode the number of command line arguments
#
NARGS=$#

if (test "$NARGS" -eq "0") then
    echo "usage: run_demo.sh test.dat"
    exit 1
fi

# define a base directory for the experiment
#
DL_EXP=`pwd`;
DL_SCRIPTS="$DL_EXP/scripts";
DL_OUT="$DL_EXP/output";

# define the output directories for training/decoding/scoring
#
#DL_TRAIN_ODIR="$DL_OUT/00_train";
DL_TRAIN_ODIR="$DL_EXP/model";
DL_MDL_PATH="$DL_TRAIN_ODIR/s3_net_model.pth";

# create the output directory
#
mkdir -p $DL_OUT

# evaluate each data set that was specified
#
echo "... starting evaluation of $1..."
$DL_SCRIPTS/decode_demo.py $DL_OUT $DL_MDL_PATH $1 | \
    tee $DL_OUT/01_decode_dev.log | grep "00 out of\|Average"
echo "... finished evaluation of $1 ..."


echo "======= end of results ======="

#
# exit gracefully
