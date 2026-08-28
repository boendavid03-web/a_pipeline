#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：--index-url, --upgrade
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：TXT
# 可能使用的关键环境变量：BASH_SOURCE, ERROR, NVIDIA, PASS, ROOT
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/environment/02_create_python_envs.sh
# 输入来源：操作系统、Python、ROS 2 和项目目录状态。
# 输出结果：虚拟环境、ROS 2 构建结果或环境配置。
# 前置条件：需要系统权限或已安装基础工具。
# 后续步骤：完成后 source 环境并运行项目 pipeline。
# 副作用与安全：可能安装依赖、创建环境或构建工作空间。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:45:42.976870135 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:18:34.391758929 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/04c_open_labelme.sh（正文中引用该脚本路径/名称）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/environment/02_create_python_envs.sh
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/04c_open_labelme.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REQ="${ROOT}/environment/requirements"

STALE_TAG="$(date +%Y%m%d_%H%M%S)"

prepare_venv() {
  local venv_dir="$1"
  local entrypoint="$2"
  local entrypoint_path="${venv_dir}/bin/${entrypoint}"
  local expected_shebang="#!${venv_dir}/bin/python"
  local actual_shebang=""
  local stale_dir

  if [[ -d "${venv_dir}" ]]; then
    if [[ -f "${entrypoint_path}" ]]; then
      actual_shebang="$(head -n 1 "${entrypoint_path}")"
    fi
    if [[ "${actual_shebang}" != "${expected_shebang}" ]]; then
      stale_dir="${venv_dir}.stale-${STALE_TAG}"
      echo "Detected a copied or incomplete virtual environment:"
      echo "  ${venv_dir}"
      echo "Moving it aside before rebuilding:"
      echo "  ${stale_dir}"
      mv "${venv_dir}" "${stale_dir}"
    fi
  fi

  python3 -m venv "${venv_dir}"
}

prepare_venv "${ROOT}/.venvs/labelme" labelme
"${ROOT}/.venvs/labelme/bin/python" -m pip install --upgrade pip
"${ROOT}/.venvs/labelme/bin/python" -m pip install -r "${REQ}/labelme.lock.txt"

prepare_venv "${ROOT}/.venvs/train" torchrun
"${ROOT}/.venvs/train/bin/python" -m pip install --upgrade pip
"${ROOT}/.venvs/train/bin/python" -m pip install \
  torch==2.11.0 torchvision==0.26.0 torchaudio==2.11.0 \
  --index-url https://download.pytorch.org/whl/cu128
"${ROOT}/.venvs/train/bin/python" -m pip install -r "${REQ}/training.lock.txt"

"${ROOT}/.venvs/labelme/bin/python" -c 'import labelme; print("labelme", labelme.__version__)'
test "$(head -n 1 "${ROOT}/.venvs/labelme/bin/labelme")" = \
  "#!${ROOT}/.venvs/labelme/bin/python"
test "$(head -n 1 "${ROOT}/.venvs/train/bin/torchrun")" = \
  "#!${ROOT}/.venvs/train/bin/python"
"${ROOT}/.venvs/train/bin/python" - <<'PY'
import torch
print("torch", torch.__version__)
print("cuda_available", torch.cuda.is_available())
if not torch.cuda.is_available():
    raise SystemExit("ERROR: PyTorch cannot see the NVIDIA GPU")
print("gpu", torch.cuda.get_device_name(0))
PY
echo "PASS: project-local Python environments created"
