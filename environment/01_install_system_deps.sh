#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：未检测到固定扩展名；通常由命令行路径或配置决定。
# 可能使用的关键环境变量：ERROR, PASS, ROS_APT_DEB, ROS_APT_SOURCE_VERSION, VERSION_CODENAME, VERSION_ID
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/environment/01_install_system_deps.sh
# 输入来源：操作系统、Python、ROS 2 和项目目录状态。
# 输出结果：虚拟环境、ROS 2 构建结果或环境配置。
# 前置条件：需要系统权限或已安装基础工具。
# 后续步骤：完成后 source 环境并运行项目 pipeline。
# 副作用与安全：可能安装依赖、创建环境或构建工作空间。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:45:42.975870093 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:18:34.391758929 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：未检测到代码正文中的其他项目脚本调用；主要依赖外部库、ROS 2 节点或系统命令。
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/environment/01_install_system_deps.sh
# 直接依赖（脚本层面）：无代码正文中的项目脚本依赖。
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
set -euo pipefail

source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "ERROR: this installer supports Ubuntu 22.04 only." >&2
  exit 2
fi

# Some older installations used a regional ROS mirror that no longer serves
# Ubuntu Jammy metadata. Disable only matching ROS source entries, preserving a
# timestamped backup, so apt can recover and the official ros2-apt-source
# package below can install the current ROS repository configuration.
STALE_ROS_MIRROR='mirrors.ustc.edu.cn/ros/ubuntu'
APT_BACKUP_DIR='/var/backups/a_pipeline'
sudo install -d -m 0755 "${APT_BACKUP_DIR}"

# Move backups created by an earlier version of this installer out of
# sources.list.d; apt warns about files there whose extension is not .list or
# .sources.
for legacy_backup in /etc/apt/sources.list.d/*.a_pipeline-backup-*; do
  [[ -e "${legacy_backup}" ]] || continue
  sudo mv -- "${legacy_backup}" "${APT_BACKUP_DIR}/$(basename "${legacy_backup}")"
done

while IFS= read -r -d '' source_file; do
  if grep -qF "${STALE_ROS_MIRROR}" "${source_file}"; then
    backup_file="${APT_BACKUP_DIR}/$(basename "${source_file}").a_pipeline-backup-$(date +%Y%m%d_%H%M%S)"
    sudo cp -a -- "${source_file}" "${backup_file}"
    sudo sed -i \
      "\#${STALE_ROS_MIRROR}#s#^[[:space:]]*#\\# disabled by a_pipeline: #" \
      "${source_file}"
    echo "WARN: disabled obsolete ROS mirror in ${source_file}; backup: ${backup_file}"
  fi
done < <(
  find /etc/apt -maxdepth 2 -type f \( \
    -path '/etc/apt/sources.list' -o \
    -path '/etc/apt/sources.list.d/*.list' -o \
    -path '/etc/apt/sources.list.d/*.sources' \
  \) -print0
)

sudo apt-get update

# Ubuntu Jammy may already provide older monolithic Python ROS packages. The
# ROS repository's newer split packages contain the same files, so dpkg would
# otherwise fail with "trying to overwrite" while unpacking them. Repair this
# before any apt install command, because an interrupted previous ROS install
# can make even an otherwise unrelated apt install fail.
LEGACY_ROS_PY_PACKAGES=(
  python3-catkin-pkg
  python3-rospkg
  python3-rosdistro
)
for package in "${LEGACY_ROS_PY_PACKAGES[@]}"; do
  if dpkg-query -W -f='${Status}' "${package}" 2>/dev/null | grep -Fq 'install ok installed'; then
    echo "WARN: removing conflicting legacy package ${package}"
    sudo dpkg --remove --force-depends "${package}"
  fi
done
sudo apt-get -f install -y

sudo apt-get install -y curl software-properties-common
sudo add-apt-repository -y universe

if ! apt-cache show ros-humble-ros-base >/dev/null 2>&1; then
  # Do not exit awk early: with pipefail enabled that can make curl report
  # CURLE_WRITE_ERROR (23) after the reader closes the pipe.
  ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | awk -F'"' '/"tag_name":/ && !found {print $4; found=1}')"
  if [[ -z "${ROS_APT_SOURCE_VERSION}" ]]; then
    echo "ERROR: could not determine ros-apt-source release." >&2
    exit 2
  fi
  ROS_APT_DEB="/tmp/ros2-apt-source.deb"
  curl -fL -o "${ROS_APT_DEB}" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
  sudo dpkg -i "${ROS_APT_DEB}"
fi

sudo apt-get update

sudo apt-get install -y \
  build-essential cmake git rsync zstd ubuntu-drivers-common python3-venv python3-pip \
  python3-numpy python3-pil python3-yaml \
  python3-colcon-common-extensions python3-rosdep \
  ros-humble-desktop ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
  ros-humble-slam-toolbox ros-humble-nav2-map-server \
  ros-humble-teleop-twist-keyboard ros-humble-rosbag2-storage-default-plugins \
  libignition-transport11-dev libignition-msgs8-dev libignition-gazebo6-dev

if command -v ubuntu-drivers >/dev/null 2>&1 && \
  ubuntu-drivers devices 2>/dev/null | grep -Eiq 'driver.*nvidia'; then
  if ! command -v nvidia-smi >/dev/null 2>&1 || ! nvidia-smi -L >/dev/null 2>&1; then
    echo "WARN: NVIDIA hardware is present but no usable driver is loaded; installing the recommended driver."
    sudo ubuntu-drivers install
    echo "ACTION REQUIRED: reboot this host, then rerun environment/00_check_host.sh."
  fi
fi

echo "PASS: system dependencies installed"
