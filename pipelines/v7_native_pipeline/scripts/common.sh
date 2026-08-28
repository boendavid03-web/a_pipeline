#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：/cmd_vel, /odom, /scan_merged
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, NPY, TXT
# 可能使用的关键环境变量：BAG_DIR, BASELINE_ROOT, BASH_REMATCH, BASH_SOURCE, BEGIN, COMMON_SH_DIR, DATASET_ROOT, DEFAULT_CMD_VEL_TOPIC, DEFAULT_DEV_RATIO, DEFAULT_IGN_PARTITION_PREFIX, DEFAULT_ODOM_TOPIC, DEFAULT_ROS_DOMAIN_ID, DEFAULT_SCAN_TOPIC, DEFAULT_SUBGOAL_LOOKAHEAD, DEFAULT_TARGET_POINTS, ERROR, IGN_PARTITION, LABELME_BIN, LABELME_PY, LAST_BAG_DIR
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.488295204 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:44.913035738 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/00_create_run.sh（正文中引用该脚本路径/名称）
# 被依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/00_create_run.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/01_start_v7_dual_fixed_map_capture.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/01_start_v7_dual_slam.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/02_check_topics.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/03_save_slam_map.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/04_prepare_labelme_map.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/04b_export_labelme_json.sh（source 加载公共环境变量/函数）; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/04c_open_labelme.sh（source 加载公共环境变量/函数）
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/00_create_run.sh
# 被依赖/被引用（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/00_create_run.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/01_start_v7_dual_fixed_map_capture.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/01_start_v7_dual_slam.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/02_check_topics.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/03_save_slam_map.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/04_prepare_labelme_map.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/04b_export_labelme_json.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/04c_open_labelme.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05_record_rosbag.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05b_teleop.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/05c_record_autonomous_profile.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/06_check_bag.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07_convert_bag_to_dataset_native_lidar.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07b_convert_bag_to_fixed_dual_lidar.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07bc_convert_export_all_raw_bags.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/07c_export_fixed_dual_training_dataset.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08_smoke_train.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08b_smoke_train_s3net.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/08c_smoke_fixed_dual_training.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/09_train_s3net_native_stats.sh; /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/10_train_semantic_cnn.sh
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜common.sh】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。

COMMON_SH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "${COMMON_SH_DIR}/../../.." && pwd)}"
PIPELINE_ROOT="${PIPELINE_ROOT:-${PROJECT_ROOT}/pipelines/v7_native_pipeline}"
METHODS_ROOT="${METHODS_ROOT:-${PROJECT_ROOT}/methods}"
BASELINE_ROOT="${BASELINE_ROOT:-${METHODS_ROOT}/baselines}"
S3NET_ROOT="${S3NET_ROOT:-${BASELINE_ROOT}/s3net}"
SEMANTIC_CNN_ROOT="${SEMANTIC_CNN_ROOT:-${BASELINE_ROOT}/semantic_cnn}"
RUNS_ROOT="${RUNS_ROOT:-${PROJECT_ROOT}/runs}"
# Backward-compatible name for scripts copied from the original project.
LEGACY_RUNS_ROOT="${LEGACY_RUNS_ROOT:-${RUNS_ROOT}}"
ROS_WS="${ROS_WS:-${PROJECT_ROOT}/workspaces/ros2_ws}"
TORCH_PY="${TORCH_PY:-${PROJECT_ROOT}/.venvs/train/bin/python}"
LABELME_PY="${LABELME_PY:-${PROJECT_ROOT}/.venvs/labelme/bin/python}"
LABELME_BIN="${LABELME_BIN:-${PROJECT_ROOT}/.venvs/labelme/bin/labelme}"
export PROJECT_ROOT PIPELINE_ROOT METHODS_ROOT BASELINE_ROOT S3NET_ROOT SEMANTIC_CNN_ROOT RUNS_ROOT LEGACY_RUNS_ROOT ROS_WS TORCH_PY LABELME_PY LABELME_BIN NAVIGATION_PROJECT_ROOT="${PROJECT_ROOT}"

DEFAULT_ROS_DOMAIN_ID="78"
DEFAULT_IGN_PARTITION_PREFIX="semantic_nav_v7_dual_self"
DEFAULT_SCAN_TOPIC="/scan_merged"
DEFAULT_ODOM_TOPIC="/odom"
DEFAULT_CMD_VEL_TOPIC="/cmd_vel"
DEFAULT_TARGET_POINTS="1081"
DEFAULT_DEV_RATIO="0.2"
DEFAULT_SUBGOAL_LOOKAHEAD="20"
V7_DUAL_BAG_SUFFIX="_v7_dual_teleop_bag"

timestamp() {
  date +%Y%m%d_%H%M%S
}

safe_source_ros() {
  local old_flags
  old_flags="$-"
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "${ROS_WS}/install/setup.bash"
  case "${old_flags}" in
    *u*) set -u ;;
  esac
}

load_manifest() {
  local manifest
  manifest="${1:-${RUN_MANIFEST:-}}"
  if [[ -z "${manifest}" ]]; then
    if [[ -n "${RUN_ROOT:-}" && -f "${RUN_ROOT}/run_manifest.env" ]]; then
      manifest="${RUN_ROOT}/run_manifest.env"
    else
      echo "ERROR: RUN_MANIFEST is not set and no run_manifest.env was found under RUN_ROOT." >&2
      echo "Run 00_create_run.sh first, or pass RUN_MANIFEST=/path/to/run_manifest.env." >&2
      exit 2
    fi
  fi
  if [[ ! -f "${manifest}" ]]; then
    echo "ERROR: manifest not found: ${manifest}" >&2
    exit 2
  fi
  # shellcheck disable=SC1090
  source "${manifest}"
}

require_file() {
  local path="$1"
  local label="${2:-file}"
  if [[ ! -f "${path}" ]]; then
    echo "ERROR: required ${label} not found: ${path}" >&2
    exit 2
  fi
}

require_dir() {
  local path="$1"
  local label="${2:-directory}"
  if [[ ! -d "${path}" ]]; then
    echo "ERROR: required ${label} not found: ${path}" >&2
    exit 2
  fi
}

dataset_num_classes() {
  local dataset_root="$1"
  local label_names_path="${dataset_root}/label_names.txt"
  local count
  require_file "${label_names_path}" "dataset label_names.txt"
  count="$(awk 'NF { count += 1 } END { print count + 0 }' "${label_names_path}")"
  if (( count < 2 )); then
    echo "ERROR: ${label_names_path} must contain _background_ plus at least one semantic class" >&2
    exit 2
  fi
  printf '%s\n' "${count}"
}

refresh_dataset_index() {
  local dataset_root="$1"
  local mode="${2:-baseline}"
  local action="${3:-write}"
  require_dir "${dataset_root}" "dataset root"
  if [[ "${action}" != "write" && "${action}" != "verify" ]]; then
    echo "ERROR: dataset index action must be write or verify, got: ${action}" >&2
    exit 2
  fi
  python3 - "${dataset_root}" "${mode}" "${action}" <<'PY'
import json
import math
import sys
from pathlib import Path

root = Path(sys.argv[1])
mode = sys.argv[2]
action = sys.argv[3]
index_path = root / "dataset.txt"

required_dirs = {
    "baseline": ["scans_lidar", "intensities_lidar", "semantic_label", "positions", "velocities"],
    "native": [
        "scans_lidar",
        "intensities_lidar",
        "angles_lidar",
        "valid_mask_lidar",
        "semantic_label",
        "positions",
        "velocities",
        "sub_goals_local",
    ],
    "native_cmd": [
        "scans_lidar",
        "intensities_lidar",
        "angles_lidar",
        "valid_mask_lidar",
        "semantic_label",
        "positions",
        "velocities",
        "cmd_velocities",
        "sub_goals_local",
    ],
    "fixed_dual_cmd": [
        "scans_lidar",
        "intensities_lidar",
        "angles_lidar",
        "virtual_ranges_lidar",
        "virtual_angles_lidar",
        "range_valid_mask_lidar",
        "self_mask_lidar",
        "valid_mask_lidar",
        "semantic_label",
        "source_sensor",
        "raw_beam_index",
        "positions",
        "velocities",
        "cmd_velocities",
        "sub_goals_local",
    ],
}[mode]

def session_status(name):
    session = root / name
    if not session.is_dir():
        return False, "missing session directory"
    for split in ("train.txt", "dev.txt", "test.txt"):
        if not (session / split).is_file():
            return False, f"missing {split}"
    for subdir in required_dirs:
        path = session / subdir
        if not path.is_dir():
            return False, f"missing {subdir}/"
    if mode in ("native_cmd", "fixed_dual_cmd"):
        scan_count = len(list((session / "scans_lidar").glob("*.npy")))
        cmd_count = len(list((session / "cmd_velocities").glob("*.npy")))
        if scan_count == 0:
            return False, "no scans_lidar samples"
        if scan_count != cmd_count:
            return False, f"cmd_velocities count {cmd_count} != scans_lidar count {scan_count}"
    if mode == "fixed_dual_cmd":
        metadata_path = session / "metadata.json"
        if not metadata_path.is_file():
            return False, "missing metadata.json"
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            return False, f"invalid metadata.json: {exc}"
        if metadata.get("format") != "semantic2d-fixed-dual-native-v3":
            return False, "unsupported fixed-dual metadata format"
        label_names_path = session / "label_names.txt"
        if not label_names_path.is_file():
            return False, "missing label_names.txt"
        label_names = [
            line.strip()
            for line in label_names_path.read_text(encoding="utf-8").splitlines()
            if line.strip()
        ]
        if label_names != metadata.get("label_names"):
            return False, "label_names.txt differs from metadata"
    return True, "ok"


def fixed_dual_contract_differences(reference, candidate):
    differences = []
    exact_keys = (
        "format",
        "samples_01",
        "samples_02",
        "total_slots",
        "slot_contract",
        "semantic_cnn_pool_mode",
        "pool_num_bins",
        "pool_range_normalization",
        "self_mask_mode",
        "forward_only",
        "reverse_recovery_frames",
        "label_names",
    )
    for key in exact_keys:
        if reference.get(key) != candidate.get(key):
            differences.append(key)
    for key in (
        "range_max_01",
        "range_max_02",
        "pool_range_max",
        "pool_angle_min",
        "pool_angle_max",
        "frame_period_tolerance_ms",
    ):
        try:
            matches = math.isclose(
                float(reference[key]), float(candidate[key]), rel_tol=0.0, abs_tol=1e-6
            )
        except (KeyError, TypeError, ValueError):
            matches = False
        if not matches:
            differences.append(key)
    if reference.get("forward_only") is True and candidate.get("forward_only") is True:
        try:
            matches = math.isclose(
                float(reference["reverse_linear_x_epsilon"]),
                float(candidate["reverse_linear_x_epsilon"]),
                rel_tol=0.0,
                abs_tol=1e-9,
            )
        except (KeyError, TypeError, ValueError):
            matches = False
        if not matches:
            differences.append("reverse_linear_x_epsilon")
    for key in ("expected_frame_period_ms", "scan_02_expected_frame_period_ms"):
        try:
            tolerance = min(
                float(reference["frame_period_tolerance_ms"]),
                float(candidate["frame_period_tolerance_ms"]),
            ) / 4.0
            matches = math.isclose(
                float(reference[key]), float(candidate[key]), rel_tol=0.0, abs_tol=tolerance
            )
        except (KeyError, TypeError, ValueError):
            matches = False
        if not matches:
            differences.append(key)
    return differences

names = []
if index_path.exists():
    names.extend(line.strip().rstrip("/") for line in index_path.read_text().splitlines())
names.extend(
    path.name
    for path in sorted(root.iterdir())
    if path.is_dir() and not path.name.startswith(".")
)

valid = []
skipped = []
seen = set()
for name in names:
    if not name or name in seen:
        continue
    seen.add(name)
    ok, reason = session_status(name)
    if ok:
        valid.append(name)
    else:
        skipped.append((name, reason))

if not valid:
    for name, reason in skipped:
        print(f"skip dataset session {name}: {reason}", file=sys.stderr)
    raise SystemExit(f"ERROR: no valid {mode} dataset sessions under {root}")

if mode == "fixed_dual_cmd":
    metadata_by_name = {
        name: json.loads((root / name / "metadata.json").read_text(encoding="utf-8"))
        for name in valid
    }
    reference_name = valid[0]
    reference = metadata_by_name[reference_name]
    for name in valid[1:]:
        differences = fixed_dual_contract_differences(reference, metadata_by_name[name])
        if differences:
            raise SystemExit(
                "ERROR: refusing to mix incompatible fixed-dual sessions: "
                f"{reference_name} vs {name}: {', '.join(differences)}"
            )
    root_label_names = root / "label_names.txt"
    if root_label_names.is_file():
        root_labels = [line.strip() for line in root_label_names.read_text(encoding="utf-8").splitlines() if line.strip()]
        if root_labels != reference.get("label_names"):
            raise SystemExit("ERROR: dataset root label_names.txt differs from fixed-dual sessions")

expected_index = "\n".join(valid) + "\n"
if action == "verify":
    if not index_path.is_file():
        raise SystemExit(f"ERROR: dataset index does not exist: {index_path}")
    if index_path.read_text() != expected_index:
        raise SystemExit(
            f"ERROR: dataset index is stale; refusing to rewrite during read-only validation: {index_path}"
        )
else:
    index_path.write_text(expected_index)
print(f"dataset_index={index_path}")
print(f"dataset_index_mode={mode}")
print(f"dataset_index_action={action}")
print(f"dataset_index_sessions={len(valid)}")
for name, reason in skipped:
    print(f"skip dataset session {name}: {reason}", file=sys.stderr)
PY
}

list_v7_dual_bag_dirs() {
  local raw_root="${RUN_ROOT}/bags/raw"
  if [[ ! -d "${raw_root}" ]]; then
    return 0
  fi
  find "${raw_root}" -mindepth 1 -maxdepth 1 -type d -name "*${V7_DUAL_BAG_SUFFIX}" | sort
}

latest_v7_dual_bag_dir() {
  local latest
  latest="$(list_v7_dual_bag_dirs | tail -n 1)"
  if [[ -n "${latest}" ]]; then
    printf '%s\n' "${latest}"
    return 0
  fi
  if [[ -n "${LAST_BAG_DIR:-}" ]]; then
    printf '%s\n' "${LAST_BAG_DIR}"
    return 0
  fi
  if [[ -n "${BAG_DIR:-}" ]]; then
    printf '%s\n' "${BAG_DIR}"
    return 0
  fi
  return 1
}

v7_dual_bag_timestamp() {
  local bag_dir="$1"
  local bag_name
  bag_dir="${bag_dir%/}"
  bag_name="${bag_dir##*/}"
  if [[ "${bag_name}" == *"${V7_DUAL_BAG_SUFFIX}" ]]; then
    printf '%s\n' "${bag_name%"${V7_DUAL_BAG_SUFFIX}"}"
  else
    printf '%s\n' "${bag_name}"
  fi
}

v7_dual_session_name_from_bag() {
  local bag_dir="$1"
  local bag_ts
  local sanitized
  bag_ts="$(v7_dual_bag_timestamp "${bag_dir}")"
  if [[ "${bag_ts}" =~ ^([0-9]{8})_([0-9]{6})$ ]]; then
    printf 'v7-dual-teleop-%s-%s\n' "${BASH_REMATCH[1]}" "${BASH_REMATCH[2]}"
    return 0
  fi
  sanitized="$(printf '%s' "${bag_ts}" | tr '_' '-' | tr -c 'A-Za-z0-9.-' '-')"
  if [[ "${sanitized}" != *-* ]]; then
    sanitized="v7-dual-${sanitized}"
  fi
  printf '%s\n' "${sanitized}"
}

print_run_summary() {
  echo "RUN_ID=${RUN_ID}"
  echo "RUN_ROOT=${RUN_ROOT}"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  echo "IGN_PARTITION=${IGN_PARTITION}"
  echo "SCAN_TOPIC=${SCAN_TOPIC}"
  echo "BAG_DIR=${BAG_DIR:-}"
  echo "LAST_BAG_DIR=${LAST_BAG_DIR:-}"
  echo "DATASET_ROOT=${DATASET_ROOT}"
}

set_manifest_var() {
  local key="$1"
  local value="$2"
  local manifest="${RUN_MANIFEST:-${RUN_ROOT}/run_manifest.env}"
  local tmp
  tmp="$(mktemp)"
  if [[ -f "${manifest}" ]] && grep -q "^export ${key}=" "${manifest}"; then
    awk -v key="${key}" -v value="${value}" '
      BEGIN { line = "export " key "=\"" value "\"" }
      $0 ~ "^export " key "=" { print line; next }
      { print }
    ' "${manifest}" > "${tmp}"
    mv "${tmp}" "${manifest}"
  else
    rm -f "${tmp}"
    printf 'export %s="%s"\n' "${key}" "${value}" >> "${manifest}"
  fi
  export "${key}=${value}"
}
