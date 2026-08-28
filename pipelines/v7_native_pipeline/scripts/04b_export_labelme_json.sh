#!/usr/bin/env bash
# 【具体数据接口】
# 代码中检测到的命令行参数：未检测到 --参数；可能通过 ROS 2 参数、环境变量或固定配置输入。
# 代码中检测到的 ROS 2 话题/路径字符串：未检测到明确 topic；请结合 ROS 2 参数声明或上层 launch 查看。
# 检测到的消息类型：未检测到 Python 消息导入；可能使用文件、标准库或 Shell 数据。
# 检测到的文件格式：JSON, PNG, TXT, YAML
# 可能使用的关键环境变量：BASH_SOURCE, ERROR, EXPORT_DIR, JSON, LABELME_JSON, LABELME_PY, LABELME_SOURCE_JSON, MAP_PGM, MAP_YAML, RUN_MANIFEST, RUN_ROOT, SCRIPT_DIR, SEMANTIC_LABEL_DIR, SEMANTIC_LABEL_PNG, WARNING
# 数据说明：以上内容从代码正文中的参数、topic、消息导入、文件扩展名和环境变量提取；实际字段、shape、单位和发布方向仍以本脚本正文为准。
# 【后续管理信息】
# 类型：Shell 流程脚本
# 推荐运行方式：bash /home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/04b_export_labelme_json.sh
# 输入来源：命令行参数、环境变量和上一步 pipeline 产生的文件或目录。
# 输出结果：下一阶段需要的数据、模型、日志或 ROS 2 进程。
# 前置条件：先加载项目环境，并确认上一步输出存在。
# 后续步骤：按 pipeline 编号执行后续阶段脚本。
# 副作用与安全：可能启动进程或写入实验输出；默认不应删除原始数据。
# 当前状态：当前项目源文件。
# 【文件时间信息】
# 文件系统创建时间：2026-07-17 03:41:31.487295161 -0400
# 添加本说明前的最后修改时间：2026-08-02 06:19:27.121706538 -0400
# 注意：本区块写入后，文件系统的最后修改时间会更新；创建时间不会因本次注释改变。
# 【依赖作用说明】
# 直接依赖的具体作用：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh（source 加载公共环境变量/函数）
# 被依赖的具体作用：未检测到其他脚本代码正文直接调用本文件；可能由人工命令、ROS 2 注册入口或外部工具启动。
# 说明依据：上述关系根据本文件中的 source、ros2 launch、ros2 run、import、subprocess 和脚本路径调用整理；参数/话题级关系仍以代码正文为准。
# 【绝对依赖关系】
# 文件绝对路径：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/04b_export_labelme_json.sh
# 直接依赖（脚本层面）：/home/user/navigation_project/a_pipeline/pipelines/v7_native_pipeline/scripts/common.sh
# 被依赖/被引用（脚本层面）：无代码正文中的直接调用者。
# 版本与来源：当前项目源文件；构建产物 install/build 不作为编辑源。
# 注意：以上是按文件引用和同名脚本调用静态整理；ROS 2 话题、系统命令、第三方库依赖仍以代码正文为准。
# 【脚本说明｜04b_export_labelme_json.sh】
# 用途：pipeline 阶段脚本，按编号执行数据采集、转换、训练、评估或辅助操作。
# 输入输出：输入通常是命令行参数、配置或上一步数据；输出通常是下一阶段数据、日志或启动的进程。
# 关系：由本 pipeline 的其他阶段脚本或人工命令调用，依赖 common.sh、ROS 2 环境和相关工具；不是备份文件。
# 版本定位：当前源文件；不要只修改 build/install/log 或 runs 中的副本。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/common.sh"
load_manifest "${RUN_MANIFEST:-}"

mkdir -p "${SEMANTIC_LABEL_DIR}"
LOG="${RUN_ROOT}/logs/04b_export_labelme_json_$(timestamp).log"
EXPORT_DIR="${RUN_ROOT}/logs/semantic_label_export_$(timestamp)"
LABELME_SOURCE_JSON="${LABELME_JSON}"

write_semantic_map_yaml() {
  python3 - "${MAP_YAML}" "${SEMANTIC_LABEL_DIR}/map.yaml" "$(basename "${MAP_PGM}")" <<'PY'
from pathlib import Path
import sys

source = Path(sys.argv[1])
destination = Path(sys.argv[2])
pgm_name = sys.argv[3]
lines = source.read_text(encoding="utf-8").splitlines()
rewritten = []
replaced = False
for line in lines:
    if line.lstrip().startswith("image:"):
        prefix = line[: len(line) - len(line.lstrip())]
        rewritten.append(f"{prefix}image: ../slam/{pgm_name}")
        replaced = True
    else:
        rewritten.append(line)
if not replaced:
    raise SystemExit(f"ERROR: map yaml has no image: entry: {source}")
destination.write_text("\n".join(rewritten) + "\n", encoding="utf-8")
PY
}

if [[ ! -f "${LABELME_SOURCE_JSON}" ]]; then
  LABELME_SOURCE_JSON="$(find "$(dirname "${LABELME_JSON}")" -maxdepth 1 -type f -name '*.json' -printf '%T@ %p\n' | sort -nr | awk 'NR==1 {print $2}')"
  if [[ -z "${LABELME_SOURCE_JSON}" || ! -f "${LABELME_SOURCE_JSON}" ]]; then
    echo "ERROR: required LabelMe JSON not found: ${LABELME_JSON}" >&2
    echo "No fallback *.json found in: $(dirname "${LABELME_JSON}")" >&2
    exit 2
  fi
  echo "WARNING: expected LabelMe JSON not found:"
  echo "  ${LABELME_JSON}"
  echo "Using newest LabelMe JSON instead:"
  echo "  ${LABELME_SOURCE_JSON}"
  if [[ "$(readlink -f "${LABELME_SOURCE_JSON}")" != "$(readlink -m "${LABELME_JSON}")" ]]; then
    cp -a "${LABELME_SOURCE_JSON}" "${LABELME_JSON}"
  fi
fi

{
  echo "Exporting ${LABELME_SOURCE_JSON}"
  if [[ -x "${LABELME_PY}" ]]; then
    "${LABELME_PY}" - "${LABELME_SOURCE_JSON}" "${EXPORT_DIR}" <<'PY'
import json
import shutil
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from labelme import utils

json_path = Path(sys.argv[1])
out_dir = Path(sys.argv[2])
out_dir.mkdir(parents=True, exist_ok=True)

data = json.loads(json_path.read_text())
image_path = Path(data.get("imagePath", ""))
if not image_path.is_absolute():
    image_path = json_path.parent / image_path
if data.get("imageData"):
    image = utils.img_b64_to_arr(data["imageData"])
elif image_path.exists():
    image = np.asarray(Image.open(image_path).convert("RGB"))
else:
    raise FileNotFoundError(f"Cannot find LabelMe image: {image_path}")

shapes = data.get("shapes", [])
if not shapes:
    raise ValueError("LabelMe JSON has no shapes; annotate at least one semantic class.")

labels_by_key = {}
for shape in shapes:
    label = str(shape.get("label", "")).strip()
    if not label:
        raise ValueError("Every LabelMe shape must have a non-empty label.")
    if label.casefold() == "_background_":
        raise ValueError("'_background_' is reserved for unlabeled pixels; use a different class name.")
    shape["label"] = label
    key = label.casefold()
    if key in labels_by_key and labels_by_key[key] != label:
        raise ValueError(
            f"Class names differ only by capitalization: {labels_by_key[key]!r} and {label!r}"
        )
    labels_by_key[key] = label

label_names = ["_background_", *(labels_by_key[key] for key in sorted(labels_by_key))]
label_name_to_value = {name: idx for idx, name in enumerate(label_names)}
lbl, _ = utils.shapes_to_label(image.shape, data["shapes"], label_name_to_value)
lbl = lbl.astype(np.uint8)
utils.lblsave(str(out_dir / "label.png"), lbl)
Image.fromarray(image).save(out_dir / "img.png")
(out_dir / "label_names.txt").write_text("\n".join(label_names) + "\n")

try:
    import imgviz

    label_viz = imgviz.label2rgb(
        label=lbl,
        image=image,
        label_names=label_names,
        font_size=15,
        loc="rb",
    )
    Image.fromarray(label_viz).save(out_dir / "label_viz.png")
except Exception:
    Image.fromarray((lbl.astype(np.uint8) * 25)).save(out_dir / "label_viz.png")

shutil.copy2(json_path, out_dir / "labelme.json")
PY
  elif command -v labelme_export_json >/dev/null 2>&1; then
    labelme_export_json "${LABELME_SOURCE_JSON}" -o "${EXPORT_DIR}"
  elif command -v labelme_json_to_dataset >/dev/null 2>&1; then
    labelme_json_to_dataset "${LABELME_SOURCE_JSON}" -o "${EXPORT_DIR}"
  else
    echo "ERROR: no LabelMe exporter found." >&2
    echo "Install labelme, or set LABELME_PY to a Python with labelme installed." >&2
    exit 2
  fi
  cp -a "${EXPORT_DIR}/." "${SEMANTIC_LABEL_DIR}/"
  if [[ "$(readlink -f "${LABELME_SOURCE_JSON}")" != "$(readlink -m "${SEMANTIC_LABEL_DIR}/map_labelme.json")" ]]; then
    cp -a "${LABELME_SOURCE_JSON}" "${SEMANTIC_LABEL_DIR}/map_labelme.json"
  fi
  write_semantic_map_yaml
  ls -lh "${SEMANTIC_LABEL_DIR}"
  test -f "${SEMANTIC_LABEL_PNG}"
} 2>&1 | tee "${LOG}"

echo "Wrote ${LOG}"
