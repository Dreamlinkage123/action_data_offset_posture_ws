"""
从 resource 下 CSV（*.data）解析上身关节轨迹。

列名与采集文件一致（带 _joint）；发布到 ROS 时使用去掉 _joint 的名称。
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

# 与 UpperJointData.joint 顺序一致：头、腰、双臂、双手（仅用户列出的关节）
JOINTS_WITH_SUFFIX: List[str] = [
    "head_yaw_joint",
    "head_pitch_joint",
    "waist_yaw_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_pitch_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
    "left_thumb_metacarpal_joint",
    "left_thumb_proximal_joint",
    "left_index_proximal_joint",
    "left_middle_proximal_joint",
    "left_ring_proximal_joint",
    "left_pinky_proximal_joint",
    "right_thumb_metacarpal_joint",
    "right_thumb_proximal_joint",
    "right_index_proximal_joint",
    "right_middle_proximal_joint",
    "right_ring_proximal_joint",
    "right_pinky_proximal_joint",
]


def strip_joint_suffix(name: str) -> str:
    if name.endswith("_joint"):
        return name[: -len("_joint")]
    return name


JOINT_NAMES_PUBLISH: List[str] = [strip_joint_suffix(n) for n in JOINTS_WITH_SUFFIX]

# 头(2) + 腰(1) 之后：双臂 7+7 与双手手指关节；乐队 CSV 顺序播放仅覆盖这些列
JOINTS_ARM_FINGER_SUFFIX: List[str] = JOINTS_WITH_SUFFIX[3:]
_JOINT_NAME_TO_INDEX: dict[str, int] = {n: i for i, n in enumerate(JOINTS_WITH_SUFFIX)}


def _header_index_map(header: Sequence[str]) -> dict[str, int]:
    return {str(h).strip(): i for i, h in enumerate(header)}


def column_index_for_joint_position(idx: dict[str, int], joint_name: str) -> Optional[int]:
    """
    在已建立的 ``idx``（表头→列下标）中查找关节 **position** 列。

    支持两种常见表头：``left_shoulder_pitch_joint`` 与 ``left_shoulder_pitch_joint_pos``
    （乐队采集 CSV 多为后者）；不匹配 ``*_vel`` 等其它后缀。
    """
    if joint_name in idx:
        return idx[joint_name]
    pos_key = f"{joint_name}_pos"
    if pos_key in idx:
        return idx[pos_key]
    return None


def parse_trajectory_file(path: Path) -> Tuple[List[str], List[List[float]]]:
    """读取 CSV：首行为列名，后续为数值行。"""
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = [h.strip() for h in next(reader)]
        rows: List[List[float]] = []
        for row in reader:
            if not row or all(not c.strip() for c in row):
                continue
            while len(row) < len(header):
                row.append("0")
            floats: List[float] = []
            for i in range(len(header)):
                cell = row[i].strip() if i < len(row) else "0"
                try:
                    floats.append(float(cell or 0.0))
                except ValueError:
                    floats.append(0.0)
            rows.append(floats)
    return header, rows


def extract_upper_body_rows(header: Sequence[str], rows: Sequence[Sequence[float]]) -> List[List[float]]:
    """按 JOINTS_WITH_SUFFIX 从每行抽取位置；缺列填 0。支持 ``*_joint`` 或 ``*_joint_pos`` 表头。"""
    idx = _header_index_map(header)
    out: List[List[float]] = []
    for row in rows:
        vals = []
        for jname in JOINTS_WITH_SUFFIX:
            col = column_index_for_joint_position(idx, jname)
            if col is None or col >= len(row):
                vals.append(0.0)
            else:
                vals.append(float(row[col]))
        out.append(vals)
    return out


def load_calibration_trajectory(path: Path) -> List[List[float]]:
    h, r = parse_trajectory_file(path)
    return extract_upper_body_rows(h, r)


def count_arm_finger_columns_in_header(header: Sequence[str]) -> int:
    """CSV 表头中可映射到 ``JOINTS_ARM_FINGER_SUFFIX`` 的 position 列数（``*_joint`` 或 ``*_joint_pos``）。"""
    idx = _header_index_map(header)
    n = 0
    for jn in JOINTS_ARM_FINGER_SUFFIX:
        if column_index_for_joint_position(idx, jn) is not None:
            n += 1
    return n


def merge_arm_finger_csv_row_into_full(
    header: Sequence[str],
    csv_row: Sequence[float],
    full_base: Sequence[float],
) -> List[float]:
    """
    在 ``full_base``（与 ``JOINTS_WITH_SUFFIX`` 同序）上，用 CSV 一行中已识别的臂/手指关节
    position 覆盖对应分量；头、腰等其它关节保持 ``full_base`` 原值。

    表头可为 ``left_shoulder_pitch_joint`` 或 ``left_shoulder_pitch_joint_pos``；不读取 ``*_vel``。
    """
    idx = _header_index_map(header)
    out = [float(x) for x in full_base]
    for jn in JOINTS_ARM_FINGER_SUFFIX:
        col = column_index_for_joint_position(idx, jn)
        if col is None or col >= len(csv_row):
            continue
        ji = _JOINT_NAME_TO_INDEX.get(jn)
        if ji is None or ji >= len(out):
            continue
        out[ji] = float(csv_row[col])
    return out
