"""
从 resource 下 CSV（*.data）解析上身关节轨迹。

列名与采集文件一致（带 _joint）；发布到 ROS 时使用去掉 _joint 的名称。
"""

from __future__ import annotations

import csv
from pathlib import Path
from typing import List, Sequence, Tuple

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
    """按 JOINTS_WITH_SUFFIX 从每行抽取位置；缺列填 0。"""
    idx = {h: i for i, h in enumerate(header)}
    out: List[List[float]] = []
    for row in rows:
        vals = []
        for jname in JOINTS_WITH_SUFFIX:
            ii = idx.get(jname)
            if ii is None or ii >= len(row):
                vals.append(0.0)
            else:
                vals.append(float(row[ii]))
        out.append(vals)
    return out


def load_calibration_trajectory(path: Path) -> List[List[float]]:
    h, r = parse_trajectory_file(path)
    return extract_upper_body_rows(h, r)
