"""左右臂 SE(3) 偏移与 ArmPassSpec 解析，供 arm_trajectory_transform / jacobian / mujoco_trajectory_sim 共用。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

import numpy as np


@dataclass
class ArmPassSpec:
    """单条手臂：齐次变换 H、位姿组合方式、运动链关节、写回列、末端 body。"""

    H: np.ndarray
    compose: str
    chain_joint_names: List[str]
    write_joint_names: List[str]
    ee_body_name: str


def parse_csv_six(s: Optional[str]) -> Optional[Tuple[float, float, float, float, float, float]]:
    if s is None or not str(s).strip():
        return None
    parts = [float(x.strip()) for x in str(s).split(",")]
    if len(parts) != 6:
        raise ValueError(f"偏移字符串须为 6 个逗号分隔数值 (x,y,z,rx,ry,rz)，当前 {len(parts)} 个: {s!r}")
    return (parts[0], parts[1], parts[2], parts[3], parts[4], parts[5])


def six_to_H(
    t: Sequence[float],
    *,
    scale_l: float,
    angle_unit: str,
    euler_seq: str,
    build_H: Callable[..., np.ndarray],
) -> np.ndarray:
    tx, ty, tz, rx, ry, rz = t
    tx *= scale_l
    ty *= scale_l
    tz *= scale_l
    if angle_unit == "deg":
        rx, ry, rz = np.deg2rad([rx, ry, rz])
    return build_H(float(tx), float(ty), float(tz), float(rx), float(ry), float(rz), euler_seq)


def _split_joints(s: Optional[str]) -> Optional[List[str]]:
    if s is None or not str(s).strip():
        return None
    return [x.strip() for x in str(s).split(",") if x.strip()]


def collect_arm_passes(
    args: Any,
    *,
    build_H: Callable[..., np.ndarray],
    default_arm_joint_names: Callable[[str], List[str]],
    default_ee_body_name: Callable[[str], str],
    chain_key: str,
) -> List[ArmPassSpec]:
    """
    chain_key: \"ik\" -> 读取 ik_joints / ik_joints_left / ik_joints_right / arm_joints*
      \"jac\" -> 读取 jac_joints / jac_joints_left / jac_joints_right / arm_joints*
    """
    scale_l = 0.001 if args.length_unit == "mm" else 1.0
    legacy = (float(args.x), float(args.y), float(args.z), float(args.rx), float(args.ry), float(args.rz))
    ol = parse_csv_six(getattr(args, "offset_left", None))
    orr = parse_csv_six(getattr(args, "offset_right", None))

    def pj(side: str) -> str:
        return f"{chain_key}_joints_{side}"

    def arm_j(side: str) -> str:
        return f"arm_joints_{side}"

    def wj(side: str) -> str:
        return f"write_joints_{side}"

    def ee_b(side: str) -> str:
        return f"ee_body_{side}"

    def chain_for_side(side: str) -> List[str]:
        v = _split_joints(getattr(args, pj(side), None))
        if v:
            return v
        v = _split_joints(getattr(args, arm_j(side), None))
        if v:
            return v
        v = _split_joints(getattr(args, f"{chain_key}_joints", None))
        if v:
            return v
        v = _split_joints(getattr(args, "arm_joints", None))
        if v:
            return v
        return default_arm_joint_names(side)

    def write_for_side(side: str) -> List[str]:
        v = _split_joints(getattr(args, wj(side), None))
        if v:
            return v
        v = _split_joints(getattr(args, "write_joints", None))
        if v:
            return v
        return chain_for_side(side)

    def ee_for_side(side: str) -> str:
        v = getattr(args, ee_b(side), None)
        if v:
            return str(v)
        v = getattr(args, "ee_body", None)
        if v:
            return str(v)
        return default_ee_body_name(side)

    compose = str(args.compose)
    euler = str(args.euler_seq)
    au = str(args.angle_unit)

    passes: List[ArmPassSpec] = []

    if args.arm == "left":
        raw = ol if ol is not None else legacy
        H = six_to_H(raw, scale_l=scale_l, angle_unit=au, euler_seq=euler, build_H=build_H)
        passes.append(
            ArmPassSpec(
                H=H,
                compose=compose,
                chain_joint_names=chain_for_side("left"),
                write_joint_names=write_for_side("left"),
                ee_body_name=ee_for_side("left"),
            )
        )
    elif args.arm == "right":
        raw = orr if orr is not None else legacy
        H = six_to_H(raw, scale_l=scale_l, angle_unit=au, euler_seq=euler, build_H=build_H)
        passes.append(
            ArmPassSpec(
                H=H,
                compose=compose,
                chain_joint_names=chain_for_side("right"),
                write_joint_names=write_for_side("right"),
                ee_body_name=ee_for_side("right"),
            )
        )
    else:
        # both
        if ol is None and orr is None:
            hl = hr = six_to_H(legacy, scale_l=scale_l, angle_unit=au, euler_seq=euler, build_H=build_H)
        else:
            z = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            hl = six_to_H(ol if ol is not None else z, scale_l=scale_l, angle_unit=au, euler_seq=euler, build_H=build_H)
            hr = six_to_H(orr if orr is not None else z, scale_l=scale_l, angle_unit=au, euler_seq=euler, build_H=build_H)
        passes.append(
            ArmPassSpec(
                H=hl,
                compose=compose,
                chain_joint_names=chain_for_side("left"),
                write_joint_names=write_for_side("left"),
                ee_body_name=ee_for_side("left"),
            )
        )
        passes.append(
            ArmPassSpec(
                H=hr,
                compose=compose,
                chain_joint_names=chain_for_side("right"),
                write_joint_names=write_for_side("right"),
                ee_body_name=ee_for_side("right"),
            )
        )

    return passes


def compose_target_pose(spec: ArmPassSpec, T_ee: np.ndarray) -> np.ndarray:
    """
    按 ``spec.compose`` 组合 H 与当前末端位姿 ``T_ee``，返回新的 4x4 目标位姿。

    - ``left``：``T_tgt = H @ T_ee``（R_H 在世界系左乘，位置被 R_H 绕世界原点旋转并加上 t_H）。
    - ``right``：``T_tgt = T_ee @ H``（H 在末端局部系右乘）。
    - ``world_ee``：``T_tgt = [R_H·R_ee, p_ee + t_H]``，即
      **平移在世界系、旋转在末端原点处沿世界轴就地旋转**（与 Web UI 示教语义一致）。
    """
    R_H = spec.H[:3, :3]
    t_H = spec.H[:3, 3]
    R_ee = T_ee[:3, :3]
    p_ee = T_ee[:3, 3]
    if spec.compose == "left":
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R_H @ R_ee
        T[:3, 3] = R_H @ p_ee + t_H
        return T
    if spec.compose == "right":
        return T_ee @ spec.H
    if spec.compose == "world_ee":
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R_H @ R_ee
        T[:3, 3] = p_ee + t_H
        return T
    raise ValueError(f"未知 compose 模式: {spec.compose!r}（支持 left/right/world_ee）")


def expected_ee_translation_delta_world(spec: ArmPassSpec, T_ee: np.ndarray) -> np.ndarray:
    """
    若位姿严格达到 T_tgt（compose 与脚本中一致），末端 body 原点在世界系下的平移增量 Δp。

    - left: T_tgt = H @ T_ee  =>  Δp = R_H p + t_H - p
    - right: T_tgt = T_ee @ H  =>  Δp = R_ee t_H
    - world_ee: T_tgt 保持 p 不变只加 t_H  =>  Δp = t_H
    """
    R_H = spec.H[:3, :3]
    t_H = spec.H[:3, 3]
    p = T_ee[:3, 3]
    R_ee = T_ee[:3, :3]
    if spec.compose == "left":
        return R_H @ p + t_H - p
    if spec.compose == "right":
        return R_ee @ t_H
    if spec.compose == "world_ee":
        return t_H.copy()
    raise ValueError(f"未知 compose 模式: {spec.compose!r}")


def header_column_index_for_mjcf_joint(
    mj_joint_name: str,
    header: Sequence[str],
    joint_alias: Dict[str, str],
) -> Optional[int]:
    """
    轨迹表头列名若与模型关节名不一致（由 joint_alias 映射），返回应对应关节名的列下标；
    用于 IK/雅可比写回时定位 `upper_left_1_joint` 等列。
    """
    for j, h in enumerate(header):
        if joint_alias.get(h, h) == mj_joint_name:
            return j
    return None


def format_trajectory_csv_float(v: float) -> str:
    """与采集 .data 一致：0 写作 0.0；非零 round 到 6 位小数后去尾零；禁止科学计数法。"""
    if not math.isfinite(v):
        return str(v)
    if abs(v) < 1e-15:
        return "0.0"
    x = round(float(v), 6)
    if abs(x) < 1e-15:
        return "0.0"
    s = f"{x:.6f}"
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    if s in ("-0", "-0.0"):
        return "0.0"
    if "." not in s:
        s = f"{x:.1f}".rstrip("0").rstrip(".")
        if "." not in s:
            s += ".0"
    return s


def write_trajectory_table(path: str, header: Sequence[str], data: np.ndarray) -> None:
    """写出轨迹 CSV：表头无尾逗号；数据行每列 format_trajectory_csv_float，行末逗号（与仓库 .data 一致）。"""
    lines = [",".join(header)]
    for row in data:
        cells = [format_trajectory_csv_float(float(v)) for v in row]
        lines.append(",".join(cells) + ",")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
