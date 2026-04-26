#!/usr/bin/env python3
"""
将关节轨迹经末端 FK -> 位姿上施加 SE(3) 变换 -> IK，再写回关节列，输出新轨迹文件。

后端：**Pinocchio**（已完全替换原 MuJoCo FK/IK 路径）。
末端：默认 ``left_wrist_roll_link`` / ``right_wrist_roll_link``（URDF link 名）。
``--robot-model`` 必须指向 URDF（``.urdf``）。

平移旋转：默认 x,y,z 为**毫米**、rx,ry,rz 为弧度；可用 --length-unit m 改为米；
可用 --angle-unit deg（如：10/0/0 mm，0/0/0°）。默认按 scipy 大写 **'XYZ'**：
绕**固定轴** X→Y→Z 的欧拉角（外旋）。可用 --euler-seq 改为其他序列。
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    from scipy.spatial.transform import Rotation as SciRotation
except ImportError as e:  # pragma: no cover
    raise SystemExit("需要 scipy（spatial.transform.Rotation）") from e

from action_data_paths import resolve_data_path, resolve_model_path, resolve_output_path
from arm_pinocchio_kin import PinocchioKinematics, pose_error, se3_from_Rt  # noqa: F401
from arm_transform_common import (
    ArmPassSpec,
    collect_arm_passes,
    compose_target_pose,
    expected_ee_translation_delta_world,
    header_column_index_for_mjcf_joint,
    write_trajectory_table,
)

# 采集数据（如 DZQ_*.data）中手指命名与模型不一致时的映射：data 列名 -> 模型关节名
CASBOT_FINGER_DATA_TO_MJCF: Dict[str, str] = {
    "left_thumb_metacarpal_joint": "left_thumb_mcp_roll_joint",
    "left_thumb_proximal_joint": "left_thumb_mcp_pitch_joint",
    "left_index_proximal_joint": "left_index_mcp_joint",
    "left_index_distal_joint": "left_index_dip_joint",
    "left_middle_proximal_joint": "left_middle_mcp_joint",
    "left_middle_distal_joint": "left_middle_dip_joint",
    "left_ring_proximal_joint": "left_ring_mcp_joint",
    "left_ring_distal_joint": "left_ring_dip_joint",
    "left_pinky_proximal_joint": "left_pinky_mcp_joint",
    "left_pinky_distal_joint": "left_pinky_dip_joint",
    "right_thumb_metacarpal_joint": "right_thumb_mcp_roll_joint",
    "right_thumb_proximal_joint": "right_thumb_mcp_pitch_joint",
    "right_index_proximal_joint": "right_index_mcp_joint",
    "right_index_distal_joint": "right_index_dip_joint",
    "right_middle_proximal_joint": "right_middle_mcp_joint",
    "right_middle_distal_joint": "right_middle_dip_joint",
    "right_ring_proximal_joint": "right_ring_mcp_joint",
    "right_ring_distal_joint": "right_ring_dip_joint",
    "right_pinky_proximal_joint": "right_pinky_mcp_joint",
    "right_pinky_distal_joint": "right_pinky_dip_joint",
}

# BS_*_full.csv 等导出：列名 leg_l* / upper_* / vhead_* / finger_* 与模型关节名不一致时的映射
_BS_TRAJECTORY_BODY_TO_MJCF: Dict[str, str] = {
    "leg_l1_joint": "left_leg_pelvic_pitch_joint",
    "leg_l2_joint": "left_leg_pelvic_roll_joint",
    "leg_l3_joint": "left_leg_pelvic_yaw_joint",
    "leg_l4_joint": "left_leg_knee_pitch_joint",
    "leg_l5_joint": "left_leg_ankle_pitch_joint",
    "leg_l6_joint": "left_leg_ankle_roll_joint",
    "leg_r1_joint": "right_leg_pelvic_pitch_joint",
    "leg_r2_joint": "right_leg_pelvic_roll_joint",
    "leg_r3_joint": "right_leg_pelvic_yaw_joint",
    "leg_r4_joint": "right_leg_knee_pitch_joint",
    "leg_r5_joint": "right_leg_ankle_pitch_joint",
    "leg_r6_joint": "right_leg_ankle_roll_joint",
    "waist_yaw_joint": "waist_yaw_joint",
    "upper_left_1_joint": "left_shoulder_pitch_joint",
    "upper_left_2_joint": "left_shoulder_roll_joint",
    "upper_left_3_joint": "left_shoulder_yaw_joint",
    "upper_left_4_joint": "left_elbow_pitch_joint",
    "upper_left_5_joint": "left_wrist_yaw_joint",
    "upper_left_6_joint": "left_wrist_pitch_joint",
    "upper_left_7_joint": "left_wrist_roll_joint",
    "upper_right_1_joint": "right_shoulder_pitch_joint",
    "upper_right_2_joint": "right_shoulder_roll_joint",
    "upper_right_3_joint": "right_shoulder_yaw_joint",
    "upper_right_4_joint": "right_elbow_pitch_joint",
    "upper_right_5_joint": "right_wrist_yaw_joint",
    "upper_right_6_joint": "right_wrist_pitch_joint",
    "upper_right_7_joint": "right_wrist_roll_joint",
    "vhead_1_joint": "head_yaw_joint",
    "vhead_2_joint": "head_pitch_joint",
}


def _bs_trajectory_joint_aliases() -> Dict[str, str]:
    out = dict(_BS_TRAJECTORY_BODY_TO_MJCF)
    for i, mj in enumerate(CASBOT_FINGER_DATA_TO_MJCF.values(), start=1):
        out[f"finger_{i}"] = mj
    return out


BS_TRAJECTORY_JOINT_TO_MJCF: Dict[str, str] = _bs_trajectory_joint_aliases()


def _parse_model_joint_names_from_urdf(urdf_path: str) -> List[str]:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    names: List[str] = []
    for j in root.findall("joint"):
        n = j.get("name")
        if n:
            names.append(n)
    return names


def _default_arm_joint_names(arm: str) -> List[str]:
    side = "left" if arm == "left" else "right"
    return [
        f"{side}_shoulder_pitch_joint",
        f"{side}_shoulder_roll_joint",
        f"{side}_shoulder_yaw_joint",
        f"{side}_elbow_pitch_joint",
        f"{side}_wrist_yaw_joint",
        f"{side}_wrist_pitch_joint",
        f"{side}_wrist_roll_joint",
    ]


def _default_ee_body_name(arm: str) -> str:
    return "left_wrist_roll_link" if arm == "left" else "right_wrist_roll_link"


def build_H(x: float, y: float, z: float, rx: float, ry: float, rz: float, euler_seq: str) -> np.ndarray:
    R = SciRotation.from_euler(euler_seq, [rx, ry, rz], degrees=False).as_matrix()
    return se3_from_Rt(R, np.array([x, y, z], dtype=np.float64))


def read_trajectory_table(path: str) -> Tuple[List[str], np.ndarray]:
    """首行为关节名，其余为数值。支持逗号或空白分隔；去掉表头与数据行末尾空列（如尾随逗号）。"""
    with open(path, newline="", encoding="utf-8") as f:
        sample = f.read(4096)
        f.seek(0)
        try:
            dialect = csv.Sniffer().sniff(sample, delimiters=",\t ")
        except csv.Error:
            dialect = csv.get_dialect("excel")
        reader = csv.reader(f, dialect)
        rows = list(reader)
    if not rows:
        raise ValueError("空数据文件")
    header = [c.strip() for c in rows[0]]
    while header and header[-1] == "":
        header.pop()
    data_rows: List[List[float]] = []
    for r in rows[1:]:
        if not r or all(not c.strip() for c in r):
            continue
        while r and r[-1].strip() == "":
            r.pop()
        if len(r) != len(header):
            raise ValueError(f"列数与表头不一致: 表头 {len(header)} 列, 数据行 {len(r)} 列")
        data_rows.append([float(c) for c in r])
    if not data_rows:
        raise ValueError("无轨迹帧")
    arr = np.asarray(data_rows, dtype=np.float64)
    return header, arr


def _warn_unknown_data_joints(
    header: Sequence[str],
    reference_joint_names: Sequence[str],
    joint_alias: Dict[str, str],
) -> None:
    ref = set(reference_joint_names)
    unknown: List[str] = []
    for h in header:
        if not h.endswith("_joint"):
            continue
        mj = joint_alias.get(h, h)
        if mj not in ref:
            unknown.append(h)
    if unknown:
        print("警告: 下列 data 中的 *_joint 列名无法映射到模型关节:", unknown[:12], file=sys.stderr)


def step_frame_ik_passes(
    kins: Sequence[PinocchioKinematics],
    passes: Sequence[ArmPassSpec],
    q: np.ndarray,
    header: Sequence[str],
    row: np.ndarray,
    default_qpos: np.ndarray,
    joint_alias: Dict[str, str],
    ik_iters: int,
    ik_tol: float,
    ik_damping: float,
    ik_max_step: float,
) -> None:
    """单帧：把轨迹行写入 ``q`` 作为初值，再对每条手臂迭代 IK 到收敛。

    供 ``mujoco_trajectory_sim`` 实时偏移回放使用；输入输出均为 Pinocchio q 向量。
    """
    if not kins:
        return
    kins[0].apply_row_to_qpos(q, header, row, default_qpos, joint_alias)
    for spec, kin in zip(passes, kins):
        T_ee = kin.fk_T_ee(q)
        T_tgt = compose_target_pose(spec, T_ee)
        for _ in range(ik_iters):
            if kin.ik_step(q, T_tgt, ik_damping, ik_max_step) < ik_tol:
                break


def run_pinocchio(
    urdf_path: str,
    data_path: str,
    out_path: str,
    passes: List[ArmPassSpec],
    ik_iters: int,
    ik_tol: float,
    ik_damping: float,
    ik_max_step: float,
    reference_joint_names: Optional[Sequence[str]],
    joint_alias: Dict[str, str],
    offset_check: bool = False,
) -> None:
    """Pinocchio FK + DLS IK（迭代至收敛）。"""
    header, frames = read_trajectory_table(data_path)
    if reference_joint_names is not None:
        _warn_unknown_data_joints(header, reference_joint_names, joint_alias)

    if not passes:
        raise ValueError("passes 为空：请使用 --arm left/right/both 并提供对应偏移")

    kins: List[PinocchioKinematics] = []
    first = PinocchioKinematics.from_urdf(
        urdf_path, passes[0].chain_joint_names, passes[0].ee_body_name
    )
    kins.append(first)
    for spec in passes[1:]:
        kins.append(
            PinocchioKinematics.share_model(first, spec.chain_joint_names, spec.ee_body_name)
        )

    default_q = first.default_qpos()
    q = default_q.copy()

    n_out = frames.shape[0]
    out_frames = frames.copy()

    for spec in passes:
        extra_ik = set(spec.chain_joint_names) - set(spec.write_joint_names)
        if extra_ik:
            print(
                f"注意: 手臂 {spec.ee_body_name} IK 使用了 {sorted(extra_ik)}，但未写入输出。",
                file=sys.stderr,
            )

    for i in range(n_out):
        first.apply_row_to_qpos(q, header, frames[i], default_q, joint_alias)
        for spec, kin in zip(passes, kins):
            T_ee = kin.fk_T_ee(q)
            p_before = T_ee[:3, 3].copy()
            exp_d = expected_ee_translation_delta_world(spec, T_ee)
            T_tgt = compose_target_pose(spec, T_ee)

            for _ in range(ik_iters):
                err_norm = kin.ik_step(q, T_tgt, ik_damping, ik_max_step)
                if err_norm < ik_tol:
                    break

            if offset_check and i == 0:
                p_after = kin.fk_T_ee(q)[:3, 3]
                act_d = p_after - p_before
                t_H = spec.H[:3, 3]
                print(
                    "[offset-check] 第 0 帧 "
                    f"{spec.ee_body_name} compose={spec.compose!r}\n"
                    f"  H 的平移 t_H (米, 已乘 --length-unit): [{t_H[0]:.6g}, {t_H[1]:.6g}, {t_H[2]:.6g}]\n"
                    f"  期望末端原点世界系位移 Δp (米): [{exp_d[0]:.6g}, {exp_d[1]:.6g}, {exp_d[2]:.6g}] "
                    f"(模长 {float(np.linalg.norm(exp_d)):.6g})\n"
                    f"  IK 后实际位移 (米): [{act_d[0]:.6g}, {act_d[1]:.6g}, {act_d[2]:.6g}] "
                    f"(模长 {float(np.linalg.norm(act_d)):.6g})",
                    file=sys.stderr,
                )

        for spec, kin in zip(passes, kins):
            for jn in spec.write_joint_names:
                col = header_column_index_for_mjcf_joint(jn, header, joint_alias)
                adr = kin.name_to_qpos_adr.get(jn)
                if col is not None and adr is not None:
                    out_frames[i, col] = float(q[adr])

    write_trajectory_table(out_path, header, out_frames)
    print(f"已写入: {out_path} ({n_out} 帧) [后端: pinocchio]")


def main() -> None:
    p = argparse.ArgumentParser(description="手臂关节轨迹: FK -> SE(3) -> IK 写回 (Pinocchio)")
    p.add_argument(
        "--robot-model",
        required=True,
        help="机器人模型：**URDF (.urdf)**；可写 urdf/xxx.urdf 相对包目录",
    )
    p.add_argument(
        "--data",
        required=True,
        help="关节轨迹；data/、output/ 前缀分别锚定包内目录；仅文件名时先 data 再 output",
    )
    p.add_argument(
        "--output",
        default=None,
        help="输出路径；仅文件名时写入包根下 output/（与 data/ 同级；只读安装时可能为 ~/ado_output）；默认 data_transform.csv",
    )
    p.add_argument("--arm", choices=("left", "right", "both"), required=True)
    p.add_argument(
        "--offset-left",
        default=None,
        help="左臂 x,y,z,rx,ry,rz（逗号分隔，单位同 --length-unit / --angle-unit）；与 --arm both 或 left 联用；"
        "首项为负须 --offset-left=-1,0,...（等号）",
    )
    p.add_argument(
        "--offset-right",
        default=None,
        help="右臂同上；与 --arm both 或 right 联用；首项为负须 --offset-right=-50,0,...（等号）",
    )
    p.add_argument(
        "--x",
        type=float,
        default=0.0,
        help="单臂或双臂共用的平移 x（单位由 --length-unit 决定，默认 mm）；可被 --offset-* 覆盖",
    )
    p.add_argument("--y", type=float, default=0.0, help="平移 y（与 --length-unit 一致；默认 mm）")
    p.add_argument("--z", type=float, default=0.0, help="平移 z")
    p.add_argument("--rx", type=float, default=0.0, help="绕固定轴欧拉分量（见 --angle-unit 与 --euler-seq）")
    p.add_argument("--ry", type=float, default=0.0)
    p.add_argument("--rz", type=float, default=0.0)
    p.add_argument(
        "--length-unit",
        choices=("m", "mm"),
        default="mm",
        help="平移 x,y,z 的单位（含 --offset-* 中前三项）。默认 mm；若以米为单位请指定 m",
    )
    p.add_argument(
        "--angle-unit",
        choices=("rad", "deg"),
        default="rad",
        help="rx,ry,rz 单位：rad 或 deg（测试用例为 deg）",
    )
    p.add_argument(
        "--euler-seq",
        default="XYZ",
        help="scipy Rotation.from_euler 序列，默认 XYZ 为外旋固定轴 X-Y-Z",
    )
    p.add_argument(
        "--compose",
        choices=("left", "right", "world_ee"),
        default="left",
        help=(
            "left: T'=H·T_ee（R_H 绕**世界原点**旋转、平移世界系）；"
            "right: T'=T_ee·H（H 在末端**局部**系，世界位移 R_ee·t_H）；"
            "world_ee: T'=[R_H·R_ee, p_ee+t_H]"
            "（**平移在世界系、旋转在末端原点处沿世界轴就地旋转**，与 Web UI 示教语义一致）"
        ),
    )
    p.add_argument(
        "--offset-check",
        action="store_true",
        help="处理第 0 帧后在 stderr 打印各臂：H 平移、期望/实际末端世界位移，用于核对 compose 与长度单位",
    )
    p.add_argument("--ee-body", default=None, help="单臂时末端 link；--arm both 时请用 --ee-body-left / --ee-body-right")
    p.add_argument("--ee-body-left", default=None, help="左臂末端 link，默认 left_wrist_roll_link")
    p.add_argument("--ee-body-right", default=None, help="右臂末端 link，默认 right_wrist_roll_link")
    p.add_argument("--arm-joints", default=None, help="单臂时 7 关节；双臂请用 --arm-joints-left/right")
    p.add_argument("--arm-joints-left", default=None, help="左臂关节名（逗号分隔）")
    p.add_argument("--arm-joints-right", default=None, help="右臂关节名（逗号分隔）")
    p.add_argument(
        "--ik-joints",
        default=None,
        help="单臂 IK 关节链；双臂请用 --ik-joints-left / --ik-joints-right",
    )
    p.add_argument("--ik-joints-left", default=None, help="左臂 IK 关节（逗号分隔）")
    p.add_argument("--ik-joints-right", default=None, help="右臂 IK 关节（逗号分隔）")
    p.add_argument(
        "--write-joints",
        default=None,
        help="单臂写回列；双臂建议 --write-joints-left / --write-joints-right",
    )
    p.add_argument("--write-joints-left", default=None, help="左臂写回列（逗号分隔）")
    p.add_argument("--write-joints-right", default=None, help="右臂写回列（逗号分隔）")
    p.add_argument("--urdf", default=None, help="可选：另一 URDF 路径，仅用于与 data 表头关节名对照")
    p.add_argument("--ik-iters", type=int, default=80)
    p.add_argument("--ik-tol", type=float, default=1e-4)
    p.add_argument("--ik-damping", type=float, default=1e-3)
    p.add_argument("--ik-max-step", type=float, default=0.15, help="每步关节增量 L2 上限（弧度）")

    args = p.parse_args()
    args.robot_model = resolve_model_path(args.robot_model)
    args.data = resolve_data_path(args.data)
    args.output = resolve_output_path(args.output, "data_transform.csv")
    if args.urdf:
        args.urdf = resolve_model_path(args.urdf)

    ext = os.path.splitext(args.robot_model)[1].lower()
    if ext != ".urdf":
        print(
            "--robot-model 必须为 URDF (.urdf)；FK/IK 已完全切换到 Pinocchio，不再接受 .xml/.mjcf。",
            file=sys.stderr,
        )
        sys.exit(2)

    passes = collect_arm_passes(
        args,
        build_H=build_H,
        default_arm_joint_names=_default_arm_joint_names,
        default_ee_body_name=_default_ee_body_name,
        chain_key="ik",
    )

    urdf_names = _parse_model_joint_names_from_urdf(
        args.urdf if args.urdf else args.robot_model
    )

    joint_alias = dict(CASBOT_FINGER_DATA_TO_MJCF)
    joint_alias.update(BS_TRAJECTORY_JOINT_TO_MJCF)

    run_pinocchio(
        urdf_path=args.robot_model,
        data_path=args.data,
        out_path=args.output,
        passes=passes,
        ik_iters=args.ik_iters,
        ik_tol=args.ik_tol,
        ik_damping=args.ik_damping,
        ik_max_step=args.ik_max_step,
        reference_joint_names=urdf_names,
        joint_alias=joint_alias,
        offset_check=args.offset_check,
    )


if __name__ == "__main__":
    main()
