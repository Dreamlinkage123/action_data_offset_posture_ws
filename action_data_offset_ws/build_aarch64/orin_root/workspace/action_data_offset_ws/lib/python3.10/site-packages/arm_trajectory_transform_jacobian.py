#!/usr/bin/env python3
"""
手臂关节轨迹：FK -> 末端施加 SE(3) 得到目标位姿 track_end_transform -> 多步雅可比增量叠加。

流程（默认 --linear-steps=8）：
  对每一帧，以当前 ``q`` 做 FK 得 ``T_track_end``，按 ``--compose`` 构造目标
  ``T_end_transform``，然后最多 ``linear_steps`` 次 DLS Jacobian 更新（每步重算 J），
  若本步叠加前 6D 误差 ≤ ``--pos-tol + --rot-tol`` 则提前退出；写回关节为收敛后的 ``q``。

compose 模式：
  - ``left``:     T_tgt = H · T_ee（旋转绕世界原点、平移世界系）
  - ``right``:    T_tgt = T_ee · H（H 在末端局部系）
  - ``world_ee``: T_tgt = [R_H·R_ee, p_ee + t_H]
                 （**平移在世界系、旋转在末端原点处沿世界轴就地旋转**，
                  与 ``casbot_arm_calibration_web`` UI 示教语义一致）

与 arm_trajectory_transform.py：后者是独立的迭代 IK 脚本；本脚本以 Jacobian DLS 为核心，
加入 ``--pos-tol / --rot-tol`` 收敛早退，避免大角度旋转因 ``--max-delta-norm`` 截断产生整体欠冲。
后端：**Pinocchio**；末端 ``left_wrist_roll_link`` / ``right_wrist_roll_link``。
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Dict, List, Optional, Sequence

import numpy as np

from action_data_paths import resolve_data_path, resolve_model_path, resolve_output_path
from arm_pinocchio_kin import PinocchioKinematics
from arm_transform_common import (
    ArmPassSpec,
    collect_arm_passes,
    compose_target_pose,
    expected_ee_translation_delta_world,
    header_column_index_for_mjcf_joint,
    write_trajectory_table,
)
from arm_trajectory_transform import (
    BS_TRAJECTORY_JOINT_TO_MJCF,
    CASBOT_FINGER_DATA_TO_MJCF,
    _default_arm_joint_names,
    _default_ee_body_name,
    _parse_model_joint_names_from_urdf,
    _warn_unknown_data_joints,
    build_H,
    read_trajectory_table,
)


def run_pinocchio(
    urdf_path: str,
    data_path: str,
    out_path: str,
    passes: List[ArmPassSpec],
    linear_steps: int,
    damping: float,
    max_delta_norm: float,
    reference_joint_names: Optional[Sequence[str]],
    joint_alias: Dict[str, str],
    offset_check: bool = False,
    pos_tol_m: float = 1e-4,
    rot_tol_rad: float = 1e-3,
) -> None:
    """Pinocchio 正解 + 两步（默认）雅可比增量叠加；末端为 URDF link 名
    （默认左 ``left_wrist_roll_link``、右 ``right_wrist_roll_link``）。"""
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
        extra = set(spec.chain_joint_names) - set(spec.write_joint_names)
        if extra:
            print(
                f"注意: 手臂 {spec.ee_body_name} 雅可比列含 {sorted(extra)}，但未写入输出。",
                file=sys.stderr,
            )

    max_iters = max(1, linear_steps)
    for i in range(n_out):
        first.apply_row_to_qpos(q, header, frames[i], default_q, joint_alias)
        for spec, kin in zip(passes, kins):
            T_track_end = kin.fk_T_ee(q)
            p_before = T_track_end[:3, 3].copy()
            exp_d = expected_ee_translation_delta_world(spec, T_track_end)
            T_end_transform = compose_target_pose(spec, T_track_end)

            iters_used = 0
            for _ in range(max_iters):
                iters_used += 1
                _dq, err_before = kin.apply_jacobian_delta(
                    q, T_end_transform, damping, max_delta_norm
                )
                # 早退：本步前 6D 误差已足够小（位置 + 旋转合范数）
                if err_before < (pos_tol_m + rot_tol_rad):
                    break

            if offset_check and i == 0:
                p_after = kin.fk_T_ee(q)[:3, 3]
                act_d = p_after - p_before
                t_H = spec.H[:3, 3]
                print(
                    "[offset-check] 第 0 帧 "
                    f"{spec.ee_body_name} compose={spec.compose!r} iters={iters_used}/{max_iters}\n"
                    f"  H 的平移 t_H (米, 已乘 --length-unit): [{t_H[0]:.6g}, {t_H[1]:.6g}, {t_H[2]:.6g}]\n"
                    f"  期望末端原点世界系位移 Δp (米): [{exp_d[0]:.6g}, {exp_d[1]:.6g}, {exp_d[2]:.6g}] "
                    f"(模长 {float(np.linalg.norm(exp_d)):.6g})\n"
                    f"  雅可比步后实际位移 (米): [{act_d[0]:.6g}, {act_d[1]:.6g}, {act_d[2]:.6g}] "
                    f"(模长 {float(np.linalg.norm(act_d)):.6g})",
                    file=sys.stderr,
                )

        for spec, kin in zip(passes, kins):
            for jn in spec.write_joint_names:
                col = header_column_index_for_mjcf_joint(jn, header, joint_alias)
                adr = kin.name_to_qpos_adr.get(jn)
                if col is not None and adr is not None:
                    out_frames[i, col] = float(q[adr])

        pct = int((i + 1) * 100 / n_out)
        if n_out <= 200 or (i + 1) % max(1, n_out // 100) == 0 or i == n_out - 1:
            print(f"[progress] {pct}% ({i+1}/{n_out})", flush=True)

    write_trajectory_table(out_path, header, out_frames)
    print(f"已写入: {out_path} ({n_out} 帧) [后端: pinocchio]")


def main() -> None:
    p = argparse.ArgumentParser(
        description="手臂轨迹: FK -> H -> 两步（默认）雅可比增量 Δq 叠加，非迭代 IK (Pinocchio)"
    )
    p.add_argument(
        "--robot-model",
        required=True,
        help="机器人模型：**URDF (.urdf)**（相对包目录或绝对路径均可）",
    )
    p.add_argument(
        "--data",
        required=True,
        help="关节轨迹（首行关节名）；data/、output/ 前缀分别锚定包内目录；仅文件名时先 data 再 output",
    )
    p.add_argument(
        "--output",
        default=None,
        help="输出路径；仅文件名时写入包根下 output/（与 data/ 同级；只读安装时可能为 ~/ado_output）；默认 data_transform_jacobian.csv",
    )
    p.add_argument("--arm", choices=("left", "right", "both"), required=True)
    p.add_argument(
        "--offset-left",
        default=None,
        help="左臂 x,y,z,rx,ry,rz（逗号分隔，单位同 --length-unit / --angle-unit）；"
        "首项为负时必须写成 --offset-left=-1,0,...（等号），否则 argparse 会把 -1 当成新选项",
    )
    p.add_argument(
        "--offset-right",
        default=None,
        help="右臂同上；首项为负时须 --offset-right=-50,0,...",
    )
    p.add_argument("--x", type=float, default=0.0, help="平移 x（单位由 --length-unit 决定，默认 mm）")
    p.add_argument("--y", type=float, default=0.0, help="平移 y（与 --length-unit 一致；默认 mm）")
    p.add_argument("--z", type=float, default=0.0)
    p.add_argument("--rx", type=float, default=0.0)
    p.add_argument("--ry", type=float, default=0.0)
    p.add_argument("--rz", type=float, default=0.0)
    p.add_argument(
        "--length-unit",
        choices=("m", "mm"),
        default="mm",
        help="x,y,z 与 --offset-* 前三项；默认 mm；米制请指定 m",
    )
    p.add_argument("--angle-unit", choices=("rad", "deg"), default="rad")
    p.add_argument("--euler-seq", default="XYZ", help="scipy from_euler，默认外旋 XYZ")
    p.add_argument(
        "--compose",
        choices=("left", "right", "world_ee"),
        default="left",
        help=(
            "left: T_tgt=H·T（旋转绕世界原点、平移世界系）；"
            "right: T_tgt=T·H（H 在末端局部系）；"
            "world_ee: 平移在世界系且旋转在末端原点处就地沿世界轴旋转"
            "（与 Web UI 示教语义一致）。"
        ),
    )
    p.add_argument(
        "--offset-check",
        action="store_true",
        help="第 0 帧后 stderr 打印期望/实际末端位移，核对 compose 与单位；雅可比脚本若步数过少实际位移会小于期望",
    )
    p.add_argument("--ee-body", default=None)
    p.add_argument("--ee-body-left", default=None)
    p.add_argument("--ee-body-right", default=None)
    p.add_argument("--arm-joints", default=None)
    p.add_argument("--arm-joints-left", default=None)
    p.add_argument("--arm-joints-right", default=None)
    p.add_argument(
        "--jac-joints",
        default=None,
        help="单臂时雅可比关节链；双臂请用 --jac-joints-left / --jac-joints-right",
    )
    p.add_argument("--jac-joints-left", default=None)
    p.add_argument("--jac-joints-right", default=None)
    p.add_argument("--write-joints", default=None)
    p.add_argument("--write-joints-left", default=None)
    p.add_argument("--write-joints-right", default=None)
    p.add_argument("--urdf", default=None, help="可选，关节名对照")
    p.add_argument(
        "--linear-steps",
        type=int,
        default=8,
        help=(
            "雅可比增量最大步数（每步重算 J）：默认 8。达到 --pos-tol/--rot-tol 后提前退出；"
            "大幅度旋转时较大步数可以避免因 --max-delta-norm 截断导致的整体欠冲。"
        ),
    )
    p.add_argument("--damping", type=float, default=1e-3, help="阻尼最小二乘 λ")
    p.add_argument(
        "--max-delta-norm",
        type=float,
        default=0.15,
        help="单次 Δq 的 L2 上限（弧度）",
    )
    p.add_argument(
        "--pos-tol",
        type=float,
        default=1e-4,
        help="位置收敛阈值（米），Jacobian 循环每步叠加前 6D 误差≤pos-tol+rot-tol 即早退（默认 1e-4m=0.1mm）",
    )
    p.add_argument(
        "--rot-tol",
        type=float,
        default=1e-3,
        help="旋转收敛阈值（弧度），与 --pos-tol 共同决定每帧 Jacobian 循环的早退条件（默认 1e-3 ≈ 0.057°）",
    )

    args = p.parse_args()
    args.robot_model = resolve_model_path(args.robot_model)
    args.data = resolve_data_path(args.data)
    args.output = resolve_output_path(args.output, "data_transform_jacobian.csv")
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
        chain_key="jac",
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
        linear_steps=args.linear_steps,
        damping=args.damping,
        max_delta_norm=args.max_delta_norm,
        reference_joint_names=urdf_names,
        joint_alias=joint_alias,
        offset_check=args.offset_check,
        pos_tol_m=args.pos_tol,
        rot_tol_rad=args.rot_tol,
    )


if __name__ == "__main__":
    main()
