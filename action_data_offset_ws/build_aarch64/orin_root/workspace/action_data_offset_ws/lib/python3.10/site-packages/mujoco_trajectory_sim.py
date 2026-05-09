#!/usr/bin/env python3
"""
MuJoCo 关节轨迹回放：加载 MJCF/XML（与 CASBOT 仓库一致）及与 arm_trajectory_transform 相同格式的 .data 表，
按时间序列将轨迹写入 qpos，再 mj_step 推进动力学（含白键铰链与手指–琴键接触）；在 viewer 中播放。

**仅支持 x86_64 平台**（Orin / aarch64 不支持 MuJoCo 仿真，请使用真机测试）。

可选 ``--tf-arm``：每帧在原始轨迹上施加与 ``arm_trajectory_transform`` 相同的 SE(3) 与迭代 IK
（可 left/right/both，配合 ``--tf-offset-left`` / ``--tf-offset-right`` 等）。**FK/IK 使用 Pinocchio**
（与 ``arm_trajectory_transform`` 保持一致；模型默认由 ``--tf-urdf`` 或由 MJCF 推断同目录 .urdf）。
对应臂 7 关节解算后写回 MuJoCo qpos 用于渲染与 mj_step；其余关节（如白键、腿、手指）仍走 MuJoCo。

说明：若 MJCF 中含白键铰链等被动关节，本脚本会保留其 qpos/qvel 并在每帧积分接触；轨迹中的手臂关节在
mj_step 后锁回给定值（运动学手臂 + 被动琴键）。纯运动学回放可使用 ``--kinematic-only``（仅 mj_forward）。
可选 ``--piano-audio``：铰链角低于阈值时按关节名（如 C4_white_24_hinge）播放对应音高（需 sounddevice 或 pygame）。
"""

from __future__ import annotations

import argparse
import os
import platform
import sys
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

_ARCH = platform.machine()
if _ARCH not in ("x86_64", "AMD64"):
    print(
        f"错误: mujoco_trajectory_sim 仅支持 x86_64 平台（当前: {_ARCH}）。\n"
        "Orin / aarch64 环境不支持 MuJoCo 仿真，请使用真机测试。",
        file=sys.stderr,
    )
    sys.exit(1)

try:
    import mujoco
    import mujoco.viewer
except ImportError:
    print(
        "错误: 未安装 mujoco 包。请运行: pip3 install mujoco\n"
        "注意: MuJoCo 仿真仅在 x86_64 环境中可用。",
        file=sys.stderr,
    )
    sys.exit(1)

from action_data_paths import resolve_data_path, resolve_model_path
from arm_pinocchio_kin import PinocchioKinematics
from arm_trajectory_transform import (
    BS_TRAJECTORY_JOINT_TO_MJCF,
    CASBOT_FINGER_DATA_TO_MJCF,
    _default_arm_joint_names,
    _default_ee_body_name,
    build_H,
    read_trajectory_table,
    step_frame_ik_passes,
)
from arm_transform_common import ArmPassSpec, collect_arm_passes

from piano_key_audio import PianoKeyAudio


def _joint_maps(model: mujoco.MjModel) -> Tuple[Dict[str, int], Dict[str, int]]:
    name_to_qpos_adr: Dict[str, int] = {}
    name_to_joint_id: Dict[str, int] = {}
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if name:
            name_to_qpos_adr[name] = int(model.jnt_qposadr[j])
            name_to_joint_id[name] = j
    return name_to_qpos_adr, name_to_joint_id


def piano_hinge_passive_indices(
    model: mujoco.MjModel,
) -> Tuple[np.ndarray, np.ndarray, List[str]]:
    """白键铰链：qpos 下标、dof 下标、与之一一对应的关节名（顺序与模型关节枚举一致）。"""
    q_list: List[int] = []
    d_list: List[int] = []
    n_list: List[str] = []
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        if not name or "_white_" not in name or not name.endswith("_hinge"):
            continue
        q_list.append(int(model.jnt_qposadr[j]))
        d_list.append(int(model.jnt_dofadr[j]))
        n_list.append(name)
    return np.array(q_list, dtype=np.int32), np.array(d_list, dtype=np.int32), n_list


def apply_trajectory_row(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    header: Sequence[str],
    row: np.ndarray,
    default_qpos: np.ndarray,
    name_to_qpos_adr: Dict[str, int],
    name_to_joint_id: Dict[str, int],
    joint_alias: Dict[str, str],
    piano_qpos_idx: Optional[np.ndarray] = None,
    piano_dof_idx: Optional[np.ndarray] = None,
) -> None:
    if piano_qpos_idx is not None and piano_dof_idx is not None and piano_qpos_idx.size > 0:
        pq = data.qpos[piano_qpos_idx].copy()
        pv = data.qvel[piano_dof_idx].copy()
        data.qpos[:] = default_qpos
        data.qpos[piano_qpos_idx] = pq
        data.qvel[piano_dof_idx] = pv
    else:
        data.qpos[:] = default_qpos
    for name, val in zip(header, row):
        mj_name = joint_alias.get(name, name)
        adr = name_to_qpos_adr.get(mj_name)
        if adr is None:
            continue
        jid = name_to_joint_id.get(mj_name, -1)
        if jid < 0:
            continue
        jt = int(model.jnt_type[jid])
        if jt == int(mujoco.mjtJoint.mjJNT_FREE):
            continue
        if jt in (int(mujoco.mjtJoint.mjJNT_SLIDE), int(mujoco.mjtJoint.mjJNT_HINGE)):
            data.qpos[adr] = val
        elif jt == int(mujoco.mjtJoint.mjJNT_BALL):
            raise NotImplementedError(f"关节 {mj_name} 为球关节，当前回放仅支持 hinge/slide")
        else:
            raise NotImplementedError(f"关节类型 {jt} 未实现")


def physics_step_kinematic_arms(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    piano_qpos_idx: np.ndarray,
    piano_dof_idx: np.ndarray,
    substeps: int,
) -> None:
    """对含被动白键铰链的模型做 mj_step；轨迹驱动的手臂关节在步进后锁回步进前的值。"""
    if piano_qpos_idx.size == 0:
        mujoco.mj_forward(model, data)
        return
    for _ in range(max(1, substeps)):
        snap_q = data.qpos.copy()
        snap_v = data.qvel.copy()
        mujoco.mj_step(model, data)
        pq_new = data.qpos[piano_qpos_idx].copy()
        pv_new = data.qvel[piano_dof_idx].copy()
        data.qpos[:] = snap_q
        data.qvel[:] = snap_v
        data.qpos[piano_qpos_idx] = pq_new
        data.qvel[piano_dof_idx] = pv_new


def _infer_default_urdf(robot_model_path: str) -> Optional[str]:
    """若 --tf-urdf 未显式指定，优先同名 .urdf，其次包内 urdf/ 目录中的 .urdf。"""
    p = Path(robot_model_path)
    cand = p.with_suffix(".urdf")
    if cand.is_file():
        return str(cand.resolve())
    pkg_urdf = p.parent.parent / "urdf"
    if pkg_urdf.is_dir():
        for u in pkg_urdf.glob("*.urdf"):
            return str(u.resolve())
    return None


def _push_arm_q_to_mujoco(
    tf_passes: Sequence[ArmPassSpec],
    tf_kins: Sequence[PinocchioKinematics],
    q_pin: np.ndarray,
    data: mujoco.MjData,
    name_to_qpos_adr: Dict[str, int],
) -> None:
    """把 Pinocchio q 中的 7 关节值写入 MuJoCo qpos。"""
    for spec, kin in zip(tf_passes, tf_kins):
        for jn, adr_pin in zip(spec.chain_joint_names, kin.ik_qpos_adr):
            mj_adr = name_to_qpos_adr.get(jn)
            if mj_adr is None:
                continue
            data.qpos[mj_adr] = float(q_pin[adr_pin])


def run() -> None:
    p = argparse.ArgumentParser(description="MuJoCo 读取 MJCF + 关节轨迹 .data 回放（FK/IK: Pinocchio）[仅 x86_64]")
    p.add_argument("--robot-model", required=True, help="MJCF/XML 路径（如 urdf/xml/...xml）")
    p.add_argument(
        "--data",
        required=True,
        help="轨迹 .data；data/ 与 output/ 前缀分别锚定包内 data、output 目录；仅文件名时先 data 再 output",
    )
    p.add_argument("--hz", type=float, default=100.0, help="回放帧率（与数据采样一致，如 100Hz）")
    p.add_argument("--speed", type=float, default=1.0, help="相对实时倍速")
    p.add_argument("--loop", action="store_true", help="播放结束后从头循环")
    p.add_argument(
        "--no-viewer",
        action="store_true",
        help="无可视化，仅推进仿真（用于快速校验）",
    )
    p.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="最多播放帧数，0 表示全部",
    )
    p.add_argument(
        "--tf-arm",
        choices=("left", "right", "both"),
        default=None,
        help="若设置，则每帧先按轨迹设 qpos，再对指定侧做 FK+SE(3)+IK（Pinocchio；与 arm_trajectory_transform 一致）",
    )
    p.add_argument(
        "--tf-urdf",
        default=None,
        help="Pinocchio FK/IK 所用 URDF 路径（仅在 --tf-arm 时生效）；未指定时自动推断同名或包内 urdf/*.urdf",
    )
    p.add_argument("--tf-offset-left", default=None, help="左臂偏移 x,y,z,rx,ry,rz（逗号分隔）")
    p.add_argument("--tf-offset-right", default=None, help="右臂偏移")
    p.add_argument("--tf-x", type=float, default=0.0)
    p.add_argument("--tf-y", type=float, default=0.0)
    p.add_argument("--tf-z", type=float, default=0.0)
    p.add_argument("--tf-rx", type=float, default=0.0)
    p.add_argument("--tf-ry", type=float, default=0.0)
    p.add_argument("--tf-rz", type=float, default=0.0)
    p.add_argument("--tf-length-unit", choices=("m", "mm"), default="mm")
    p.add_argument("--tf-angle-unit", choices=("rad", "deg"), default="rad")
    p.add_argument("--tf-euler-seq", default="XYZ")
    p.add_argument(
        "--tf-compose",
        choices=("left", "right", "world_ee"),
        default="left",
        help=(
            "实时 IK 偏移组合方式：left=H·T_ee；right=T_ee·H；"
            "world_ee=平移在世界系 + 末端原点就地绕世界轴旋转（与 Web UI 示教一致）"
        ),
    )
    p.add_argument("--tf-ik-iters", type=int, default=60)
    p.add_argument("--tf-ik-tol", type=float, default=1e-4)
    p.add_argument("--tf-ik-damping", type=float, default=1e-3)
    p.add_argument("--tf-ik-max-step", type=float, default=0.15)
    p.add_argument(
        "--kinematic-only",
        action="store_true",
        help="仅 mj_forward（旧行为）：无接触动力学，白键与穿模问题仍存在",
    )
    p.add_argument(
        "--mj-substeps",
        type=int,
        default=8,
        help="每轨迹帧内 mj_step 次数（手指–键接触建议 6～12）",
    )
    p.add_argument(
        "--piano-audio",
        action="store_true",
        help="白键按下时按关节名播放对应音高（需 sounddevice 或 pygame）",
    )
    p.add_argument(
        "--piano-audio-threshold",
        type=float,
        default=-0.004,
        help="铰链角(rad)低于该值视为按下触发琴音；仿真里常见仅 -0.003～-0.01，默认已放宽（约 -4e-3 rad）",
    )
    p.add_argument(
        "--piano-audio-debug",
        action="store_true",
        help="stderr 打印每次触发的关节角与 MIDI；用于确认是否达到阈值",
    )
    p.add_argument(
        "--piano-audio-test-beep",
        action="store_true",
        help="启动时播放一声中央 C，用于确认扬声器/后端正常",
    )
    args = p.parse_args()
    args.robot_model = resolve_model_path(args.robot_model)
    args.data = resolve_data_path(args.data)

    if not args.robot_model.lower().endswith((".xml", ".mjcf")):
        print("请使用 MJCF/XML 路径（--robot-model）。MuJoCo 仿真/渲染需要 MJCF。", file=sys.stderr)
        sys.exit(2)

    model = mujoco.MjModel.from_xml_path(args.robot_model)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    default_qpos = data.qpos.copy()

    header, frames = read_trajectory_table(args.data)
    n = frames.shape[0]
    if args.max_frames > 0:
        n = min(n, args.max_frames)

    name_to_qpos_adr, name_to_joint_id = _joint_maps(model)
    piano_qpos_idx, piano_dof_idx, piano_joint_names = piano_hinge_passive_indices(model)
    piano_audio: Optional[PianoKeyAudio] = None
    if args.piano_audio and piano_qpos_idx.size > 0:
        piano_audio = PianoKeyAudio(
            piano_joint_names,
            press_threshold_rad=args.piano_audio_threshold,
            debug=args.piano_audio_debug,
        )
        if piano_audio.backend == "none":
            print(
                "警告: --piano-audio 已开启但无可用音频后端。请安装其一："
                "sudo apt install pulseaudio-utils（提供 paplay）或 alsa-utils（提供 aplay）；"
                "或 pip install pygame / sounddevice。",
                file=sys.stderr,
            )
        else:
            print(f"琴音: 后端={piano_audio.backend}，阈值(rad)={args.piano_audio_threshold}", flush=True)
            if args.piano_audio_test_beep:
                piano_audio.play_test_beep(60)
                print("琴音: 已播放测试音(中央 C/MIDI 60)", flush=True)
    if piano_qpos_idx.size > 0 and not args.kinematic_only:
        print(
            f"被动白键铰链: {piano_qpos_idx.size} 个，接触动力学 mj_step（--mj-substeps {args.mj_substeps}）",
            flush=True,
        )
    alias = dict(CASBOT_FINGER_DATA_TO_MJCF)
    alias.update(BS_TRAJECTORY_JOINT_TO_MJCF)
    dt = 1.0 / max(args.hz, 1e-6)
    model.opt.timestep = dt

    tf_passes: Optional[List[ArmPassSpec]] = None
    tf_kins: Optional[List[PinocchioKinematics]] = None
    q_pin: Optional[np.ndarray] = None
    default_q_pin: Optional[np.ndarray] = None
    if args.tf_arm is not None:
        tf_urdf = args.tf_urdf
        if tf_urdf is None:
            tf_urdf = _infer_default_urdf(args.robot_model)
        if not tf_urdf:
            print(
                "--tf-arm 需要 Pinocchio URDF：请用 --tf-urdf 指定 .urdf，"
                "或在包内 urdf/ 目录放置同名 .urdf。",
                file=sys.stderr,
            )
            sys.exit(2)
        tf_urdf = resolve_model_path(tf_urdf) if not os.path.isabs(tf_urdf) else tf_urdf
        if not os.path.isfile(tf_urdf):
            print(f"--tf-urdf 指向的文件不存在: {tf_urdf}", file=sys.stderr)
            sys.exit(2)

        tf_ns = SimpleNamespace(
            arm=args.tf_arm,
            offset_left=args.tf_offset_left,
            offset_right=args.tf_offset_right,
            x=args.tf_x,
            y=args.tf_y,
            z=args.tf_z,
            rx=args.tf_rx,
            ry=args.tf_ry,
            rz=args.tf_rz,
            length_unit=args.tf_length_unit,
            angle_unit=args.tf_angle_unit,
            euler_seq=args.tf_euler_seq,
            compose=args.tf_compose,
            arm_joints=None,
            arm_joints_left=None,
            arm_joints_right=None,
            ik_joints=None,
            ik_joints_left=None,
            ik_joints_right=None,
            write_joints=None,
            write_joints_left=None,
            write_joints_right=None,
            ee_body=None,
            ee_body_left=None,
            ee_body_right=None,
        )
        tf_passes = collect_arm_passes(
            tf_ns,
            build_H=build_H,
            default_arm_joint_names=_default_arm_joint_names,
            default_ee_body_name=_default_ee_body_name,
            chain_key="ik",
        )
        first = PinocchioKinematics.from_urdf(
            tf_urdf, tf_passes[0].chain_joint_names, tf_passes[0].ee_body_name
        )
        tf_kins = [first]
        for spec in tf_passes[1:]:
            tf_kins.append(
                PinocchioKinematics.share_model(
                    first, spec.chain_joint_names, spec.ee_body_name
                )
            )
        default_q_pin = first.default_qpos()
        q_pin = default_q_pin.copy()
        print(
            f"实时变换: --tf-arm {args.tf_arm}, {len(tf_passes)} 条手臂链, "
            f"IK iters={args.tf_ik_iters} [后端: pinocchio @ {tf_urdf}]"
        )

    mj_cnt = sum(1 for h in header if alias.get(h, h) in name_to_qpos_adr)
    print(f"轨迹 {n} 帧, {mj_cnt}/{len(header)} 列映射到 MuJoCo 关节, dt={dt:.4f}s")

    def tf_step(frame_idx: int) -> None:
        assert tf_passes is not None and tf_kins is not None
        assert q_pin is not None and default_q_pin is not None
        apply_trajectory_row(
            model,
            data,
            header,
            frames[frame_idx],
            default_qpos,
            name_to_qpos_adr,
            name_to_joint_id,
            alias,
            piano_qpos_idx,
            piano_dof_idx,
        )
        step_frame_ik_passes(
            tf_kins,
            tf_passes,
            q_pin,
            header,
            frames[frame_idx],
            default_q_pin,
            alias,
            args.tf_ik_iters,
            args.tf_ik_tol,
            args.tf_ik_damping,
            args.tf_ik_max_step,
        )
        _push_arm_q_to_mujoco(tf_passes, tf_kins, q_pin, data, name_to_qpos_adr)
        if args.kinematic_only:
            mujoco.mj_forward(model, data)
        else:
            physics_step_kinematic_arms(
                model, data, piano_qpos_idx, piano_dof_idx, args.mj_substeps
            )

    def plain_step(frame_idx: int) -> None:
        apply_trajectory_row(
            model,
            data,
            header,
            frames[frame_idx],
            default_qpos,
            name_to_qpos_adr,
            name_to_joint_id,
            alias,
            piano_qpos_idx,
            piano_dof_idx,
        )
        if args.kinematic_only:
            mujoco.mj_forward(model, data)
        else:
            physics_step_kinematic_arms(
                model, data, piano_qpos_idx, piano_dof_idx, args.mj_substeps
            )

    do_step = tf_step if tf_passes is not None else plain_step

    if args.no_viewer:
        t0 = time.time()
        for i in range(n):
            do_step(i)
            if piano_audio is not None:
                piano_audio.update(data.qpos, piano_qpos_idx, data.qvel, piano_dof_idx)
        print(f"无可视回放完成, wall {time.time() - t0:.2f}s")
        return

    frame_i = 0
    last = time.time()
    paused = False

    def key_callback(keycode: int) -> None:
        nonlocal paused
        if keycode == 32:
            paused = not paused
            print(f"{'⏸  已暂停 (按空格继续)' if paused else '▶  继续播放'}")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "waist_yaw_link")
        if bid >= 0:
            viewer.cam.lookat[:] = data.xpos[bid]
        else:
            viewer.cam.lookat[:] = np.array([0.0, 0.0, 1.0])
        viewer.cam.distance = 3.0
        viewer.cam.elevation = -20.0
        viewer.cam.azimuth = 90.0

        while viewer.is_running():
            now = time.time()
            if not paused and now - last >= dt / max(args.speed, 1e-6):
                last = now
                do_step(frame_i)
                if piano_audio is not None:
                    piano_audio.update(
                        data.qpos, piano_qpos_idx, data.qvel, piano_dof_idx
                    )
                if bid >= 0:
                    viewer.cam.lookat[:] = data.xpos[bid]

                frame_i += 1
                if frame_i >= n:
                    if args.loop:
                        frame_i = 0
                    else:
                        frame_i = n - 1

            viewer.sync()


if __name__ == "__main__":
    run()
