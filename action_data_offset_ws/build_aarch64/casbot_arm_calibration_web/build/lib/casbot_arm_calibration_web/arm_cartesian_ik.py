"""
基于 base_link 的笛卡尔直线 IK 路点生成。

- FK/Jacobian 复用 :mod:`arm_fk` 的 :class:`UrdfArmFk`，保证与界面「当前值」同一套运动学。
- 位移方向/步长均在 **base_link** 系下解释，单位 **0.1 mm**。
- 按 ``step_size_01mm`` 逐步积分，每步用 DLS(Pseudo-Inverse) 做位置雅可比逆解；
  每一步都累计到 ``results`` 中，**调用方把全部路点按 100Hz 播放，末端即沿直线移动**。
- 仅关心位置（3 维），姿态会漂移；测量点若不在 ``*_wrist_roll_link`` 原点，手上远点可能多几毫米弧长。
- 主循环后对 **腕部连杆原点** 做终端位置闭环修正，减小分步雅可比累积误差。
- :func:`cartesian_rotate_waypoints`：绕 ``base_link`` 的 X/Y/Z（RX/RY/RZ）做定轴旋转示教；
  内部按固定角度步长分步，6 维任务（位置保持 + 角速度）DLS 解算，末端再做位姿闭环修正。
- 若已安装 **tracikpy**，:func:`cartesian_linear_waypoints_trac` /
  :func:`cartesian_rotate_waypoints_trac` 用 TRAC‑IK 按子步解算，避免数值雅可比 DLS（见节点参数 ``use_trac_ik``）。
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

from casbot_arm_calibration_web.arm_fk import UrdfArmFk
from casbot_arm_calibration_web.kin_pin_trac import KinematicsPinTrac


def parse_joint_limits(
    urdf_path: Path,
    joint_names: List[str],
) -> Dict[str, Tuple[float, float]]:
    """从 URDF 读取指定关节名的 (lower, upper) 限位（弧度），找不到时按 (-π, π)。"""
    limits: Dict[str, Tuple[float, float]] = {}
    default = (-math.pi, math.pi)
    try:
        tree = ET.parse(str(urdf_path))
        root = tree.getroot()
        for j_el in root.findall("joint"):
            name = j_el.attrib.get("name")
            if name not in joint_names:
                continue
            lim = j_el.find("limit")
            if lim is not None:
                try:
                    lower = float(lim.attrib.get("lower", default[0]))
                    upper = float(lim.attrib.get("upper", default[1]))
                except (TypeError, ValueError):
                    lower, upper = default
            else:
                lower, upper = default
            limits[name] = (lower, upper)
    except Exception:
        pass
    for jn in joint_names:
        limits.setdefault(jn, default)
    return limits


def _dls_pinv(J: np.ndarray, lambd: float) -> np.ndarray:
    """DLS 伪逆：J 为 m×n，返回 n×m（适用于 m≤n 的常见机器人雅可比）。"""
    JJT = J @ J.T
    m = JJT.shape[0]
    return J.T @ np.linalg.inv(JJT + (lambd * lambd) * np.eye(m, dtype=float))


def _vee_log_so3(R: np.ndarray) -> np.ndarray:
    """SO(3) 对数映射为旋转矢量（轴×角，|v|≤π）。"""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    tr = float(np.trace(R))
    cos_theta = np.clip((tr - 1.0) * 0.5, -1.0, 1.0)
    theta = float(math.acos(cos_theta))
    if theta < 1e-8:
        return np.zeros(3, dtype=float)
    rx = R[2, 1] - R[1, 2]
    ry = R[0, 2] - R[2, 0]
    rz = R[1, 0] - R[0, 1]
    s = math.sin(theta)
    if abs(s) < 1e-6:
        # θ≈π：用大对角元找转轴
        i = int(np.argmax(np.diag(R)))
        j = (i + 1) % 3
        k = (i + 2) % 3
        axis = np.zeros(3, dtype=float)
        axis[i] = math.sqrt(max(0.0, (R[i, i] + 1.0) * 0.5))
        axis[j] = R[i, j] / (2.0 * axis[i] + 1e-12)
        axis[k] = R[i, k] / (2.0 * axis[i] + 1e-12)
        n = float(np.linalg.norm(axis))
        if n < 1e-12:
            return np.zeros(3, dtype=float)
        return axis / n * theta
    omega = np.array([rx, ry, rz], dtype=float) * 0.5
    return omega * (theta / s)


def _rot_unit_axis(axis: str) -> np.ndarray:
    a = str(axis).strip().lower()
    if a in ("rx", "x"):
        return np.array([1.0, 0.0, 0.0], dtype=float)
    if a in ("ry", "y"):
        return np.array([0.0, 1.0, 0.0], dtype=float)
    if a in ("rz", "z"):
        return np.array([0.0, 0.0, 1.0], dtype=float)
    raise ValueError("axis 应为 rx/ry/rz（或 x/y/z）")


def _R_fixed_axis(angle_rad: float, axis_unit: np.ndarray) -> np.ndarray:
    """base_link 下定轴旋转矩阵（右手系）。"""
    u = np.asarray(axis_unit, dtype=float).reshape(3)
    n = float(np.linalg.norm(u))
    if n < 1e-12:
        return np.eye(3, dtype=float)
    k = u / n
    x, y, z = float(k[0]), float(k[1]), float(k[2])
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ],
        dtype=float,
    )


def cartesian_linear_waypoints(
    fk: UrdfArmFk,
    chain_base_to_tip: List[str],
    arm_joint_names: List[str],
    chain_joint_q_base: Dict[str, float],
    current_arm_q_rad: List[float],
    dx_01mm: float,
    dy_01mm: float,
    dz_01mm: float,
    joint_limits: Dict[str, Tuple[float, float]],
    step_size_01mm: float = 1.0,
    damping: float = 0.005,
) -> List[Dict]:
    """
    生成一串 base_link 下的直线路点（每点 ``step_size_01mm × 0.1 mm``）。

    Parameters
    ----------
    fk
        已 load() 的 :class:`UrdfArmFk`。
    chain_base_to_tip
        从 ``base_link`` 到腕部连杆的关节名序列（应覆盖 ``arm_joint_names``）。
    arm_joint_names
        需要求解的臂 7 关节名（按 URDF 链顺序）。
    chain_joint_q_base
        链上各关节当前角（rad），通常来自 /joint_states 缓存；臂 7 关节会被 ``current_arm_q_rad`` 覆盖。
    current_arm_q_rad
        臂 7 关节当前角（rad），长度需等于 ``arm_joint_names``。
    dx_01mm, dy_01mm, dz_01mm
        目标位移（单位 0.1 mm，base_link 系）。
    joint_limits
        关节限位，名称→(lower, upper)（rad）。
    step_size_01mm
        每个内部路点沿目标方向的步长（0.1 mm）。
    damping
        DLS 阻尼系数。

    Returns
    -------
    list of dict
        每项形如::

            {
                "step": i,                               # 1-based
                "ee_position_m": {"x":.., "y":.., "z":..},
                "joint_angles_rad": {jn: rad, ...},      # 仅含 arm_joint_names
            }
    """
    n = len(arm_joint_names)
    if len(current_arm_q_rad) != n:
        raise ValueError(
            f"current_arm_q_rad 长度 {len(current_arm_q_rad)} 与 arm_joint_names {n} 不一致"
        )
    for jn in arm_joint_names:
        if jn not in chain_base_to_tip:
            raise ValueError(f"arm joint {jn} 不在 FK 链里")

    total = np.array([dx_01mm, dy_01mm, dz_01mm], dtype=float) * 1e-4  # 米
    dist = float(np.linalg.norm(total))
    if dist < 1e-12:
        return []

    direction = total / dist
    step_m_target = float(step_size_01mm) * 1e-4
    # 把 dist 均分为 n_steps 段，确保每段长度 ≤ step_m_target，且最终累计恰好 dist
    n_steps = max(1, int(math.ceil(dist / step_m_target)))
    step_m = dist / n_steps

    lower = np.array([joint_limits.get(jn, (-math.pi, math.pi))[0] for jn in arm_joint_names], dtype=float)
    upper = np.array([joint_limits.get(jn, (-math.pi, math.pi))[1] for jn in arm_joint_names], dtype=float)

    q = np.asarray(current_arm_q_rad, dtype=float).copy()

    name_q0 = dict(chain_joint_q_base)
    for k, jn in enumerate(arm_joint_names):
        name_q0[jn] = float(current_arm_q_rad[k])
    p0 = fk.fk_tip_xyz(chain_base_to_tip, name_q0)
    if p0 is None:
        return []
    p_target_arr = np.asarray(p0, dtype=float) + total

    results: List[Dict] = []
    for i in range(1, n_steps + 1):
        name_q = dict(chain_joint_q_base)
        for k, jn in enumerate(arm_joint_names):
            name_q[jn] = float(q[k])
        J = fk.tip_position_jacobian_numeric(chain_base_to_tip, name_q, arm_joint_names)
        J_inv = _dls_pinv(J, damping)
        dq = J_inv @ (direction * step_m)
        q = np.clip(q + dq, lower, upper)

        name_q_new = dict(chain_joint_q_base)
        for k, jn in enumerate(arm_joint_names):
            name_q_new[jn] = float(q[k])
        p = fk.fk_tip_xyz(chain_base_to_tip, name_q_new)
        if p is None:
            break
        results.append(
            {
                "step": i,
                "ee_position_m": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])},
                "joint_angles_rad": {jn: float(q[k]) for k, jn in enumerate(arm_joint_names)},
            }
        )

    # 终端位置修正：分步积分仅一阶近似，易累积几毫米误差；用 p_target 闭环收敛到目标腕部原点
    pos_tol_m = 8e-5  # 80 µm
    max_refine = 25
    max_dq_norm = 0.03  # rad，单步关节增量上限
    step_idx = len(results)
    for _ in range(max_refine):
        name_q_new = dict(chain_joint_q_base)
        for k, jn in enumerate(arm_joint_names):
            name_q_new[jn] = float(q[k])
        p = fk.fk_tip_xyz(chain_base_to_tip, name_q_new)
        if p is None:
            break
        err = p_target_arr - np.asarray(p, dtype=float)
        if float(np.linalg.norm(err)) < pos_tol_m:
            break
        J = fk.tip_position_jacobian_numeric(chain_base_to_tip, name_q_new, arm_joint_names)
        J_inv = _dls_pinv(J, damping)
        dq = J_inv @ err
        dn = float(np.linalg.norm(dq))
        if dn > max_dq_norm:
            dq = dq * (max_dq_norm / dn)
        q = np.clip(q + dq, lower, upper)
        step_idx += 1
        name_q2 = dict(chain_joint_q_base)
        for k, jn in enumerate(arm_joint_names):
            name_q2[jn] = float(q[k])
        p2 = fk.fk_tip_xyz(chain_base_to_tip, name_q2)
        if p2 is None:
            break
        results.append(
            {
                "step": step_idx,
                "ee_position_m": {"x": float(p2[0]), "y": float(p2[1]), "z": float(p2[2])},
                "joint_angles_rad": {jn: float(q[k]) for k, jn in enumerate(arm_joint_names)},
            }
        )

    return results


def cartesian_rotate_waypoints(
    fk: UrdfArmFk,
    chain_base_to_tip: List[str],
    arm_joint_names: List[str],
    chain_joint_q_base: Dict[str, float],
    current_arm_q_rad: List[float],
    axis: str,
    total_angle_rad: float,
    joint_limits: Dict[str, Tuple[float, float]],
    step_angle_rad: float,
    damping: float = 0.005,
) -> List[Dict]:
    """
    绕 **base_link** 的 X/Y/Z（RX/RY/RZ）定轴旋转腕部连杆坐标系，**尽量保持腕部原点位置不变**。

    将 ``|total_angle_rad|`` 均分为若干段，每段不超过 ``step_angle_rad``（弧度），
    每步用 6×N 位姿雅可比 DLS 解 ``[Δp; ω] = [0; u·Δθ]``。
    """
    n = len(arm_joint_names)
    if len(current_arm_q_rad) != n:
        raise ValueError(
            f"current_arm_q_rad 长度 {len(current_arm_q_rad)} 与 arm_joint_names {n} 不一致"
        )
    for jn in arm_joint_names:
        if jn not in chain_base_to_tip:
            raise ValueError(f"arm joint {jn} 不在 FK 链里")

    if abs(float(total_angle_rad)) < 1e-12:
        return []

    u = _rot_unit_axis(axis)
    u = u / (float(np.linalg.norm(u)) + 1e-12)

    sa = abs(float(step_angle_rad))
    if sa < 1e-12:
        raise ValueError("step_angle_rad 须为正")
    n_steps = max(1, int(math.ceil(abs(float(total_angle_rad)) / sa)))
    dtheta = float(total_angle_rad) / n_steps

    lower = np.array([joint_limits.get(jn, (-math.pi, math.pi))[0] for jn in arm_joint_names], dtype=float)
    upper = np.array([joint_limits.get(jn, (-math.pi, math.pi))[1] for jn in arm_joint_names], dtype=float)
    q = np.asarray(current_arm_q_rad, dtype=float).copy()

    name_q0 = dict(chain_joint_q_base)
    for k, jn in enumerate(arm_joint_names):
        name_q0[jn] = float(current_arm_q_rad[k])
    p0 = fk.fk_tip_xyz(chain_base_to_tip, name_q0)
    T0 = fk.fk_chain_transform(chain_base_to_tip, name_q0)
    if p0 is None:
        return []
    R0 = T0[:3, :3].copy()
    p_target_arr = np.asarray(p0, dtype=float)
    R_target = _R_fixed_axis(float(total_angle_rad), u) @ R0

    results: List[Dict] = []
    for i in range(1, n_steps + 1):
        name_q = dict(chain_joint_q_base)
        for k, jn in enumerate(arm_joint_names):
            name_q[jn] = float(q[k])
        Jv = fk.tip_position_jacobian_numeric(chain_base_to_tip, name_q, arm_joint_names)
        Jw = fk.tip_angular_jacobian_numeric(chain_base_to_tip, name_q, arm_joint_names)
        J = np.vstack([Jv, Jw])
        task = np.concatenate([np.zeros(3, dtype=float), u * dtheta])
        dq = _dls_pinv(J, damping) @ task
        q = np.clip(q + dq, lower, upper)

        name_q_new = dict(chain_joint_q_base)
        for k, jn in enumerate(arm_joint_names):
            name_q_new[jn] = float(q[k])
        p = fk.fk_tip_xyz(chain_base_to_tip, name_q_new)
        if p is None:
            break
        results.append(
            {
                "step": i,
                "ee_position_m": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])},
                "joint_angles_rad": {jn: float(q[k]) for k, jn in enumerate(arm_joint_names)},
            }
        )

    pos_tol_m = 8e-5
    rot_tol_rad = float(math.radians(0.12))
    max_refine = 30
    max_dq_norm = 0.05
    step_idx = len(results)
    for _ in range(max_refine):
        name_q_new = dict(chain_joint_q_base)
        for k, jn in enumerate(arm_joint_names):
            name_q_new[jn] = float(q[k])
        Tc = fk.fk_chain_transform(chain_base_to_tip, name_q_new)
        pc = fk.fk_tip_xyz(chain_base_to_tip, name_q_new)
        if pc is None:
            break
        Rc = Tc[:3, :3]
        err_p = p_target_arr - np.asarray(pc, dtype=float)
        R_err = R_target @ Rc.T
        err_r = _vee_log_so3(R_err)
        if float(np.linalg.norm(err_p)) < pos_tol_m and float(np.linalg.norm(err_r)) < rot_tol_rad:
            break
        err = np.concatenate([err_p, err_r])
        Jv = fk.tip_position_jacobian_numeric(chain_base_to_tip, name_q_new, arm_joint_names)
        Jw = fk.tip_angular_jacobian_numeric(chain_base_to_tip, name_q_new, arm_joint_names)
        J = np.vstack([Jv, Jw])
        dq = _dls_pinv(J, damping) @ err
        dn = float(np.linalg.norm(dq))
        if dn > max_dq_norm:
            dq = dq * (max_dq_norm / dn)
        q = np.clip(q + dq, lower, upper)
        step_idx += 1
        name_q2 = dict(chain_joint_q_base)
        for k, jn in enumerate(arm_joint_names):
            name_q2[jn] = float(q[k])
        p2 = fk.fk_tip_xyz(chain_base_to_tip, name_q2)
        if p2 is None:
            break
        results.append(
            {
                "step": step_idx,
                "ee_position_m": {"x": float(p2[0]), "y": float(p2[1]), "z": float(p2[2])},
                "joint_angles_rad": {jn: float(q[k]) for k, jn in enumerate(arm_joint_names)},
            }
        )

    return results


def _clip_arm_joints(
    merged: Dict[str, float],
    arm_joint_names: List[str],
    joint_limits: Dict[str, Tuple[float, float]],
) -> None:
    for jn in arm_joint_names:
        lo, hi = joint_limits.get(jn, (-math.pi, math.pi))
        merged[jn] = float(np.clip(merged[jn], lo, hi))


def cartesian_linear_waypoints_trac(
    kin: KinematicsPinTrac,
    side: str,
    chain_base_to_tip: List[str],
    arm_joint_names: List[str],
    chain_joint_q_base: Dict[str, float],
    current_arm_q_rad: List[float],
    dx_01mm: float,
    dy_01mm: float,
    dz_01mm: float,
    joint_limits: Dict[str, Tuple[float, float]],
    step_size_01mm: float = 1.0,
) -> List[Dict]:
    """
    与 :func:`cartesian_linear_waypoints` 相同输入语义，每子步用 TRAC‑IK 解目标位姿
    （位置沿直线、姿态保持起始 R）。
    """
    n = len(arm_joint_names)
    if len(current_arm_q_rad) != n:
        raise ValueError(
            f"current_arm_q_rad 长度 {len(current_arm_q_rad)} 与 arm_joint_names {n} 不一致"
        )
    for jn in arm_joint_names:
        if jn not in chain_base_to_tip:
            raise ValueError(f"arm joint {jn} 不在 FK 链里")
    if not kin.available:
        return []

    total = np.array([dx_01mm, dy_01mm, dz_01mm], dtype=float) * 1e-4
    dist = float(np.linalg.norm(total))
    if dist < 1e-12:
        return []
    direction = total / dist
    step_m_target = float(step_size_01mm) * 1e-4
    n_steps = max(1, int(math.ceil(dist / step_m_target)))
    step_m = dist / n_steps

    merged: Dict[str, float] = dict(chain_joint_q_base)
    for k, jn in enumerate(arm_joint_names):
        merged[jn] = float(current_arm_q_rad[k])

    T0 = kin.fk_tip_T(side, merged)
    if T0 is None:
        return []
    p0 = np.asarray(T0[:3, 3], dtype=float).copy()
    R0 = np.asarray(T0[:3, :3], dtype=float).copy()
    p_target_arr = p0 + total

    results: List[Dict] = []
    for i in range(1, n_steps + 1):
        p_i = p0 + direction * (i * step_m)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R0
        T[:3, 3] = p_i
        q_sol = kin.solve_ik(side, T, merged)
        if q_sol is None:
            break
        arm_d = kin.q_chain_to_arm_dict(side, q_sol, arm_joint_names)
        for jn, v in arm_d.items():
            merged[jn] = v
        _clip_arm_joints(merged, arm_joint_names, joint_limits)
        p = kin.fk_tip_xyz_trac(side, merged)
        if p is None:
            break
        results.append(
            {
                "step": i,
                "ee_position_m": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])},
                "joint_angles_rad": {jn: float(merged[jn]) for jn in arm_joint_names},
            }
        )

    pos_tol_m = 8e-5
    max_refine = 25
    step_idx = len(results)
    for _ in range(max_refine):
        p = kin.fk_tip_xyz_trac(side, merged)
        if p is None:
            break
        err = p_target_arr - np.asarray(p, dtype=float)
        if float(np.linalg.norm(err)) < pos_tol_m:
            break
        T_fix = np.eye(4, dtype=np.float64)
        T_fix[:3, :3] = R0
        T_fix[:3, 3] = p_target_arr
        q_sol = kin.solve_ik(side, T_fix, merged)
        if q_sol is None:
            break
        arm_d = kin.q_chain_to_arm_dict(side, q_sol, arm_joint_names)
        for jn, v in arm_d.items():
            merged[jn] = v
        _clip_arm_joints(merged, arm_joint_names, joint_limits)
        step_idx += 1
        p2 = kin.fk_tip_xyz_trac(side, merged)
        if p2 is None:
            break
        results.append(
            {
                "step": step_idx,
                "ee_position_m": {"x": float(p2[0]), "y": float(p2[1]), "z": float(p2[2])},
                "joint_angles_rad": {jn: float(merged[jn]) for jn in arm_joint_names},
            }
        )

    return results


def cartesian_rotate_waypoints_trac(
    kin: KinematicsPinTrac,
    side: str,
    chain_base_to_tip: List[str],
    arm_joint_names: List[str],
    chain_joint_q_base: Dict[str, float],
    current_arm_q_rad: List[float],
    axis: str,
    total_angle_rad: float,
    joint_limits: Dict[str, Tuple[float, float]],
    step_angle_rad: float,
) -> List[Dict]:
    """与 :func:`cartesian_rotate_waypoints` 相同任务，每子步 TRAC‑IK。"""
    n = len(arm_joint_names)
    if len(current_arm_q_rad) != n:
        raise ValueError(
            f"current_arm_q_rad 长度 {len(current_arm_q_rad)} 与 arm_joint_names {n} 不一致"
        )
    for jn in arm_joint_names:
        if jn not in chain_base_to_tip:
            raise ValueError(f"arm joint {jn} 不在 FK 链里")
    if not kin.available:
        return []
    if abs(float(total_angle_rad)) < 1e-12:
        return []

    u = _rot_unit_axis(axis)
    u = u / (float(np.linalg.norm(u)) + 1e-12)
    sa = abs(float(step_angle_rad))
    if sa < 1e-12:
        raise ValueError("step_angle_rad 须为正")
    n_steps = max(1, int(math.ceil(abs(float(total_angle_rad)) / sa)))
    dtheta = float(total_angle_rad) / n_steps
    R_delta = _R_fixed_axis(dtheta, u)

    merged: Dict[str, float] = dict(chain_joint_q_base)
    for k, jn in enumerate(arm_joint_names):
        merged[jn] = float(current_arm_q_rad[k])

    T0 = kin.fk_tip_T(side, merged)
    if T0 is None:
        return []
    p0 = np.asarray(T0[:3, 3], dtype=float).copy()
    R0 = np.asarray(T0[:3, :3], dtype=float).copy()
    R_target = _R_fixed_axis(float(total_angle_rad), u) @ R0

    R_curr = R0.copy()
    results: List[Dict] = []
    for i in range(1, n_steps + 1):
        R_curr = R_delta @ R_curr
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R_curr
        T[:3, 3] = p0
        q_sol = kin.solve_ik(side, T, merged)
        if q_sol is None:
            break
        arm_d = kin.q_chain_to_arm_dict(side, q_sol, arm_joint_names)
        for jn, v in arm_d.items():
            merged[jn] = v
        _clip_arm_joints(merged, arm_joint_names, joint_limits)
        p = kin.fk_tip_xyz_trac(side, merged)
        if p is None:
            break
        results.append(
            {
                "step": i,
                "ee_position_m": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])},
                "joint_angles_rad": {jn: float(merged[jn]) for jn in arm_joint_names},
            }
        )

    pos_tol_m = 8e-5
    rot_tol_rad = float(math.radians(0.12))
    max_refine = 30
    step_idx = len(results)
    for _ in range(max_refine):
        Tc = kin.fk_tip_T(side, merged)
        if Tc is None:
            break
        pc = np.asarray(Tc[:3, 3], dtype=float)
        Rc = np.asarray(Tc[:3, :3], dtype=float)
        err_p = p0 - pc
        R_err = R_target @ Rc.T
        err_r = _vee_log_so3(R_err)
        if float(np.linalg.norm(err_p)) < pos_tol_m and float(np.linalg.norm(err_r)) < rot_tol_rad:
            break
        T_des = np.eye(4, dtype=np.float64)
        T_des[:3, :3] = R_target
        T_des[:3, 3] = p0
        q_sol = kin.solve_ik(side, T_des, merged)
        if q_sol is None:
            break
        arm_d = kin.q_chain_to_arm_dict(side, q_sol, arm_joint_names)
        for jn, v in arm_d.items():
            merged[jn] = v
        _clip_arm_joints(merged, arm_joint_names, joint_limits)
        step_idx += 1
        p2 = kin.fk_tip_xyz_trac(side, merged)
        if p2 is None:
            break
        results.append(
            {
                "step": step_idx,
                "ee_position_m": {"x": float(p2[0]), "y": float(p2[1]), "z": float(p2[2])},
                "joint_angles_rad": {jn: float(merged[jn]) for jn in arm_joint_names},
            }
        )

    return results
