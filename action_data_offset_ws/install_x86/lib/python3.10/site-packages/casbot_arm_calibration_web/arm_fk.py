"""
双臂末端（腕部连杆原点）相对 base_link 的正运动学。
从 URDF 解析关节链，仅用 numpy；不依赖 PyKDL。
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

LEFT_TIP_LINK = "left_wrist_roll_link"
RIGHT_TIP_LINK = "right_wrist_roll_link"
BASE_LINK = "base_link"


def _parse_xyz(s: str) -> np.ndarray:
    parts = s.split()
    return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=np.float64)


def _parse_rpy(s: str) -> np.ndarray:
    parts = s.split()
    return np.array([float(parts[0]), float(parts[1]), float(parts[2])], dtype=np.float64)


def _rpy_to_R(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return Rz @ Ry @ Rx


def rpy_to_R(rpy_roll_pitch_yaw: np.ndarray) -> np.ndarray:
    """由 (roll, pitch, yaw) 弧度构造 ``R = Rz·Ry·Rx``（与 URDF / 界面欧拉约定一致）。"""
    return _rpy_to_R(np.asarray(rpy_roll_pitch_yaw, dtype=np.float64))


def rotation_matrix_to_rpy(R: np.ndarray) -> np.ndarray:
    """
    与 :func:`_rpy_to_R` 同一约定：``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``。

    返回 ``(roll, pitch, yaw)``（弧度），顺序对应界面 **RX / RY / RZ**（绕 base 的 X/Y/Z 角）。
    """
    Rm = np.asarray(R, dtype=np.float64).reshape(3, 3)
    sp = -float(Rm[2, 0])
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    yaw = math.atan2(float(Rm[1, 0]), float(Rm[0, 0]))
    cp = math.cos(pitch)
    if abs(cp) > 1e-8:
        roll = math.atan2(float(Rm[2, 1]), float(Rm[2, 2]))
    else:
        roll = math.atan2(-float(Rm[0, 1]), float(Rm[1, 1]))
    return np.array([roll, pitch, yaw], dtype=np.float64)


def _T_from_xyz_rpy(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = _rpy_to_R(rpy)
    T[:3, 3] = xyz
    return T


def _rot_about_axis(axis: np.ndarray, q: float) -> np.ndarray:
    """Rodrigues: 绕单位轴 axis 旋转 q（弧度）。"""
    axis = np.asarray(axis, dtype=np.float64).reshape(3)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    k = axis / n
    c, s = math.cos(q), math.sin(q)
    x, y, z = k[0], k[1], k[2]
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=np.float64)
    R = np.eye(3) + s * K + (1.0 - c) * (K @ K)
    return R


class UrdfArmFk:
    """缓存 URDF 关节链与 FK。"""

    def __init__(self, urdf_path: Path) -> None:
        self._urdf_path = Path(urdf_path)
        self._joint_names: List[str] = []
        self._left_chain: List[str] = []
        self._right_chain: List[str] = []
        self._joint_parent: Dict[str, str] = {}
        self._joint_child: Dict[str, str] = {}
        self._joint_origin: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        self._joint_axis: Dict[str, np.ndarray] = {}
        self._loaded = False
        self._load_error: Optional[str] = None

    def load(self) -> bool:
        if self._loaded:
            return self._load_error is None
        try:
            tree = ET.parse(self._urdf_path)
            root = tree.getroot()
            for j_el in root.findall("joint"):
                name = j_el.attrib.get("name")
                jtype = j_el.attrib.get("type", "fixed")
                if jtype not in ("revolute", "continuous", "prismatic"):
                    continue
                parent = j_el.find("parent")
                child = j_el.find("child")
                origin = j_el.find("origin")
                axis_el = j_el.find("axis")
                if parent is None or child is None or name is None:
                    continue
                pl = parent.attrib.get("link")
                cl = child.attrib.get("link")
                if not pl or not cl:
                    continue
                xyz = _parse_xyz(origin.attrib.get("xyz", "0 0 0")) if origin is not None else np.zeros(3)
                rpy = _parse_rpy(origin.attrib.get("rpy", "0 0 0")) if origin is not None else np.zeros(3)
                ax = _parse_xyz(axis_el.attrib.get("xyz", "1 0 0")) if axis_el is not None else np.array([1.0, 0.0, 0.0])
                self._joint_parent[name] = pl
                self._joint_child[name] = cl
                self._joint_origin[name] = (xyz, rpy)
                self._joint_axis[name] = ax
                self._joint_names.append(name)

            self._left_chain = self._chain_between(BASE_LINK, LEFT_TIP_LINK)
            self._right_chain = self._chain_between(BASE_LINK, RIGHT_TIP_LINK)
            if not self._left_chain or not self._right_chain:
                self._load_error = "无法在 URDF 中解析 base→腕部连杆关节链"
                self._loaded = True
                return False
            self._loaded = True
            return True
        except Exception as e:
            self._load_error = str(e)
            self._loaded = True
            return False

    @property
    def load_error(self) -> Optional[str]:
        return self._load_error

    def _chain_between(self, start_link: str, end_link: str) -> List[str]:
        """从 start 到 end 的 child 方向：列出路径上的关节名（顺序从 base 到末端）。"""
        children: Dict[str, List[Tuple[str, str]]] = {}
        for jn, pl in self._joint_parent.items():
            cl = self._joint_child[jn]
            children.setdefault(pl, []).append((jn, cl))

        # BFS: (link, path_of_joint_names)
        from collections import deque

        q = deque([(start_link, [])])
        seen = {start_link}
        while q:
            link, path = q.popleft()
            if link == end_link:
                return path
            for jn, cl in children.get(link, []):
                if cl not in seen:
                    seen.add(cl)
                    q.append((cl, path + [jn]))
        return []

    def fk_tip_xyz(
        self,
        chain: List[str],
        name_to_q: Dict[str, float],
    ) -> Optional[np.ndarray]:
        """
        末端连杆原点相对 base_link 的位置（米）。
        name_to_q: 关节名 -> 弧度（与 URDF joint name 一致，如 left_shoulder_pitch_joint）。
        """
        if not chain:
            return None
        T = np.eye(4, dtype=np.float64)
        for jn in chain:
            xyz, rpy = self._joint_origin[jn]
            T0 = _T_from_xyz_rpy(xyz, rpy)
            axis = self._joint_axis[jn]
            q = float(name_to_q.get(jn, 0.0))
            Rq = _rot_about_axis(axis, q)
            Tq = np.eye(4, dtype=np.float64)
            Tq[:3, :3] = Rq
            T = T @ T0 @ Tq
        return T[:3, 3].copy()

    def fk_chain_transform(self, chain: List[str], name_to_q: Dict[str, float]) -> np.ndarray:
        """
        链末端（最后一个关节的子连杆）坐标系相对 base_link 的 4x4 齐次变换。

        ``p_base = T @ p_local``（齐次坐标）。与 :meth:`fk_tip_xyz` 使用相同的链上运动学。
        """
        if not chain:
            return np.eye(4, dtype=np.float64)
        T = np.eye(4, dtype=np.float64)
        for jn in chain:
            xyz, rpy = self._joint_origin[jn]
            T0 = _T_from_xyz_rpy(xyz, rpy)
            axis = self._joint_axis[jn]
            q = float(name_to_q.get(jn, 0.0))
            Rq = _rot_about_axis(axis, q)
            Tq = np.eye(4, dtype=np.float64)
            Tq[:3, :3] = Rq
            T = T @ T0 @ Tq
        return T

    def tip_angular_jacobian_numeric(
        self,
        chain: List[str],
        name_to_q: Dict[str, float],
        arm_joint_names: List[str],
        eps: float = 1e-6,
    ) -> np.ndarray:
        """
        数值雅可比：末端姿态在 **base_link** 下的角速度 ω（小旋转向量）对臂关节角的偏导。

        满足 dR/dq_i ≈ [J_ω(:,i)]× R，R 为当前末端旋转矩阵。
        """
        R0 = self.fk_chain_transform(chain, name_to_q)[:3, :3]
        n = len(arm_joint_names)
        Jw = np.zeros((3, n), dtype=np.float64)
        for i, jn in enumerate(arm_joint_names):
            q_plus = dict(name_to_q)
            q_plus[jn] = float(q_plus.get(jn, 0.0)) + eps
            q_minus = dict(name_to_q)
            q_minus[jn] = float(q_minus.get(jn, 0.0)) - eps
            Rp = self.fk_chain_transform(chain, q_plus)[:3, :3]
            Rm = self.fk_chain_transform(chain, q_minus)[:3, :3]
            dR = (Rp - Rm) / (2.0 * eps)
            M = dR @ R0.T
            Jw[:, i] = np.array(
                [(M[2, 1] - M[1, 2]) * 0.5, (M[0, 2] - M[2, 0]) * 0.5, (M[1, 0] - M[0, 1]) * 0.5],
                dtype=np.float64,
            )
        return Jw

    def tip_position_jacobian_numeric(
        self,
        chain: List[str],
        name_to_q: Dict[str, float],
        arm_joint_names: List[str],
        eps: float = 1e-6,
    ) -> np.ndarray:
        """
        数值雅可比：末端在 **base_link** 下的位置 (m) 对 ``arm_joint_names`` 关节角 (rad) 的偏导。

        与 :meth:`fk_tip_xyz` 使用同一套链与 FK，保证与界面末端坐标系一致。
        """
        J = np.zeros((3, len(arm_joint_names)), dtype=np.float64)
        for i, jn in enumerate(arm_joint_names):
            q_plus = dict(name_to_q)
            q_plus[jn] = float(q_plus.get(jn, 0.0)) + eps
            q_minus = dict(name_to_q)
            q_minus[jn] = float(q_minus.get(jn, 0.0)) - eps
            p_plus = self.fk_tip_xyz(chain, q_plus)
            p_minus = self.fk_tip_xyz(chain, q_minus)
            if p_plus is None or p_minus is None:
                continue
            J[:, i] = (p_plus - p_minus) / (2.0 * eps)
        return J


def build_q_map_for_chain(
    chain: List[str],
    names_msg: List[str],
    positions: List[float],
) -> Dict[str, float]:
    """
    将 JointState 与 URDF 关节链对齐：支持 `left_shoulder_pitch` 与 `left_shoulder_pitch_joint` 两种命名。
    缺失的关节角度按 0 处理。
    """
    idx = {n: i for i, n in enumerate(names_msg)}
    out: Dict[str, float] = {}
    for jn in chain:
        val = None
        if jn in idx:
            val = positions[idx[jn]]
        else:
            short = jn.replace("_joint", "")
            if short in idx:
                val = positions[idx[short]]
        out[jn] = float(val) if val is not None else 0.0
    return out
