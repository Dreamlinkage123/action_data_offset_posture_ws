"""
基于 Pinocchio 的左右臂 FK/IK。

设计要点
- 默认末端：左臂 ``left_wrist_roll_link``、右臂 ``right_wrist_roll_link``（URDF link 名）。
- 仅加载运动学模型（:func:`pinocchio.buildModelFromUrdf`），不依赖 mesh 文件。
- IK 采用阻尼最小二乘（DLS）一步迭代，支持与 ``arm_trajectory_transform.py`` 的收敛循环
  以及 ``arm_trajectory_transform_jacobian.py`` 的两步雅可比叠加直接对接。

依赖
- ``pinocchio``（ROS Humble 可用 ``ros-humble-pinocchio``）。
- NumPy 1.26.x（若 NumPy 2.x 可能与 Pinocchio ABI 不兼容，运行时会报错）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np


def _import_pinocchio():
    try:
        import pinocchio as pin  # noqa: PLC0415

        return pin
    except Exception as e:  # noqa: BLE001
        raise SystemExit(
            "无法导入 pinocchio（apt: ros-humble-pinocchio；pip: pin）。当前错误：" f"{e}"
        ) from e


def se3_from_Rt(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def pose_error(T_cur: np.ndarray, T_des: np.ndarray) -> np.ndarray:
    """6 维误差 [平移; 旋转轴角]，平移为 des - cur。"""
    R_cur, t_cur = T_cur[:3, :3], T_cur[:3, 3]
    R_des, t_des = T_des[:3, :3], T_des[:3, 3]
    e_trans = t_des - t_cur
    R_err = R_des @ R_cur.T
    tr = np.clip((np.trace(R_err) - 1.0) * 0.5, -1.0, 1.0)
    theta = float(np.arccos(tr))
    if theta < 1e-8:
        e_rot = np.zeros(3, dtype=np.float64)
    else:
        w = np.array(
            [
                R_err[2, 1] - R_err[1, 2],
                R_err[0, 2] - R_err[2, 0],
                R_err[1, 0] - R_err[0, 1],
            ],
            dtype=np.float64,
        )
        w = w / (2.0 * np.sin(theta))
        e_rot = theta * w
    return np.concatenate([e_trans, e_rot])


@dataclass
class PinocchioKinematics:
    """Pinocchio 运动学包装：一臂一个实例。

    - ``ik_joint_ids``：参与 IK/Jacobian 的关节在 model 中的 joint id。
    - ``ik_qpos_adr``：关节在 q 中的索引（一维关节）。
    - ``ik_dof_adr``：关节在 v 中的索引（用于 Jacobian 列选）。
    - ``name_to_qpos_adr``：模型中所有 1-DoF 关节名 → q 索引，用于把数据行按名字填入 q。
    """

    model: Any
    data: Any
    ee_frame_id: int
    ik_joint_ids: List[int]
    ik_qpos_adr: List[int]
    ik_dof_adr: List[int]
    name_to_qpos_adr: Dict[str, int]
    name_to_joint_id: Dict[str, int]
    _pin: Any = field(repr=False)
    _neutral_q: np.ndarray = field(repr=False)
    # 关节限位缓存（仅 1-DoF）：name -> (lo, hi)；非限位为 (-inf, +inf)
    _limits: Dict[str, Tuple[float, float]] = field(repr=False)

    @staticmethod
    def from_urdf(
        urdf_path: str,
        ik_joint_names: Sequence[str],
        ee_link_name: str,
    ) -> "PinocchioKinematics":
        pin = _import_pinocchio()
        model = pin.buildModelFromUrdf(urdf_path)
        data = model.createData()

        if not model.existFrame(ee_link_name):
            raise ValueError(f"URDF 模型中找不到末端 frame/link: {ee_link_name}")
        ee_frame_id = int(model.getFrameId(ee_link_name))

        name_to_qpos_adr: Dict[str, int] = {}
        name_to_joint_id: Dict[str, int] = {}
        limits: Dict[str, Tuple[float, float]] = {}
        lower = np.asarray(model.lowerPositionLimit, dtype=np.float64)
        upper = np.asarray(model.upperPositionLimit, dtype=np.float64)
        for jid in range(1, model.njoints):  # 0 是 universe
            jmodel = model.joints[jid]
            if jmodel.nq != 1:
                continue
            name = model.names[jid]
            idx_q = int(jmodel.idx_q)
            name_to_qpos_adr[name] = idx_q
            name_to_joint_id[name] = int(jid)
            lo = float(lower[idx_q]) if np.isfinite(lower[idx_q]) else -np.inf
            hi = float(upper[idx_q]) if np.isfinite(upper[idx_q]) else np.inf
            # 若上下限为零或反序（ROS URDF 里连续旋转关节常以 0/0 表示无限），视为无限
            if lo >= hi:
                lo, hi = -np.inf, np.inf
            limits[name] = (lo, hi)

        ik_joint_ids: List[int] = []
        ik_qpos_adr: List[int] = []
        ik_dof_adr: List[int] = []
        for jn in ik_joint_names:
            jid = name_to_joint_id.get(jn)
            if jid is None:
                raise ValueError(f"URDF 模型中找不到 1-DoF 关节: {jn}")
            jmodel = model.joints[jid]
            ik_joint_ids.append(int(jid))
            ik_qpos_adr.append(int(jmodel.idx_q))
            ik_dof_adr.append(int(jmodel.idx_v))

        return PinocchioKinematics(
            model=model,
            data=data,
            ee_frame_id=ee_frame_id,
            ik_joint_ids=ik_joint_ids,
            ik_qpos_adr=ik_qpos_adr,
            ik_dof_adr=ik_dof_adr,
            name_to_qpos_adr=name_to_qpos_adr,
            name_to_joint_id=name_to_joint_id,
            _pin=pin,
            _neutral_q=np.asarray(pin.neutral(model), dtype=np.float64).copy(),
            _limits=limits,
        )

    @staticmethod
    def share_model(
        src: "PinocchioKinematics",
        ik_joint_names: Sequence[str],
        ee_link_name: str,
    ) -> "PinocchioKinematics":
        """复用已加载的 model/data（左右臂共享同一机器人模型，避免二次解析 URDF）。"""
        if not src.model.existFrame(ee_link_name):
            raise ValueError(f"URDF 模型中找不到末端 frame/link: {ee_link_name}")
        ee_frame_id = int(src.model.getFrameId(ee_link_name))
        ik_joint_ids: List[int] = []
        ik_qpos_adr: List[int] = []
        ik_dof_adr: List[int] = []
        for jn in ik_joint_names:
            jid = src.name_to_joint_id.get(jn)
            if jid is None:
                raise ValueError(f"URDF 模型中找不到 1-DoF 关节: {jn}")
            jmodel = src.model.joints[jid]
            ik_joint_ids.append(int(jid))
            ik_qpos_adr.append(int(jmodel.idx_q))
            ik_dof_adr.append(int(jmodel.idx_v))
        return PinocchioKinematics(
            model=src.model,
            data=src.data,
            ee_frame_id=ee_frame_id,
            ik_joint_ids=ik_joint_ids,
            ik_qpos_adr=ik_qpos_adr,
            ik_dof_adr=ik_dof_adr,
            name_to_qpos_adr=src.name_to_qpos_adr,
            name_to_joint_id=src.name_to_joint_id,
            _pin=src._pin,
            _neutral_q=src._neutral_q,
            _limits=src._limits,
        )

    def default_qpos(self) -> np.ndarray:
        return self._neutral_q.copy()

    def apply_row_to_qpos(
        self,
        q: np.ndarray,
        joint_names: Sequence[str],
        row: np.ndarray,
        default_qpos: np.ndarray,
        joint_alias: Optional[Dict[str, str]] = None,
    ) -> np.ndarray:
        """以 ``default_qpos`` 为底，按 (name, val) 填入轨迹行。返回原地修改后的 q。"""
        alias = joint_alias or {}
        q[:] = default_qpos
        for name, val in zip(joint_names, row):
            mj_name = alias.get(name, name)
            adr = self.name_to_qpos_adr.get(mj_name)
            if adr is None:
                continue
            q[adr] = float(val)
        return q

    def fk_T_ee(self, q: np.ndarray) -> np.ndarray:
        """以 q 做一次正解，返回末端 4×4 位姿（世界系）。"""
        pin = self._pin
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.ee_frame_id)
        oMf = self.data.oMf[self.ee_frame_id]
        return se3_from_Rt(
            np.asarray(oMf.rotation, dtype=np.float64),
            np.asarray(oMf.translation, dtype=np.float64).reshape(3),
        )

    def _frame_jacobian_world(self, q: np.ndarray) -> np.ndarray:
        """返回末端 frame 的 6×nv 雅可比（LOCAL_WORLD_ALIGNED：线/角速度均表示在世界轴）。"""
        pin = self._pin
        J = pin.computeFrameJacobian(
            self.model,
            self.data,
            q,
            self.ee_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
        )
        return np.asarray(J, dtype=np.float64)

    def _clip_qpos(self, q: np.ndarray) -> None:
        """对 IK 所涉及的关节做硬限位裁剪。"""
        for jid, adr in zip(self.ik_joint_ids, self.ik_qpos_adr):
            name = self.model.names[jid]
            lo, hi = self._limits.get(name, (-np.inf, np.inf))
            if np.isfinite(lo) or np.isfinite(hi):
                q[adr] = float(np.clip(q[adr], lo, hi))

    def ik_step(
        self,
        q: np.ndarray,
        T_des: np.ndarray,
        damping: float,
        max_step: float,
    ) -> float:
        """一步阻尼最小二乘 IK；仅更新选中的 7 关节。返回叠加前误差范数。"""
        T_cur = self.fk_T_ee(q)
        e = pose_error(T_cur, T_des)
        err_before = float(np.linalg.norm(e))

        J = self._frame_jacobian_world(q)
        cols = np.asarray(self.ik_dof_adr, dtype=int)
        J_sub = J[:, cols]
        lam = float(damping)
        M = J_sub @ J_sub.T + (lam**2) * np.eye(6, dtype=np.float64)
        delta = np.linalg.solve(M, e)
        dq = J_sub.T @ delta
        n = float(np.linalg.norm(dq))
        if n > max_step and n > 1e-12:
            dq *= max_step / n
        for adr, inc in zip(self.ik_qpos_adr, dq):
            q[adr] = float(q[adr] + inc)
        self._clip_qpos(q)
        return err_before

    def apply_jacobian_delta(
        self,
        q: np.ndarray,
        T_end_transform: np.ndarray,
        damping: float,
        max_delta_norm: float,
    ) -> Tuple[np.ndarray, float]:
        """返回 (Δq 向量, 叠加前误差范数 ||e||)；Δq 已叠加、已限位。"""
        T_cur = self.fk_T_ee(q)
        e = pose_error(T_cur, T_end_transform)
        err_before = float(np.linalg.norm(e))

        J = self._frame_jacobian_world(q)
        cols = np.asarray(self.ik_dof_adr, dtype=int)
        J_sub = J[:, cols]
        lam = float(damping)
        M = J_sub @ J_sub.T + (lam**2) * np.eye(6, dtype=np.float64)
        delta = np.linalg.solve(M, e)
        dq = J_sub.T @ delta
        n = float(np.linalg.norm(dq))
        if n > max_delta_norm and n > 1e-12:
            dq *= max_delta_norm / n
        for adr, inc in zip(self.ik_qpos_adr, dq):
            q[adr] = float(q[adr] + inc)
        self._clip_qpos(q)
        return dq, err_before
