"""
Pinocchio 正运动学 + 阻尼最小二乘迭代逆运动学（不再依赖 tracikpy）。

- Web 示教笛卡尔直线 / 姿态旋转：每子步对目标位姿做 Pinocchio 帧雅可比 IK。
- ``/joint_states`` 末端位姿优先用 Pinocchio 帧正解（失败时回退 :mod:`arm_fk`）。
- 依赖：``pinocchio``（如 ``ros-humble-pinocchio``）；**NumPy 建议 1.26.x（<2）**，
  与 Humble 自带扩展 ABI 一致。NumPy 2.x 默认跳过 Pinocchio（见 :func:`_load_pin_module`）。
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

_log = logging.getLogger(__name__)

from casbot_arm_calibration_web.arm_fk import BASE_LINK, LEFT_TIP_LINK, RIGHT_TIP_LINK


def _load_pin_module():
    """
    延迟导入 pinocchio。ROS Humble 自带扩展通常针对 NumPy 1.x；**NumPy 2.x** 下默认跳过，
    避免 ABI 不兼容。设置 ``CASBOT_FORCE_PINOCCHIO=1`` 可仍尝试加载。
    """
    import os

    if os.environ.get("CASBOT_FORCE_PINOCCHIO", "").strip() not in ("1", "true", "yes"):
        try:
            major = int(np.__version__.split(".")[0])
        except (TypeError, ValueError):
            major = 0
        if major >= 2:
            _log.info(
                "NumPy %s：跳过 Pinocchio（避免与 NumPy 2 ABI 不兼容）；示教 IK 将不可用",
                np.__version__,
            )
            return None
    try:
        import pinocchio as pin_mod  # noqa: PLC0415

        return pin_mod
    except BaseException as e:  # noqa: BLE001
        _log.debug("pinocchio 导入失败: %s", e)
        return None


def _joint_name_candidates(jn0: str) -> Tuple[str, ...]:
    s = str(jn0)
    return (
        s,
        s if str(s).endswith("_joint") else f"{s}_joint",
        str(s).replace("_joint", "") + "_joint",
    )


def _fill_pin_q(pin_mod, model, joint_positions_by_name: Dict[str, float]) -> np.ndarray:
    """将 JointState 风格名→角（rad）填入 Pinocchio 配置向量。"""
    q = pin_mod.neutral(model)
    for jn0, val in joint_positions_by_name.items():
        for cand in _joint_name_candidates(jn0):
            if not model.existJointName(cand):
                continue
            jid = model.getJointId(cand)
            jmodel = model.joints[jid]
            if jmodel.nq != 1:
                break
            q[jmodel.idx_q] = float(val)
            break
    return q


def _idx_q_for_joint_name(model, jn0: str) -> Optional[int]:
    for cand in _joint_name_candidates(jn0):
        if not model.existJointName(cand):
            continue
        jid = model.getJointId(cand)
        jm = model.joints[jid]
        if jm.nq == 1:
            return int(jm.idx_q)
        continue
    return None


def _velocity_cols_for_joint_names(model, joint_names: List[str]) -> List[int]:
    """Pinocchio 速度空间下标列表（与 ``computeFrameJacobian`` 列顺序一致）。"""
    cols: List[int] = []
    for jn in joint_names:
        added = False
        for cand in _joint_name_candidates(jn):
            if not model.existJointName(cand):
                continue
            jid = model.getJointId(cand)
            jm = model.joints[jid]
            for k in range(jm.nv):
                cols.append(int(jm.idx_v + k))
            added = True
            break
        if not added:
            _log.debug("velocity cols: joint %s not found in model", jn)
    return cols


def _dls_pinv(J: np.ndarray, lambd: float) -> np.ndarray:
    """DLS 伪逆：J 为 m×n，返回 n×m。"""
    JJT = J @ J.T
    m = JJT.shape[0]
    return J.T @ np.linalg.inv(JJT + (lambd * lambd) * np.eye(m, dtype=float))


def _log6_vec(pin_mod, iMd) -> np.ndarray:
    """SE3 相对误差 → 6 维空间速度向量（numpy）。"""
    m = pin_mod.log6(iMd)
    if hasattr(m, "np"):
        return np.asarray(m.np, dtype=np.float64).reshape(6)
    return np.asarray(m.vector, dtype=np.float64).reshape(6)


class KinematicsPinTrac:
    """Pinocchio FK + Pinocchio 迭代 IK（类名保留以兼容现有 import）。"""

    def __init__(
        self,
        urdf_path: Path,
        left_chain: List[str],
        right_chain: List[str],
    ) -> None:
        self.available = False
        self.pin_ok = False
        self.load_error: str = ""
        self._urdf = str(Path(urdf_path).resolve())
        self._left_chain = list(left_chain)
        self._right_chain = list(right_chain)
        self._model = None
        self._data = None
        self._fid_left: int = -1
        self._fid_right: int = -1
        self._pin = None

        pin_mod = _load_pin_module()
        if pin_mod is None:
            self.load_error = "pinocchio 未加载（检查 NumPy 版本或安装 ros-*-pinocchio）"
            return

        try:
            self._model = pin_mod.buildModelFromUrdf(self._urdf)
            self._data = self._model.createData()
        except Exception as e:  # noqa: BLE001
            self.load_error = f"Pinocchio URDF 加载失败: {e}"
            return

        if not self._model.existFrame(LEFT_TIP_LINK) or not self._model.existFrame(
            RIGHT_TIP_LINK
        ):
            self.load_error = (
                f"Pinocchio URDF 缺少末端 FRAME（{LEFT_TIP_LINK} / {RIGHT_TIP_LINK}）"
            )
            return

        self._fid_left = int(self._model.getFrameId(LEFT_TIP_LINK))
        self._fid_right = int(self._model.getFrameId(RIGHT_TIP_LINK))
        self._pin = pin_mod
        self.pin_ok = True
        self.available = True
        self.load_error = ""
        _log.info(
            "[Kin] Pinocchio 正解 + 迭代 IK 已启用（左链 %d、右链 %d 关节；BASE=%s）",
            len(self._left_chain),
            len(self._right_chain),
            BASE_LINK,
        )

    def fk_tip_T(self, side: str, joint_positions_by_name: Dict[str, float]) -> Optional[np.ndarray]:
        """base_link 下腕部 4×4。"""
        if not self.available or not self.pin_ok or self._pin is None:
            return None
        try:
            q = _fill_pin_q(self._pin, self._model, joint_positions_by_name)
            self._pin.forwardKinematics(self._model, self._data, q)
            self._pin.updateFramePlacements(self._model, self._data)
            fid = self._fid_left if side == "left" else self._fid_right
            oMf = self._data.oMf[fid]
            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = np.array(oMf.rotation, dtype=np.float64)
            T[:3, 3] = np.array(oMf.translation, dtype=np.float64).reshape(3)
            return T
        except Exception as e:  # noqa: BLE001
            _log.debug("Pinocchio FK 失败: %s", e)
            return None

    def chain_dict_to_q_trac(self, side: str, chain_joint_q: Dict[str, float]) -> np.ndarray:
        """链关节顺序 → 长度 len(chain) 的角向量（与旧 tracikpy 链顺序一致）。"""
        chain = self._left_chain if side == "left" else self._right_chain
        q = np.zeros(len(chain), dtype=np.float64)
        for i, jn in enumerate(chain):
            v = chain_joint_q.get(jn)
            if v is None and not str(jn).endswith("_joint"):
                v = chain_joint_q.get(f"{jn}_joint")
            q[i] = float(v) if v is not None else 0.0
        return q

    def _chain_array_from_pin_q(self, q: np.ndarray, side: str) -> np.ndarray:
        chain = self._left_chain if side == "left" else self._right_chain
        out = np.zeros(len(chain), dtype=np.float64)
        for i, jn in enumerate(chain):
            idx = _idx_q_for_joint_name(self._model, jn)
            if idx is not None and 0 <= idx < len(q):
                out[i] = float(q[idx])
        return out

    def solve_ik(
        self,
        side: str,
        T_target: np.ndarray,
        chain_joint_q_seed: Dict[str, float],
        *,
        dq_joint_names: Optional[List[str]] = None,
        position_only: bool = False,
    ) -> Optional[np.ndarray]:
        """
        迭代阻尼 IK，返回链上关节角向量（长度 = len(chain)）。

        ``dq_joint_names``：仅对这些关节积分速度（如 7 个臂关节，腰等保持 seed）；
        为 ``None`` 时对整条链上的模型关节求列（与链名一一对应的可动列）。

        ``position_only``：为 True 时仅用 **世界系位置误差** 与线速度雅可比（前 3 行），
        用于直线示教（目标 ``T`` 只改平移、姿态由数值保持）；避免 6D ``log6`` 与 ``LOCAL``
        雅可比在「锁姿态」任务上组合不当导致 **dq≈0**。
        """
        if not self.available or not self.pin_ok or self._pin is None:
            return None

        pin = self._pin
        model = self._model
        data = self._data
        chain = self._left_chain if side == "left" else self._right_chain
        fid = self._fid_left if side == "left" else self._fid_right

        dq_names = list(dq_joint_names) if dq_joint_names is not None else list(chain)
        cols = _velocity_cols_for_joint_names(model, dq_names)
        if not cols:
            return None

        q = _fill_pin_q(pin, model, chain_joint_q_seed)
        R = np.asarray(T_target[:3, :3], dtype=np.float64)
        p = np.asarray(T_target[:3, 3], dtype=np.float64).reshape(3)
        oMdes = pin.SE3(R, p)

        max_iter = 400 if position_only else 200
        if position_only:
            # 未在 eps 内收敛时，旧版 fail_norm=2e-2（20mm）会接受过大残差，子步累积会短行程；
            # 过严（如 0.35mm）则难姿态易弹「IK 无有效步」。折中：fail_norm≈0.5mm。
            eps = 5.0e-5
            fail_norm = 5.0e-4
            damp = 0.012
            alpha = 0.82
            max_step = 0.32
        else:
            eps = 1.0e-4
            fail_norm = 8.0e-2
            damp = 1.0e-3
            alpha = 0.55
            max_step = 0.18

        rf_6d = pin.LOCAL
        rf_lin = getattr(pin, "LOCAL_WORLD_ALIGNED", pin.LOCAL)

        best_err = float("inf")
        best_q_chain: Optional[np.ndarray] = None

        for _ in range(max_iter):
            try:
                pin.forwardKinematics(model, data, q)
                pin.updateFramePlacements(model, data)
                oMf = data.oMf[fid]
                if position_only:
                    p_cur = np.asarray(oMf.translation, dtype=np.float64).reshape(3)
                    p_des = np.asarray(oMdes.translation, dtype=np.float64).reshape(3)
                    err_vec = p_des - p_cur
                    n = float(np.linalg.norm(err_vec))
                    if n < best_err:
                        best_err = n
                        best_q_chain = self._chain_array_from_pin_q(q, side)
                    if n < eps:
                        return self._chain_array_from_pin_q(q, side)
                    J6 = pin.computeFrameJacobian(model, data, q, fid, rf_lin)
                    Jred = np.asarray(J6[:3, cols], dtype=np.float64)
                    dq_red = _dls_pinv(Jred, damp) @ err_vec
                else:
                    iMd = oMf.actInv(oMdes)
                    ev = _log6_vec(pin, iMd)
                    n = float(np.linalg.norm(ev))
                    if n < best_err:
                        best_err = n
                        best_q_chain = self._chain_array_from_pin_q(q, side)
                    if n < eps:
                        return self._chain_array_from_pin_q(q, side)
                    J = pin.computeFrameJacobian(model, data, q, fid, rf_6d)
                    Jred = np.asarray(J[:, cols], dtype=np.float64)
                    dq_red = _dls_pinv(Jred, damp) @ (-ev)
                step = float(np.linalg.norm(dq_red))
                if step > max_step:
                    dq_red *= max_step / step
                v = np.zeros(model.nv, dtype=np.float64)
                for j, c in enumerate(cols):
                    v[c] = dq_red[j]
                q = pin.integrate(model, q, alpha * v)
            except Exception as e:  # noqa: BLE001
                _log.debug("Pinocchio IK 步失败: %s", e)
                break

        if best_q_chain is not None and best_err < fail_norm:
            return best_q_chain
        return None

    def q_chain_to_arm_dict(
        self,
        side: str,
        q_chain: np.ndarray,
        arm_joint_names: List[str],
    ) -> Dict[str, float]:
        chain = self._left_chain if side == "left" else self._right_chain
        out: Dict[str, float] = {}
        for jn in arm_joint_names:
            try:
                idx = chain.index(jn)
            except ValueError:
                continue
            if 0 <= idx < len(q_chain):
                out[jn] = float(q_chain[idx])
        return out

    def fk_T_from_chain(self, side: str, chain_joint_q: Dict[str, float]) -> Optional[np.ndarray]:
        """与链字典一致的正解 4×4（Pinocchio）。"""
        return self.fk_tip_T(side, chain_joint_q)

    def fk_tip_xyz_trac(
        self, side: str, chain_joint_q: Dict[str, float]
    ) -> Optional[np.ndarray]:
        T = self.fk_tip_T(side, chain_joint_q)
        if T is None:
            return None
        return np.array(T[:3, 3], dtype=np.float64).copy()
