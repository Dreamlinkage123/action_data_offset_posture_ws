"""
Pinocchio 正运动学 + tracikpy（TRAC‑IK）逆运动学可选后端。

- 用于 Web 示教笛卡尔直线 / 姿态旋转：每步对目标位姿做一次 TRAC‑IK，替代数值雅可比 DLS。
- `/joint_states` 末端位姿可用 Pinocchio 帧正解（比自研 XML FK 快）。
- 依赖（需自行安装，见类文档）::
    - ``pinocchio``（ROS 常用 ``ros-humble-pinocchio``）；**NumPy 建议 1.26.x（<2）**，与 Humble 自带扩展 ABI 一致。
    - ``tracikpy``：https://github.com/mjd3/tracikpy （及 README 中的 libeigen3、kdl、nlopt、urdfdom 等）
- 若导入或 URDF 初始化失败，:attr:`KinematicsPinTrac.available` 为 False，节点回退到 :mod:`arm_fk`。
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

_log = logging.getLogger(__name__)

try:
    from tracikpy import TracIKSolver
except Exception as e:  # noqa: BLE001
    TracIKSolver = None  # type: ignore[assignment]
    _trac_import_err = str(e)
else:
    _trac_import_err = ""

from casbot_arm_calibration_web.arm_fk import BASE_LINK, LEFT_TIP_LINK, RIGHT_TIP_LINK


def _load_pin_module():
    """
    延迟导入 pinocchio。ROS Humble 自带扩展通常针对 NumPy 1.x（如 1.26.4）；**NumPy 2.x** 下导入可能报错或崩溃，
    故在检测到 major>=2 时默认跳过，仅用 tracikpy FK。NumPy 1.26.x 时会正常加载 Pinocchio。
    设置环境变量 ``CASBOT_FORCE_PINOCCHIO=1`` 可在 NumPy 2 上仍尝试加载（可能不稳定）。
    """
    import os

    if os.environ.get("CASBOT_FORCE_PINOCCHIO", "").strip() not in ("1", "true", "yes"):
        try:
            major = int(np.__version__.split(".")[0])
        except (TypeError, ValueError):
            major = 0
        if major >= 2:
            _log.info(
                "NumPy %s：跳过 Pinocchio 扩展（避免与 NumPy 2 ABI 不兼容）；FK 使用 tracikpy",
                np.__version__,
            )
            return None
    try:
        import pinocchio as pin_mod  # noqa: PLC0415

        return pin_mod
    except BaseException as e:  # noqa: BLE001
        _log.debug("pinocchio 导入失败: %s", e)
        return None


def _fill_pin_q(pin_mod, model, joint_positions_by_name: Dict[str, float]) -> np.ndarray:
    """将 JointState 风格名→角（rad）填入 Pinocchio 配置向量。"""
    q = pin_mod.neutral(model)
    for jn0, val in joint_positions_by_name.items():
        candidates = (
            jn0,
            jn0 if str(jn0).endswith("_joint") else f"{jn0}_joint",
            str(jn0).replace("_joint", "") + "_joint",
        )
        for cand in candidates:
            if not model.existJointName(cand):
                continue
            jid = model.getJointId(cand)
            jmodel = model.joints[jid]
            if jmodel.nq != 1:
                break
            q[jmodel.idx_q] = float(val)
            break
    return q


class KinematicsPinTrac:
    """Pinocchio FK + 左右臂各一个 TracIKSolver。"""

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
        self._ik_left = None
        self._ik_right = None
        self._pin = None

        if TracIKSolver is None:
            self.load_error = f"tracikpy: {_trac_import_err or '未安装'}"
            return

        try:
            self._ik_left = TracIKSolver(self._urdf, BASE_LINK, LEFT_TIP_LINK)
            self._ik_right = TracIKSolver(self._urdf, BASE_LINK, RIGHT_TIP_LINK)
        except Exception as e:  # noqa: BLE001
            self.load_error = f"TracIKSolver 构造失败: {e}"
            return

        nl = self._ik_left.number_of_joints
        nr = self._ik_right.number_of_joints
        if nl != len(self._left_chain):
            self.load_error = (
                f"左臂 TRAC‑IK 关节数 {nl} 与 FK 链长度 {len(self._left_chain)} 不一致"
            )
            return
        if nr != len(self._right_chain):
            self.load_error = (
                f"右臂 TRAC‑IK 关节数 {nr} 与 FK 链长度 {len(self._right_chain)} 不一致"
            )
            return

        self.available = True

        pin_mod = _load_pin_module()
        if pin_mod is not None:
            try:
                # 仅用运动学模型：buildModelsFromUrdf 会加载 collision/visual 的 STL，
                # 本包 URDF 引用 ../meshes/*.STL 且默认不随包分发 mesh，会报错。
                self._model = pin_mod.buildModelFromUrdf(self._urdf)
                self._data = self._model.createData()
                if self._model.existFrame(LEFT_TIP_LINK) and self._model.existFrame(
                    RIGHT_TIP_LINK
                ):
                    self._fid_left = int(self._model.getFrameId(LEFT_TIP_LINK))
                    self._fid_right = int(self._model.getFrameId(RIGHT_TIP_LINK))
                    self.pin_ok = True
                    self._pin = pin_mod
                else:
                    _log.warning(
                        "Pinocchio URDF 缺少末端 FRAME，仅用 tracikpy FK（%s / %s）",
                        LEFT_TIP_LINK,
                        RIGHT_TIP_LINK,
                    )
            except Exception as e:  # noqa: BLE001
                _log.warning(
                    "Pinocchio 不可用（%s），正解将使用 tracikpy.fk",
                    e,
                )

        if self.pin_ok:
            _log.info(
                "[Kin] Pinocchio 正解 + TRAC‑IK 逆解已启用（左链 %d、右链 %d 关节）",
                nl,
                nr,
            )
        else:
            _log.info(
                "[Kin] TRAC‑IK 逆解 + tracikpy 正解已启用（左链 %d、右链 %d 关节）",
                nl,
                nr,
            )

    def fk_tip_T(self, side: str, joint_positions_by_name: Dict[str, float]) -> Optional[np.ndarray]:
        """base_link 下腕部 4×4；优先 Pinocchio，否则 tracikpy.fk。"""
        if not self.available:
            return None
        if (
            self.pin_ok
            and self._pin is not None
            and self._model is not None
            and self._data is not None
        ):
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
                _log.debug("Pinocchio FK 失败，回退 tracikpy: %s", e)
        return self.fk_T_from_chain(side, joint_positions_by_name)

    def chain_dict_to_q_trac(
        self, side: str, chain_joint_q: Dict[str, float]
    ) -> np.ndarray:
        solver = self._ik_left if side == "left" else self._ik_right
        chain = self._left_chain if side == "left" else self._right_chain
        q = np.zeros(solver.number_of_joints, dtype=np.float64)
        for i, jn in enumerate(chain):
            v = chain_joint_q.get(jn)
            if v is None and not str(jn).endswith("_joint"):
                v = chain_joint_q.get(f"{jn}_joint")
            q[i] = float(v) if v is not None else 0.0
        return q

    def solve_ik(
        self,
        side: str,
        T_target: np.ndarray,
        chain_joint_q_seed: Dict[str, float],
    ) -> Optional[np.ndarray]:
        """返回链上关节角向量（长度 = number_of_joints）；无解返回 None。"""
        if not self.available:
            return None
        solver = self._ik_left if side == "left" else self._ik_right
        q0 = self.chain_dict_to_q_trac(side, chain_joint_q_seed)
        try:
            q_out = solver.ik(np.asarray(T_target, dtype=np.float64), qinit=q0)
            if q_out is None:
                return None
            q_arr = np.asarray(q_out, dtype=np.float64).ravel()
            if q_arr.size != solver.number_of_joints or not np.all(np.isfinite(q_arr)):
                return None
            return q_arr.copy()
        except Exception as e:  # noqa: BLE001
            _log.debug("TRAC‑IK 失败: %s", e)
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
        """tracikpy 正解 4×4（与 TRAC‑IK 链一致）。"""
        if not self.available:
            return None
        solver = self._ik_left if side == "left" else self._ik_right
        q = self.chain_dict_to_q_trac(side, chain_joint_q)
        try:
            return np.asarray(solver.fk(q), dtype=np.float64).copy()
        except Exception:  # noqa: BLE001
            return None

    def fk_tip_xyz_trac(
        self, side: str, chain_joint_q: Dict[str, float]
    ) -> Optional[np.ndarray]:
        """tracikpy 末端位置；Pinocchio 不可用时界面仍可用此路径。"""
        T = self.fk_T_from_chain(side, chain_joint_q)
        if T is None:
            return None
        return np.array(T[:3, 3], dtype=np.float64).copy()
