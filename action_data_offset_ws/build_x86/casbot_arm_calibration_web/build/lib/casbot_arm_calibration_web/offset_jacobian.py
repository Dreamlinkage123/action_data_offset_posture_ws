"""调用 action_data_offset 的雅可比轨迹变换；Web 端将副本写入 ``new_offset_data/``（见 web_node）。"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path
from typing import Optional, Sequence, Tuple


Offset6 = Tuple[float, float, float, float, float, float]


def format_offset_6(left: Sequence[float], right: Sequence[float]) -> Tuple[str, str]:
    """
    左右臂 6D 偏移量字符串化：``"x,y,z,rx,ry,rz"``。

    - ``x,y,z``：沿 base_link 的直线位移（单位由 ``--length-unit`` 决定，本包统一 mm）。
    - ``rx,ry,rz``：外旋 XYZ 欧拉角（单位由 ``--angle-unit`` 决定，本包统一 rad）。
      应由 ``R_cur·R_init^T`` 转换得到，与 ``arm_trajectory_transform_jacobian``
      默认的 ``--euler-seq XYZ`` 保持一致。
    """
    if len(left) != 6 or len(right) != 6:
        raise ValueError("format_offset_6: 左右偏移必须各 6 个分量 (x,y,z,rx,ry,rz)")
    lx, ly, lz, lrx, lry, lrz = (float(v) for v in left)
    rx, ry, rz, rrx, rry, rrz = (float(v) for v in right)
    left_s = f"{lx:.6f},{ly:.6f},{lz:.6f},{lrx:.6f},{lry:.6f},{lrz:.6f}"
    right_s = f"{rx:.6f},{ry:.6f},{rz:.6f},{rrx:.6f},{rry:.6f},{rrz:.6f}"
    return left_s, right_s


def run_arm_trajectory_transform_jacobian(
    urdf_path: str,
    data_in: str,
    data_out_primary: str,
    *,
    offset_left: str,
    offset_right: str,
    data_out_copy: Optional[str] = None,
    timeout_sec: float = 7200.0,
) -> Tuple[bool, str]:
    """
    通过 ``ros2 run`` 调用 ``arm_trajectory_transform_jacobian``（Pinocchio 后端，输入 URDF）；
    成功后可选复制到 ``data_out_copy``。``offset_*`` 为 ``format_offset_6`` 生成的 6D 字符串。
    """
    # Web UI 示教语义：
    # - 平移按世界轴直线移动；
    # - 旋转在末端原点处沿世界轴就地旋转（保持位置不变）。
    # 这与 arm_trajectory_transform_jacobian 的 --compose world_ee 精确对应。
    # 同时将最大 Jacobian 步数与收敛阈值显式传入，避免大角度旋转时因 --max-delta-norm
    # 截断造成每帧欠冲（以便最后一帧准确落在 ``初始 + 偏移`` 上）。
    #
    # 偏移串首数字若为负（如 -0.1,0,...），必须写成 --offset-left=-0.1,... 单 token；
    # 若拆成 ``--offset-left`` 与 ``-0.1,...`` 两个参数，argparse 会把后者当成新选项，
    # 报错 ``--offset-left: expected one argument``。
    cmd = [
        "ros2",
        "run",
        "action_data_offset",
        "arm_trajectory_transform_jacobian",
        "--",
        "--robot-model",
        urdf_path,
        "--data",
        data_in,
        "--output",
        data_out_primary,
        "--arm",
        "both",
        f"--offset-left={offset_left}",
        f"--offset-right={offset_right}",
        "--length-unit",
        "mm",
        "--angle-unit",
        "rad",
        "--compose",
        "world_ee",
        "--linear-steps",
        "20",
        "--pos-tol",
        "1e-4",
        "--rot-tol",
        "1e-3",
    ]
    env = os.environ.copy()
    proc = subprocess.run(
        cmd,
        env=env,
        capture_output=True,
        text=True,
        timeout=timeout_sec,
    )
    if proc.returncode != 0:
        err = (proc.stderr or "").strip() or (proc.stdout or "").strip() or f"exit {proc.returncode}"
        return False, err
    out_p = Path(data_out_primary)
    if not out_p.is_file():
        return False, f"输出未生成: {data_out_primary}"
    if data_out_copy:
        dest = Path(data_out_copy)
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(out_p, dest)
    return True, ""
