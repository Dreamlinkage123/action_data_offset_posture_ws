#!/usr/bin/env python3
"""
离线诊断：读 URDF + 一组当前关节角，跑一次 base_link 系下的直线 IK，打印：

- 链/7 关节名/限位
- 起始末端位置、目标末端位置、IK 最终末端位置
- 每关节累计 Δq（度）与贡献占比
- 检查是否撞限位

用法::

    python3 -m casbot_arm_calibration_web.diag_cartesian_ik \\
        --urdf /path/to/xxx.urdf \\
        --side left --axis x --dir + --step-mm 10

若不指定 ``--q``，则默认当前关节角全 0（即 URDF 零位姿）。
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import List

import numpy as np

from casbot_arm_calibration_web.arm_fk import UrdfArmFk
from casbot_arm_calibration_web.arm_cartesian_ik import (
    cartesian_linear_waypoints,
    parse_joint_limits,
)


def _fmt_deg(q_rad: List[float]) -> str:
    return ", ".join(f"{q * 180.0 / math.pi:+.3f}" for q in q_rad)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", required=True, help="URDF 路径")
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--axis", choices=("x", "y", "z"), default="x")
    parser.add_argument("--dir", dest="direction", choices=("+", "-"), default="+")
    parser.add_argument("--step-mm", type=float, default=10.0)
    parser.add_argument("--step-size-mm", type=float, default=2.0, help="IK 单步（mm）")
    parser.add_argument(
        "--q",
        default="",
        help="用逗号分隔的 7 个关节角（弧度），顺序按 URDF 链末端 7 关节",
    )
    args = parser.parse_args()

    fk = UrdfArmFk(Path(args.urdf))
    if not fk.load():
        print(f"[FATAL] URDF 加载失败: {fk.load_error}")
        raise SystemExit(1)

    chain = fk._left_chain if args.side == "left" else fk._right_chain
    arm_names = list(chain[-7:])
    print(f"[INFO] chain({len(chain)}): {chain}")
    print(f"[INFO] arm 7 joints: {arm_names}")

    limits = parse_joint_limits(Path(args.urdf), arm_names)
    for jn in arm_names:
        lo, hi = limits[jn]
        print(f"[INFO]   limit {jn}: [{lo:+.3f}, {hi:+.3f}] rad ({lo*180/math.pi:+.2f}°, {hi*180/math.pi:+.2f}°)")

    if args.q.strip():
        parts = [p.strip() for p in args.q.split(",") if p.strip()]
        if len(parts) != 7:
            print(f"[FATAL] --q 需要 7 个数，实际 {len(parts)}")
            raise SystemExit(2)
        current = [float(p) for p in parts]
    else:
        current = [0.0] * 7

    chain_q_base = {jn: 0.0 for jn in chain}
    for k, jn in enumerate(arm_names):
        chain_q_base[jn] = current[k]

    p_start = fk.fk_tip_xyz(chain, chain_q_base)
    print(f"[INFO] start q (deg): [{_fmt_deg(current)}]")
    print(f"[INFO] start ee (base_link, m): x={p_start[0]:+.4f} y={p_start[1]:+.4f} z={p_start[2]:+.4f}")

    sgn = 1.0 if args.direction == "+" else -1.0
    mag_01mm = args.step_mm * 10.0 * sgn
    dx = mag_01mm if args.axis == "x" else 0.0
    dy = mag_01mm if args.axis == "y" else 0.0
    dz = mag_01mm if args.axis == "z" else 0.0
    print(f"[INFO] request dx={dx/10:+.2f}mm dy={dy/10:+.2f}mm dz={dz/10:+.2f}mm")

    wps = cartesian_linear_waypoints(
        fk=fk,
        chain_base_to_tip=chain,
        arm_joint_names=arm_names,
        chain_joint_q_base=chain_q_base,
        current_arm_q_rad=current,
        dx_01mm=dx,
        dy_01mm=dy,
        dz_01mm=dz,
        joint_limits=limits,
        step_size_01mm=max(1.0, args.step_size_mm * 10.0),
        damping=0.005,
    )
    print(f"[INFO] waypoints generated: {len(wps)}")
    if not wps:
        print("[FATAL] IK 未产生任何路点（位移为 0？）")
        raise SystemExit(3)

    last = wps[-1]
    last_q = [float(last["joint_angles_rad"][jn]) for jn in arm_names]
    last_ee = last["ee_position_m"]
    dq_deg = [(last_q[k] - current[k]) * 180.0 / math.pi for k in range(7)]
    print(f"[INFO] final q (deg): [{_fmt_deg(last_q)}]")
    print(f"[INFO] dq (deg):      [{', '.join(f'{d:+.3f}' for d in dq_deg)}]")
    print(
        f"[INFO] final ee (m):  x={last_ee['x']:+.4f} y={last_ee['y']:+.4f} z={last_ee['z']:+.4f}"
    )
    ach = (
        (last_ee["x"] - float(p_start[0])) * 1000.0,
        (last_ee["y"] - float(p_start[1])) * 1000.0,
        (last_ee["z"] - float(p_start[2])) * 1000.0,
    )
    req = (dx / 10.0, dy / 10.0, dz / 10.0)
    err = (ach[0] - req[0], ach[1] - req[1], ach[2] - req[2])
    print(f"[INFO] achieved Δee (mm): ({ach[0]:+.2f}, {ach[1]:+.2f}, {ach[2]:+.2f})")
    print(f"[INFO] requested Δee (mm):({req[0]:+.2f}, {req[1]:+.2f}, {req[2]:+.2f})")
    print(f"[INFO] error (mm):        ({err[0]:+.2f}, {err[1]:+.2f}, {err[2]:+.2f})")

    clipped = []
    for k, jn in enumerate(arm_names):
        lo, hi = limits[jn]
        if abs(last_q[k] - lo) < 1e-6 or abs(last_q[k] - hi) < 1e-6:
            clipped.append(jn)
    if clipped:
        print(f"[WARN] joints saturated at limit: {clipped}")

    dq_abs = [abs(x) for x in dq_deg]
    order = sorted(range(7), key=lambda i: -dq_abs[i])
    print("[INFO] joint contribution (deg desc):")
    for i in order:
        print(f"         {arm_names[i]:40s} dq={dq_deg[i]:+.3f}")

    # 中间采样（1/4、1/2、3/4），看 ee 是否沿直线均匀变化
    for frac in (0.25, 0.5, 0.75):
        idx = max(0, min(len(wps) - 1, int(round(frac * (len(wps) - 1)))))
        ee = wps[idx]["ee_position_m"]
        print(
            f"[INFO] ee at {int(frac*100)}%: x={ee['x']:+.4f} y={ee['y']:+.4f} z={ee['z']:+.4f} m"
        )


if __name__ == "__main__":
    main()
