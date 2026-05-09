#!/usr/bin/env python3
"""
ROS2 节点：提供 CASBOT 手臂位姿标定 Web 界面。

- 上身调试：/motion/upper_body_debug (std_srvs/SetBool)
- 标定轨迹：从 resource 读取 CSV，以 100Hz 发布 /upper_body_debug/joint_cmd (UpperJointData)
"""

from __future__ import annotations

import logging
import math
import re
import shutil
import socket
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import SetBool

try:
    from action_msgs.msg import GoalStatus
except ImportError:
    GoalStatus = None  # type: ignore[misc, assignment]

try:
    from crb_ros_msg.action import ActionPlay as ActionPlayMsg
except ImportError:
    ActionPlayMsg = None  # type: ignore[misc, assignment]

try:
    from crb_ros_msg.msg import UpperJointData
except ImportError:
    UpperJointData = None  # type: ignore[misc, assignment]

try:
    from flask import Flask, jsonify, request, send_from_directory
except ImportError as e:
    raise ImportError(
        "请安装 Flask: pip install flask 或在 package.xml 中声明依赖后 colcon build"
    ) from e

from casbot_arm_calibration_web.arm_fk import UrdfArmFk, build_q_map_for_chain, rotation_matrix_to_rpy, rpy_to_R
from casbot_arm_calibration_web.arm_cartesian_ik import (
    _rot_unit_axis,
    _vee_log_so3,
    cartesian_linear_waypoints,
    cartesian_linear_waypoints_trac,
    cartesian_rotate_waypoints,
    cartesian_rotate_waypoints_trac,
    parse_joint_limits,
)
from casbot_arm_calibration_web.kin_pin_trac import KinematicsPinTrac
from casbot_arm_calibration_web.calibration_data import JOINT_NAMES_PUBLISH, JOINTS_WITH_SUFFIX, load_calibration_trajectory
from casbot_arm_calibration_web.offset_jacobian import format_offset_6, run_arm_trajectory_transform_jacobian
from casbot_arm_calibration_web.trajectory_sources import (
    calibration_web_package_root,
    find_resource_biaoding_file,
    list_action_data_basenames,
    list_offset_data_relative_paths,
    list_resource_biaoding_files,
    new_offset_data_write_targets,
    offset_data_root,
    resolve_trajectory_path,
)

# 前端 value -> resource 子目录名（resource/<subdir>/BiaoDingA_*.data、BiaoDingB_*.data）
INSTRUMENT_TO_SUBDIR = {
    "drum": "drum",
    "bass": "bass",
    "guitar": "guitar",
    "keyboard": "keyboard",
}

# JOINTS_WITH_SUFFIX 中左/右臂 7 关节下标（头、腰之后，灵巧手之前）
_LEFT_ARM_JOINT_INDICES = slice(3, 10)
_RIGHT_ARM_JOINT_INDICES = slice(10, 17)
# 单臂示教：这些关节不用 /joint_states 覆盖 base_full（与对侧臂一同保持 hold），与标定末帧后一致，减轻只动一侧时另一侧腕 mm 级视在偏
_JOG_SYNC_SKIP_FROM_JOINT_STATES_NAMES = frozenset(
    ("head_yaw_joint", "head_pitch_joint", "waist_yaw_joint")
)


def _pin_seed_joint_dict(base_full: List[float], chain_q_base: Dict[str, float]) -> Dict[str, float]:
    """
    Pinocchio 全模型 FK/IK 用的关节名→弧度表。

    用 ``UpperJointData`` 全身向量 ``base_full`` 与链上基线 ``chain_q_base`` 合并，后者覆盖前者
    （如腰用 ``_apply_live_waist_to_chain_q_base`` 对齐 /joint_states）。避免仅传 8 个链关节时
    右臂/头等在 Pin 里落 neutral、与真机不符，导致「只示教 Y」却在 X/Z 或对侧末端偏移乱跳。
    """
    out: Dict[str, float] = {}
    for i, jn in enumerate(JOINTS_WITH_SUFFIX):
        if i < len(base_full):
            out[jn] = float(base_full[i])
    out.update(chain_q_base)
    return out


# 与常见 ``ros2 topic pub``（多为 Best Effort）对齐，便于订阅外部部分关节命令
_JOINT_CMD_SUB_QOS = QoSProfile(
    depth=200,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def _web_root() -> Path:
    """静态资源目录：优先与 web_node 同目录的 web/，否则 share。"""
    local = Path(__file__).resolve().parent / "web"
    if local.is_dir() and (local / "index.html").is_file():
        return local
    try:
        from ament_index_python.packages import get_package_share_directory

        alt = Path(get_package_share_directory("casbot_arm_calibration_web")) / "web"
        if alt.is_dir():
            return alt
    except Exception:
        pass
    return local


def _resource_root() -> Path:
    """安装后的 resource/，或源码树中的 resource/。"""
    try:
        from ament_index_python.packages import get_package_share_directory

        p = Path(get_package_share_directory("casbot_arm_calibration_web")) / "resource"
        if p.is_dir():
            return p
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent / "resource"


def _package_urdf_dir() -> Path:
    """
    机器人模型目录：优先使用 colcon 安装后的路径
    ``<install>/share/casbot_arm_calibration_web/urdf``（与 setup.py 安装的 urdf 一致）。
    若无法解析 ament 索引（例如未 source 安装空间），则退回源码包内 ``.../casbot_arm_calibration_web/urdf``。
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("casbot_arm_calibration_web")) / "urdf"
    except Exception:
        pass
    return Path(__file__).resolve().parent.parent / "urdf"


# 参数 robot_urdf_path 为空时：在 _package_urdf_dir() 下使用该 .urdf 文件名（安装后在 share/.../urdf/）
_DEFAULT_ROBOT_URDF_NAME = "CASBOT02_ENCOS_7dof_shell_20251015_P1L.urdf"


def _resolve_robot_urdf_path(param: str) -> Path:
    """
    - 空字符串：``_package_urdf_dir() / _DEFAULT_ROBOT_URDF_NAME``（通常为 share 下 urdf 目录）
    - 绝对路径：直接使用
    - 其它相对路径：相对于 _package_urdf_dir()（例如只写文件名）
    """
    raw = (param or "").strip()
    urdf_dir = _package_urdf_dir()
    if not raw:
        return urdf_dir / _DEFAULT_ROBOT_URDF_NAME
    p = Path(raw)
    if p.is_absolute():
        return p
    return urdf_dir / p


def _guess_primary_ipv4() -> Optional[str]:
    """默认路由上的本机 IPv4（无外网时可能失败；用于日志提示局域网 URL）。"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(0.25)
            s.connect(("8.8.8.8", 80))
            return str(s.getsockname()[0])
    except OSError:
        return None


def _log_calibration_web_urls(logger, host: str, port: int, resource_root: Path) -> None:
    """启动时打印 x86 本机与 Orin/局域网典型访问方式，便于 docker 与现场部署。"""
    logger.info(
        f"[Web] CASBOT 手臂位姿标定 — HTTP 监听 {host}:{port}，resource: {resource_root}"
    )
    listen_all = host in ("0.0.0.0", "")
    if listen_all:
        logger.info(f"[Web] x86 / Docker 本机浏览器: http://127.0.0.1:{port}/")
        logger.info(f"[Web] x86 / Docker（与 launch 默认 host=0.0.0.0 一致）: http://0.0.0.0:{port}/")
    else:
        logger.info(f"[Web] 浏览器访问: http://{host}:{port}/")

    lan = _guess_primary_ipv4()
    if lan and not lan.startswith("127."):
        logger.info(
            f"[Web] 局域网其它电脑浏览器（自动探测本机 IP）: http://{lan}:{port}/ "
            f"（Orin 部署示例 http://172.16.0.10:{port}/，以实际 ifconfig/ip 为准）"
        )
    else:
        logger.info(
            f"[Web] 局域网其它电脑浏览器: http://<本机IP>:{port}/ "
            f"（Orin 示例 http://172.16.0.10:{port}/）"
        )


def create_app(node: "ArmCalibrationWebNode") -> Flask:
    app = Flask(__name__)
    # 关闭 Werkzeug 对每个请求的 INFO 日志（前端轮询 /api/state 会刷屏）
    for _name in ("werkzeug", "werkzeug.serving"):
        logging.getLogger(_name).setLevel(logging.ERROR)
    _api_lock = threading.Lock()

    @app.route("/")
    def index():
        return send_from_directory(node._web_root, "index.html")

    @app.route("/style.css")
    def style():
        return send_from_directory(node._web_root, "style.css")

    @app.route("/app.js")
    def script():
        return send_from_directory(node._web_root, "app.js")

    @app.get("/api/state")
    def api_get_state():
        return jsonify({"ok": True, "data": node.get_ui_state()})

    @app.post("/api/arm/<side>/axis/<axis>/adjust")
    def api_adjust(side: str, axis: str):
        body = request.get_json(silent=True) or {}
        direction = body.get("direction")
        try:
            step_mm = float(body.get("step", 1.0))
        except (TypeError, ValueError):
            return jsonify({"ok": False, "message": "step 必须为数值"}), 400
        node.get_logger().info(
            f"[Web] 笛卡尔示教: side={side!r} axis={axis!r} dir={direction!r} step={step_mm} mm"
        )
        with _api_lock:
            with node._traj_lock:
                if node._playing:
                    return jsonify({"ok": False, "message": "有轨迹正在播放，请等待播完后再点击"}), 409
            ok, message, extra = node.adjust_arm_cartesian(side, axis, direction, step_mm)
        if not ok:
            return jsonify({"ok": False, "message": message}), 400
        payload: Dict[str, Any] = {"ok": True, "message": message}
        if extra:
            payload.update(extra)
        return jsonify(payload)

    @app.post("/api/arm/<side>/rotate/<axis>")
    def api_rotate(side: str, axis: str):
        body = request.get_json(silent=True) or {}
        direction = body.get("direction")
        try:
            step_deg = float(body.get("step", 0.3))
        except (TypeError, ValueError):
            return jsonify({"ok": False, "message": "step 必须为数值（度）"}), 400
        node.get_logger().info(
            f"[Web] 姿态示教: side={side!r} axis={axis!r} dir={direction!r} step={step_deg}°"
        )
        with _api_lock:
            with node._traj_lock:
                if node._playing:
                    return jsonify({"ok": False, "message": "有轨迹正在播放，请等待播完后再点击"}), 409
            ok, message, extra = node.adjust_arm_cartesian_rotate(side, axis, direction, step_deg)
        if not ok:
            return jsonify({"ok": False, "message": message}), 400
        payload: Dict[str, Any] = {"ok": True, "message": message}
        if extra:
            payload.update(extra)
        return jsonify(payload)

    @app.post("/api/arm/<side>/initial/refresh")
    def api_refresh_initial(side: str):
        """把当前界面左右腕末端 FK 位姿写入「初始值」（任一面板触发均更新双臂，与末帧标定快照一致）。"""
        side_l = str(side).strip().lower()
        if side_l not in ("left", "right"):
            return jsonify({"ok": False, "message": "side 应为 left 或 right"}), 400
        ok, message, extra = node.refresh_initial_ee(side_l)
        if not ok:
            return jsonify({"ok": False, "message": message}), 400
        payload: Dict[str, Any] = {"ok": True, "message": message}
        if extra:
            payload.update(extra)
        return jsonify(payload)

    @app.get("/api/trajectory/action_data_files")
    def api_action_data_files():
        return jsonify({"ok": True, "files": list_action_data_basenames()})

    @app.get("/api/trajectory/offset_data_files")
    def api_offset_data_files():
        return jsonify({"ok": True, "files": list_offset_data_relative_paths()})

    @app.get("/api/trajectory/recent_generated_offset_files")
    def api_recent_generated_offset_files():
        """本次 Web 进程内「生成偏移数据」成功的条目（与下拉 ``offset_data_relative`` 一致），便于立即选用。"""
        return jsonify({"ok": True, "files": node.list_recent_generated_offset_rels()})

    @app.post("/api/calibration/play_selected")
    def api_calibration_play_selected():
        body = request.get_json(silent=True) or {}
        node.get_logger().info(f"[Web] 播放所选轨迹: body={body!r}")
        mode = str(body.get("trajectory_mode", "")).strip().lower()
        instruments = body.get("instruments") or []
        with _api_lock:
            with node._traj_lock:
                if node._playing:
                    return jsonify(
                        {"ok": False, "message": "标定轨迹正在播放中，请等待完成后再试"}
                    ), 409
            if mode.startswith("resource"):
                if not instruments:
                    return jsonify({"ok": False, "message": "请至少选择一种乐器"}), 400
                kind = "start" if mode.endswith("start") else "end"
                rows = node._load_calibration_rows(instruments, kind)
                if not rows:
                    return jsonify(
                        {"ok": False, "message": "未加载到任何轨迹数据（检查 resource 下对应 .data）"}
                    ), 400
            else:
                path_in, err = resolve_trajectory_path(
                    mode=mode,
                    resource_root=node._resource_root,
                    instruments=instruments,
                    action_data_basename=str(body.get("action_data_basename", "")),
                    offset_data_relative=str(body.get("offset_data_relative", "")),
                    custom_path=str(body.get("custom_path", "")),
                )
                if path_in is None:
                    return jsonify({"ok": False, "message": err or "无法解析轨迹路径"}), 400
                try:
                    rows = load_calibration_trajectory(path_in)
                except Exception as ex:
                    return jsonify({"ok": False, "message": f"读取失败: {ex}"}), 400
                if not rows:
                    return jsonify({"ok": False, "message": "轨迹为空"}), 400
            with node._traj_lock:
                node._traj_rows = rows
                node._traj_index = 0
                node._playing = True
                node._traj_mode = "start" if mode.startswith("resource") else "custom_play"
            return jsonify({"ok": True, "frames": len(rows)})

    @app.post("/api/save")
    def api_save():
        """保存当前左右臂末端 6D 偏移量（位置 mm + 姿态 rad，XYZ 外旋）至 JSON。

        仅持久化偏移量，不再立即生成轨迹；生成轨迹由 /api/offset_data_produce 触发。
        """
        body = request.get_json(silent=True) or {}
        node.get_logger().info(f"[Web] 保存 6D 偏移量: body={body!r}")
        with _api_lock:
            with node._traj_lock:
                if node._playing:
                    return jsonify({"ok": False, "message": "轨迹播放中，请结束后再保存"}), 409
            left_6, right_6, oerr = node.get_pose_offsets_6()
            if oerr:
                return jsonify({"ok": False, "message": oerr}), 400
            try:
                saved_path = node.persist_saved_offsets(left_6, right_6)
            except Exception as ex:
                return jsonify({"ok": False, "message": f"写入 web_saved_offsets.json 失败: {ex}"}), 500
        return jsonify(
            {
                "ok": True,
                "message": "已保存左右臂末端 6D 偏移量（x,y,z[mm] + rx,ry,rz[rad] 外旋 XYZ）",
                "saved_path": str(saved_path),
                "left": {
                    "x": left_6[0], "y": left_6[1], "z": left_6[2],
                    "rx": left_6[3], "ry": left_6[4], "rz": left_6[5],
                },
                "right": {
                    "x": right_6[0], "y": right_6[1], "z": right_6[2],
                    "rx": right_6[3], "ry": right_6[4], "rz": right_6[5],
                },
            }
        )

    def _plan_offset_output(body: Dict[str, Any]) -> Tuple[
        Optional[Dict[str, Any]], Optional[Tuple[str, int]]
    ]:
        """解析「生成偏移数据」的输出路径（不执行生成）。

        成功返回 ``(plan, None)``，失败返回 ``(None, (message, http_status))``。
        ``plan`` 包含 ``path_in`` / ``out_base`` / ``primary_out`` / ``off_copy`` /
        ``output_copies_planned`` / ``offset_data_relative`` / ``write_dirs``。
        """
        mode = str(body.get("trajectory_mode", "resource_start")).strip().lower()
        instruments = body.get("instruments") or []
        path_in, err = resolve_trajectory_path(
            mode=mode,
            resource_root=node._resource_root,
            instruments=instruments,
            action_data_basename=str(body.get("action_data_basename", "")),
            offset_data_relative=str(body.get("offset_data_relative", "")),
            custom_path=str(body.get("custom_path", "")),
        )
        if path_in is None:
            return None, (err or "无法解析输入轨迹", 400)
        stem = path_in.stem
        if stem.endswith("_web_offset"):
            stem = stem[: -len("_web_offset")]
        stem = re.sub(r"_web_offset_\d{8}_\d{6}(?:_\d{3})?$", "", stem)
        out_base = str(body.get("output_basename", "")).strip()
        if not out_base:
            from datetime import datetime  # noqa: PLC0415

            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            out_base = f"{stem}_web_offset_{ts}.data"
        if "/" in out_base or "\\" in out_base:
            return None, ("output_basename 请只填文件名，不要路径", 400)
        try:
            from action_data_paths import default_output_dir as ado_default_output  # noqa: PLC0415
        except Exception as ex:
            return None, (f"需要 action_data_offset 包（action_data_paths）: {ex}", 500)
        primary_out = str((ado_default_output() / out_base).resolve())
        write_dirs = new_offset_data_write_targets()
        off_copy = str((write_dirs[0] / Path(out_base).name).resolve())
        output_copies_planned = [off_copy] + [
            str((td / Path(out_base).name).resolve()) for td in write_dirs[1:]
        ]
        return (
            {
                "path_in": path_in,
                "out_base": out_base,
                "primary_out": primary_out,
                "off_copy": off_copy,
                "output_copies_planned": output_copies_planned,
                "offset_data_relative": f"new_offset_data/{Path(out_base).name}",
                "write_dirs": write_dirs,
            },
            None,
        )

    @app.post("/api/offset_data_produce/preview")
    def api_offset_data_produce_preview():
        """返回「生成偏移数据」即将写入的文件名与完整路径（**不**执行生成）。

        前端在确认弹窗中展示该路径；确认后再以同名 ``output_basename`` 调用
        ``/api/offset_data_produce``，保证显示与落地一致。
        """
        body = request.get_json(silent=True) or {}
        node.get_logger().info(f"[Web] 生成偏移数据(预览): body={body!r}")
        plan, perr = _plan_offset_output(body)
        if plan is None:
            assert perr is not None
            return jsonify({"ok": False, "message": perr[0]}), perr[1]
        return jsonify(
            {
                "ok": True,
                "output_basename": plan["out_base"],
                "output_primary": plan["primary_out"],
                "output_copy": plan["off_copy"],
                "output_copies": plan["output_copies_planned"],
                "offset_data_relative": plan["offset_data_relative"],
            }
        )

    @app.post("/api/offset_data_produce")
    def api_offset_data_produce():
        """
        基于「保存」的 6D 偏移量（若无则使用当前实时 6D 偏移量），调用
        ``arm_trajectory_transform_jacobian`` 生成新轨迹文件（``<stem>_web_offset.data``）。
        输出保存到 ``action_data_offset/output`` 与本包 ``new_offset_data/``（默认文件名带时间戳，避免与旧 _web_offset 混淆）。
        """
        body = request.get_json(silent=True) or {}
        node.get_logger().info(f"[Web] 生成偏移数据: body={body!r}")
        with _api_lock:
            with node._traj_lock:
                if node._playing:
                    return jsonify({"ok": False, "message": "轨迹播放中，请结束后再生成"}), 409
            saved = node.load_saved_offsets()
            source_hint: str
            if saved is not None:
                left_6, right_6 = saved
                source_hint = "saved"
            else:
                left_6, right_6, oerr = node.get_pose_offsets_6()
                if oerr:
                    return jsonify(
                        {"ok": False, "message": f"未找到已保存偏移，且实时偏移不可用：{oerr}"}
                    ), 400
                source_hint = "live"
        node.get_logger().info(
            f"[offset_data_produce] 偏移来源={source_hint}, JSON路径={node.saved_offsets_path()}"
        )
        plan, perr = _plan_offset_output(body)
        if plan is None:
            assert perr is not None
            return jsonify({"ok": False, "message": perr[0]}), perr[1]
        path_in = plan["path_in"]
        out_base = plan["out_base"]
        primary_out = plan["primary_out"]
        off_copy = plan["off_copy"]
        write_dirs = plan["write_dirs"]
        node.get_logger().info(
            f"[offset_data_produce] new_offset_data 写入目录数={len(write_dirs)}: "
            + ", ".join(str(p) for p in write_dirs)
        )
        ol, ort = format_offset_6(left_6, right_6)
        try:
            urdf = node.resolve_urdf_path_for_transform()
        except Exception as ex:
            return jsonify({"ok": False, "message": f"URDF 路径解析失败: {ex}"}), 500
        ok, msg = run_arm_trajectory_transform_jacobian(
            urdf,
            str(path_in.resolve()),
            primary_out,
            offset_left=ol,
            offset_right=ort,
            data_out_copy=off_copy,
        )
        if not ok:
            node.get_logger().error(f"arm_trajectory_transform_jacobian 失败: {msg}")
            return jsonify({"ok": False, "message": msg[:2000]}), 500
        gen_p = Path(primary_out).resolve()
        output_copies: List[str] = [off_copy]
        if gen_p.is_file():
            for tdir in write_dirs[1:]:
                dest = (tdir / Path(out_base).name).resolve()
                try:
                    shutil.copy2(gen_p, dest)
                    output_copies.append(str(dest))
                except Exception as ex:
                    node.get_logger().warn(f"复制 new_offset 到 {dest} 失败: {ex}")
        rel_select = f"new_offset_data/{Path(out_base).name}"
        node.record_generated_offset_rel(rel_select)
        return jsonify(
            {
                "ok": True,
                "message": (
                    "已生成偏移轨迹：主副本优先写在可解析的 "
                    "src/casbot_arm_calibration_web/new_offset_data/，并复制到 install/share/.../new_offset_data/。"
                    "数据来源已切换为「本包 offest_data / new_offset_data」，并选中本次生成项。"
                ),
                "output_primary": primary_out,
                "output_copy": off_copy,
                "output_copies": output_copies,
                "offset_left": ol,
                "offset_right": ort,
                "offset_source": source_hint,
                "offset_data_relative": rel_select,
            }
        )

    def _wait_service_future(fut, timeout_sec: float = 5.0) -> bool:
        """等待异步服务结果（由主线程 Executor 处理回调）。"""
        t0 = time.time()
        while rclpy.ok() and not fut.done() and (time.time() - t0) < timeout_sec:
            time.sleep(0.01)
        return fut.done()

    @app.get("/api/calibration/status")
    def api_calibration_status():
        with node._traj_lock:
            playing = node._playing
            total = len(node._traj_rows)
            idx = node._traj_index
        return jsonify(
            {
                "ok": True,
                "playing": playing,
                "frames_total": total,
                "frames_sent": idx,
            }
        )

    # ---- 乐队多片段数据偏移生成播放 ---------------------------------------------
    #
    # 「标定数据偏移生成播放」针对开始/结束标定单文件；本段支持：
    #   - 指定来源目录，列出 .data / .csv 多个动作数据；
    #   - 基于已保存（或实时）的 6D 偏移，批量生成偏移文件到用户指定目录；
    #   - 按用户选择的顺序把多个（已生成或其它）偏移文件串接播放。
    # 前端负责"按点击顺序"维护播放次序；后端仅校验路径并拼接 rows。

    def _resolve_user_dir(
        raw: str, *, must_exist: bool = True, create: bool = False
    ) -> Tuple[Optional[Path], str]:
        """把用户输入的路径解析成安全的绝对 :class:`Path`。

        - 空字符串：失败（调用方可自己再降级到默认目录）。
        - 必须为绝对路径（避免相对路径随工作目录漂移）。
        - ``must_exist``/``create`` 控制是否要求已存在或自动创建。
        """
        r = (raw or "").strip()
        if not r:
            return None, "路径为空"
        p = Path(r).expanduser()
        if not p.is_absolute():
            return None, f"需要绝对路径: {r}"
        if create:
            try:
                p.mkdir(parents=True, exist_ok=True)
            except Exception as ex:
                return None, f"无法创建目录 {p}: {ex}"
        if must_exist and not p.is_dir():
            return None, f"目录不存在: {p}"
        return p.resolve(), ""

    def _list_data_files_in(d: Path) -> List[str]:
        """列出目录内 ``.data`` / ``.csv`` 文件名（仅当前层；按字典序；不递归）。"""
        try:
            return sorted(
                f.name
                for f in d.iterdir()
                if f.is_file() and f.suffix.lower() in (".data", ".csv")
            )
        except Exception:
            return []

    def _list_csv_files_in(d: Path) -> List[str]:
        """列出目录内 ``.csv`` 文件名（仅当前层；按字典序；不递归）。"""
        try:
            return sorted(
                f.name
                for f in d.iterdir()
                if f.is_file() and f.suffix.lower() == ".csv"
            )
        except Exception:
            return []

    @app.post("/api/band/list_dir")
    def api_band_list_dir():
        """列出目录下 ``.data``/``.csv`` 文件（顶层，不递归）。前端用于"来源/保存"两个目录。"""
        body = request.get_json(silent=True) or {}
        p, err = _resolve_user_dir(str(body.get("dir", "")), must_exist=True)
        if p is None:
            return jsonify({"ok": False, "message": err}), 400
        files = _list_data_files_in(p)
        return jsonify({"ok": True, "dir": str(p), "files": files})

    @app.post("/api/band/produce")
    def api_band_produce():
        """对 ``source_dir`` 下选中的一批文件批量生成偏移轨迹到 ``save_dir``。

        输出文件与来源**同名**（含扩展名），写入 ``save_dir``；同一文件再次生成将覆盖。
        偏移量来源与单文件版一致：优先 ``web_saved_offsets.json``，否则实时 6D 偏移。
        ``save_dir`` 为空时退回 ``new_offset_data_write_targets()[0]``。
        返回每个文件的成功/失败与输出绝对路径，不做部分回滚。
        """
        body = request.get_json(silent=True) or {}
        node.get_logger().info(f"[Web][band] 批量生成: body={body!r}")
        names = body.get("filenames") or []
        if not isinstance(names, list) or not names:
            return jsonify({"ok": False, "message": "filenames 为空"}), 400
        src_p, err = _resolve_user_dir(str(body.get("source_dir", "")), must_exist=True)
        if src_p is None:
            return jsonify({"ok": False, "message": f"source_dir 无效: {err}"}), 400
        raw_save = str(body.get("save_dir", "")).strip()
        if raw_save:
            save_p, err = _resolve_user_dir(raw_save, must_exist=False, create=True)
            if save_p is None:
                return jsonify({"ok": False, "message": f"save_dir 无效: {err}"}), 400
        else:
            targets = new_offset_data_write_targets()
            if not targets:
                return jsonify({"ok": False, "message": "未解析到默认 new_offset_data 目录"}), 500
            save_p = targets[0]
        # 仅在进入长耗时路径前校验一次播放状态
        with _api_lock:
            with node._band_csv_state_lock:
                if node._band_csv_playing:
                    return jsonify(
                        {"ok": False, "message": "CSV Action 顺序播放中，请结束后再生成"}
                    ), 409
            with node._traj_lock:
                if node._playing:
                    return jsonify({"ok": False, "message": "轨迹播放中，请结束后再生成"}), 409
            saved = node.load_saved_offsets()
            if saved is not None:
                left_6, right_6 = saved
                source_hint = "saved"
            else:
                left_6, right_6, oerr = node.get_pose_offsets_6()
                if oerr:
                    return jsonify(
                        {"ok": False, "message": f"未找到已保存偏移，且实时偏移不可用：{oerr}"}
                    ), 400
                source_hint = "live"
        ol, ort = format_offset_6(left_6, right_6)
        try:
            urdf = node.resolve_urdf_path_for_transform()
        except Exception as ex:
            return jsonify({"ok": False, "message": f"URDF 路径解析失败: {ex}"}), 500

        results: List[Dict[str, Any]] = []
        all_ok = True

        for raw_name in names:
            name = str(raw_name or "").strip()
            if not name or "/" in name or "\\" in name or name.startswith("."):
                results.append({"input": name, "ok": False, "message": "非法文件名"})
                all_ok = False
                continue
            in_path = (src_p / name).resolve()
            try:
                in_path.relative_to(src_p)
            except ValueError:
                results.append({"input": name, "ok": False, "message": "越界路径"})
                all_ok = False
                continue
            if not in_path.is_file():
                results.append({"input": name, "ok": False, "message": f"文件不存在: {in_path}"})
                all_ok = False
                continue
            # 与来源文件名（含扩展名）一致，便于 data/ 与 cali_data/ 一一对应；重复生成则覆盖。
            out_name = name
            out_path = (save_p / out_name).resolve()
            try:
                out_path.relative_to(save_p)
            except ValueError:
                results.append({"input": name, "ok": False, "message": "输出越界"})
                all_ok = False
                continue
            ok, msg = run_arm_trajectory_transform_jacobian(
                urdf,
                str(in_path),
                str(out_path),
                offset_left=ol,
                offset_right=ort,
            )
            if not ok:
                node.get_logger().error(f"[band] {name} 生成失败: {msg}")
                results.append(
                    {"input": name, "ok": False, "message": (msg or "未知错误")[:2000]}
                )
                all_ok = False
                continue
            results.append(
                {
                    "input": name,
                    "input_path": str(in_path),
                    "output_path": str(out_path),
                    "output_name": out_name,
                    "ok": True,
                }
            )
            # 记录到"本次进程内生成"，便于单文件下拉里也能看到
            try:
                out_rel = out_path.relative_to(save_p).as_posix()
                # 仅当保存目录恰好是扫描列表中的 new_offset_data 根时，才注入单文件下拉
                for nroot in new_offset_data_write_targets():
                    if save_p == nroot:
                        node.record_generated_offset_rel(f"new_offset_data/{out_rel}")
                        break
            except Exception:
                pass
        return jsonify(
            {
                "ok": all_ok,
                "message": ("批量生成完成" if all_ok else "批量生成部分失败，详见 results"),
                "offset_source": source_hint,
                "save_dir": str(save_p),
                "offset_left": ol,
                "offset_right": ort,
                "results": results,
            }
        )

    @app.post("/api/band/play_sequence")
    def api_band_play_sequence():
        """按 ``files`` 给定的绝对路径顺序，串接为一条轨迹从头播放。

        前端应已先调用 ``/api/upper_body_debug`` 开启调试（与"播放所选轨迹"一致）。
        任何一个文件读取失败则整体失败，不会进入部分播放。
        """
        body = request.get_json(silent=True) or {}
        node.get_logger().info(f"[Web][band] 顺序播放: body={body!r}")
        files = body.get("files") or []
        if not isinstance(files, list) or not files:
            return jsonify({"ok": False, "message": "files 为空"}), 400
        abs_list: List[Path] = []
        for raw in files:
            p = Path(str(raw or "").strip()).expanduser()
            if not p.is_absolute():
                return jsonify({"ok": False, "message": f"需要绝对路径: {raw}"}), 400
            if not p.is_file():
                return jsonify({"ok": False, "message": f"文件不存在: {p}"}), 400
            abs_list.append(p.resolve())
        with _api_lock:
            with node._band_csv_state_lock:
                if node._band_csv_playing:
                    return jsonify(
                        {"ok": False, "message": "CSV Action 顺序播放中，请等待完成后再试"}
                    ), 409
            with node._traj_lock:
                if node._playing:
                    return jsonify(
                        {"ok": False, "message": "标定轨迹正在播放中，请等待完成后再试"}
                    ), 409
            concat_rows: List[List[float]] = []
            per_file_frames: List[Dict[str, Any]] = []
            for p in abs_list:
                try:
                    rows = load_calibration_trajectory(p)
                except Exception as ex:
                    return jsonify({"ok": False, "message": f"读取失败 {p}: {ex}"}), 400
                if not rows:
                    return jsonify({"ok": False, "message": f"轨迹为空: {p}"}), 400
                per_file_frames.append({"path": str(p), "frames": len(rows)})
                concat_rows.extend(rows)
            with node._traj_lock:
                node._traj_rows = concat_rows
                node._traj_index = 0
                node._playing = True
                node._traj_mode = "band_play"
            return jsonify(
                {
                    "ok": True,
                    "frames": len(concat_rows),
                    "segments": per_file_frames,
                }
            )

    @app.post("/api/band_csv/list_dir")
    def api_band_csv_list_dir():
        """列出目录下 ``.csv`` 文件（顶层，不递归）；用于 CSV 乐队面板的来源/保存目录。"""
        body = request.get_json(silent=True) or {}
        p, err = _resolve_user_dir(str(body.get("dir", "")), must_exist=True)
        if p is None:
            return jsonify({"ok": False, "message": err}), 400
        files = _list_csv_files_in(p)
        return jsonify({"ok": True, "dir": str(p), "files": files})

    @app.post("/api/band_csv/produce")
    def api_band_csv_produce():
        """与 ``/api/band/produce`` 相同流程，但仅接受 ``.csv`` 输入/输出（逐文件同名写入 save_dir）。"""
        body = request.get_json(silent=True) or {}
        node.get_logger().info(f"[Web][band_csv] 批量生成: body={body!r}")
        names = body.get("filenames") or []
        if not isinstance(names, list) or not names:
            return jsonify({"ok": False, "message": "filenames 为空"}), 400
        src_p, err = _resolve_user_dir(str(body.get("source_dir", "")), must_exist=True)
        if src_p is None:
            return jsonify({"ok": False, "message": f"source_dir 无效: {err}"}), 400
        raw_save = str(body.get("save_dir", "")).strip()
        if raw_save:
            save_p, err = _resolve_user_dir(raw_save, must_exist=False, create=True)
            if save_p is None:
                return jsonify({"ok": False, "message": f"save_dir 无效: {err}"}), 400
        else:
            targets = new_offset_data_write_targets()
            if not targets:
                return jsonify({"ok": False, "message": "未解析到默认 new_offset_data 目录"}), 500
            save_p = targets[0]
        with _api_lock:
            with node._band_csv_state_lock:
                if node._band_csv_playing:
                    return jsonify(
                        {"ok": False, "message": "CSV Action 顺序播放中，请结束后再生成"}
                    ), 409
            with node._traj_lock:
                if node._playing:
                    return jsonify({"ok": False, "message": "轨迹播放中，请结束后再生成"}), 409
            saved = node.load_saved_offsets()
            if saved is not None:
                left_6, right_6 = saved
                source_hint = "saved"
            else:
                left_6, right_6, oerr = node.get_pose_offsets_6()
                if oerr:
                    return jsonify(
                        {"ok": False, "message": f"未找到已保存偏移，且实时偏移不可用：{oerr}"}
                    ), 400
                source_hint = "live"
        ol, ort = format_offset_6(left_6, right_6)
        try:
            urdf = node.resolve_urdf_path_for_transform()
        except Exception as ex:
            return jsonify({"ok": False, "message": f"URDF 路径解析失败: {ex}"}), 500

        results: List[Dict[str, Any]] = []
        all_ok = True

        for raw_name in names:
            name = str(raw_name or "").strip()
            if not name or "/" in name or "\\" in name or name.startswith("."):
                results.append({"input": name, "ok": False, "message": "非法文件名"})
                all_ok = False
                continue
            if Path(name).suffix.lower() != ".csv":
                results.append({"input": name, "ok": False, "message": "仅支持 .csv 文件"})
                all_ok = False
                continue
            in_path = (src_p / name).resolve()
            try:
                in_path.relative_to(src_p)
            except ValueError:
                results.append({"input": name, "ok": False, "message": "越界路径"})
                all_ok = False
                continue
            if not in_path.is_file():
                results.append({"input": name, "ok": False, "message": f"文件不存在: {in_path}"})
                all_ok = False
                continue
            out_name = name
            out_path = (save_p / out_name).resolve()
            try:
                out_path.relative_to(save_p)
            except ValueError:
                results.append({"input": name, "ok": False, "message": "输出越界"})
                all_ok = False
                continue
            ok, msg = run_arm_trajectory_transform_jacobian(
                urdf,
                str(in_path),
                str(out_path),
                offset_left=ol,
                offset_right=ort,
            )
            if not ok:
                node.get_logger().error(f"[band_csv] {name} 生成失败: {msg}")
                results.append(
                    {"input": name, "ok": False, "message": (msg or "未知错误")[:2000]}
                )
                all_ok = False
                continue
            results.append(
                {
                    "input": name,
                    "input_path": str(in_path),
                    "output_path": str(out_path),
                    "output_name": out_name,
                    "ok": True,
                }
            )
            try:
                out_rel = out_path.relative_to(save_p).as_posix()
                for nroot in new_offset_data_write_targets():
                    if save_p == nroot:
                        node.record_generated_offset_rel(f"new_offset_data/{out_rel}")
                        break
            except Exception:
                pass
        return jsonify(
            {
                "ok": all_ok,
                "message": ("批量生成完成" if all_ok else "批量生成部分失败，详见 results"),
                "offset_source": source_hint,
                "save_dir": str(save_p),
                "offset_left": ol,
                "offset_right": ort,
                "results": results,
            }
        )

    @app.post("/api/band_csv/play_sequence")
    def api_band_csv_play_sequence():
        """按 ``files`` 绝对路径顺序，依次向 ``/motion/action/play`` 发送 ActionPlay（action_name = 无扩展名基名）。"""
        body = request.get_json(silent=True) or {}
        node.get_logger().info(f"[Web][band_csv] Action 顺序播放: body={body!r}")
        if node._action_play_client is None:
            return jsonify(
                {
                    "ok": False,
                    "message": "ActionPlay 不可用（请编译并 source crb_ros_msg）",
                }
            ), 503
        files = body.get("files") or []
        if not isinstance(files, list) or not files:
            return jsonify({"ok": False, "message": "files 为空"}), 400
        abs_list: List[Path] = []
        for raw in files:
            p = Path(str(raw or "").strip()).expanduser()
            if not p.is_absolute():
                return jsonify({"ok": False, "message": f"需要绝对路径: {raw}"}), 400
            if not p.is_file():
                return jsonify({"ok": False, "message": f"文件不存在: {p}"}), 400
            if p.suffix.lower() != ".csv":
                return jsonify({"ok": False, "message": f"需要 .csv 文件: {p}"}), 400
            abs_list.append(p.resolve())

        seg_info = [{"path": str(p), "action_name": p.stem} for p in abs_list]

        def _worker() -> None:
            try:
                for p in abs_list:
                    an = p.stem
                    ok, msg = node._action_play_sync(an, node._band_csv_action_timeout_sec)
                    node.get_logger().info(f"[band_csv] ActionPlay 片段: {an} ok={ok} {msg}")
                    if not ok:
                        node.get_logger().error(f"[band_csv] 顺序播放中止: {an} — {msg}")
                        break
            finally:
                with node._band_csv_state_lock:
                    node._band_csv_playing = False

        with _api_lock:
            with node._band_csv_state_lock:
                if node._band_csv_playing:
                    return jsonify(
                        {"ok": False, "message": "CSV Action 顺序播放已在进行中"}
                    ), 409
            with node._traj_lock:
                if node._playing:
                    return jsonify(
                        {"ok": False, "message": "标定轨迹正在播放中，请等待完成后再试"}
                    ), 409
            with node._band_csv_state_lock:
                node._band_csv_playing = True
            threading.Thread(target=_worker, daemon=True).start()

        return jsonify({"ok": True, "started": True, "segments": seg_info})

    @app.post("/api/upper_body_debug")
    def api_upper_body_debug():
        body = request.get_json(silent=True) or {}
        node.get_logger().info(f"[Web] 上半身调试: body={body!r}")
        if "enable" not in body:
            return jsonify({"ok": False, "message": "缺少 enable"}), 400
        enable = bool(body["enable"])
        with _api_lock:
            if not node._debug_cli.wait_for_service(timeout_sec=2.0):
                return jsonify({"ok": False, "message": "服务 /motion/upper_body_debug 不可用"}), 503
            req = SetBool.Request()
            req.data = enable
            fut = node._debug_cli.call_async(req)
            if not _wait_service_future(fut, timeout_sec=5.0):
                return jsonify({"ok": False, "message": "调用上身调试服务超时"}), 504
            try:
                resp = fut.result()
            except Exception as ex:
                return jsonify({"ok": False, "message": str(ex)}), 500
            if resp is None:
                return jsonify({"ok": False, "message": "服务无响应"}), 503
            if not resp.success:
                return jsonify({"ok": False, "message": resp.message or "服务返回失败"}), 500
            node._upper_body_debug_enabled_local = enable
            return jsonify({"ok": True, "message": resp.message or "ok"})

    @app.post("/api/calibration/start")
    def api_calibration_start():
        body = request.get_json(silent=True) or {}
        node.get_logger().info(f"[Web] 开始标定: body={body!r}")
        instruments = body.get("instruments") or []
        if not instruments:
            return jsonify({"ok": False, "message": "请至少选择一种乐器"}), 400
        with _api_lock:
            with node._traj_lock:
                if node._playing:
                    return jsonify(
                        {
                            "ok": False,
                            "message": "标定轨迹正在播放中，请等待完成后再点击",
                        }
                    ), 409
            rows = node._load_calibration_rows(instruments, "start")
            if not rows:
                return jsonify({"ok": False, "message": "未加载到任何轨迹数据（检查 resource 下对应 .data 文件）"}), 400
            with node._traj_lock:
                node._traj_rows = rows
                node._traj_index = 0
                node._playing = True
                node._traj_mode = "start"
            return jsonify({"ok": True, "frames": len(rows)})

    @app.post("/api/calibration/end")
    def api_calibration_end():
        body = request.get_json(silent=True) or {}
        node.get_logger().info(f"[Web] 结束标定: body={body!r}")
        instruments = body.get("instruments") or []
        if not instruments:
            return jsonify({"ok": False, "message": "请至少选择一种乐器"}), 400
        with _api_lock:
            with node._traj_lock:
                if node._playing:
                    return jsonify(
                        {
                            "ok": False,
                            "message": "标定轨迹正在播放中，请等待完成后再点击",
                        }
                    ), 409
            rows = node._load_calibration_rows(instruments, "end")
            if not rows:
                return jsonify({"ok": False, "message": "未加载到任何轨迹数据（检查 resource 下对应 .data 文件）"}), 400
            with node._traj_lock:
                node._traj_rows = rows
                node._traj_index = 0
                node._playing = True
                node._traj_mode = "end"
            return jsonify({"ok": True, "frames": len(rows)})

    return app


class ArmCalibrationWebNode(Node):
    def __init__(self) -> None:
        super().__init__("casbot_arm_calibration_web")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("action_play_topic", "/motion/action/play")
        self.declare_parameter("band_csv_action_timeout_sec", 7200.0)
        # 空字符串：使用 share/casbot_arm_calibration_web/urdf/<默认 .urdf>（见 _package_urdf_dir）
        self.declare_parameter("robot_urdf_path", "")
        self.declare_parameter("joint_states_topic", "/joint_states")
        # true：笛卡尔示教使用 Pinocchio 迭代 IK；false：仅用 arm_fk 数值 DLS（参数名沿用 use_trac_ik）
        self.declare_parameter("use_trac_ik", True)
        # 单次示教 / 100Hz 轨迹发布到 /upper_body_debug/joint_cmd 时的 time_ref（秒）、vel_scale
        self.declare_parameter("joint_cmd_time_ref", 1.0)
        self.declare_parameter("joint_cmd_vel_scale", 1.0)
        self.declare_parameter("joint_cmd_topic", "/upper_body_debug/joint_cmd")
        # 「刷新初始值」后若干次示教：每次前先 merge(/joint_states, last_cmd)，避免仍 hold 标定末帧。
        # 默认 0：刷新时已把 merge 结果写入 ``_last_joint_cmd_positions``，与标定末帧后一致，避免
        # 每次示教前用实时反馈重刷全身基线导致对侧臂/正交轴耦合异常；需要旧行为时可改为 8 等。
        self.declare_parameter("jog_hold_baseline_feedback_merges_after_refresh", 0)
        # 「开始标定」末帧播完后等待本秒数（默认 3），再将此时 ``_ee_*`` 拷入初值（与界面 current 数值一致）
        self.declare_parameter("calibration_initial_ee_joint_resample_sec", 3.0)
        # 「刷新初始值」时是否先调用 /motion/upper_body_debug(true)（与点击开始标定一致，否则 joint_cmd 可能被忽略）
        self.declare_parameter("refresh_initial_auto_enable_upper_body_debug", True)
        # 刷新成功后是否向 joint_cmd 下发与 baseline 相同的全身 hold（默认开），使控制器与示教基线一致
        self.declare_parameter("refresh_initial_publish_hold_baseline", True)
        self.declare_parameter("refresh_initial_publish_hold_repeat_count", 1)
        # 下发 hold 后等待本秒数，再根据 /joint_states 驱动的界面 FK 写入左右腕初值（覆盖「开始标定」写入的初值）
        self.declare_parameter("refresh_initial_ee_resample_sec", 3.0)
        # action_data_offset 包内 URDF 相对路径（由 action_data_paths 解析；
        # 现由 Pinocchio 完全承担 FK/IK，输入必须为 .urdf）
        self.declare_parameter(
            "action_data_urdf_relative",
            "urdf/CASBOT02_ENCOS_7dof_shell_20251015_P1L.urdf",
        )

        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self._urdf_dir = _package_urdf_dir()
        urdf_path = _resolve_robot_urdf_path(str(self.get_parameter("robot_urdf_path").value))
        js_topic = str(self.get_parameter("joint_states_topic").value)
        self._joint_cmd_time_ref = float(self.get_parameter("joint_cmd_time_ref").value)
        self._joint_cmd_vel_scale = float(self.get_parameter("joint_cmd_vel_scale").value)
        joint_cmd_topic = str(self.get_parameter("joint_cmd_topic").value)

        self.get_logger().info(f"机器人模型目录（优先 share）: {self._urdf_dir}")

        self._fk_lock = threading.Lock()
        self._joint_positions_by_name: Dict[str, float] = {}
        self._joint_index_by_urdf: Dict[str, int] = {jn: i for i, jn in enumerate(JOINTS_WITH_SUFFIX)}

        self._fk: Optional[UrdfArmFk] = UrdfArmFk(urdf_path)
        self._fk_ok = self._fk.load()
        self._fk_pose_ready = False
        self._ee_left = (0.0, 0.0, 0.0)
        self._ee_right = (0.0, 0.0, 0.0)
        # 腕部连杆姿态相对 base_link：R = Rz*Ry*Rx 的 (roll, pitch, yaw) 弧度，对应 RX/RY/RZ
        self._ee_left_rpy = (0.0, 0.0, 0.0)
        self._ee_right_rpy = (0.0, 0.0, 0.0)
        # 「开始标定」末帧播完并等待 settle 后，由 ``/joint_states`` FK 写入；None 表示尚未写入
        self._initial_ee_left: Optional[Tuple[float, float, float]] = None
        self._initial_ee_right: Optional[Tuple[float, float, float]] = None
        self._initial_rpy_left: Optional[Tuple[float, float, float]] = None
        self._initial_rpy_right: Optional[Tuple[float, float, float]] = None
        # 上次发布的全身关节指令；joint_states 缺某臂关节时用它回退，避免误发 0 导致另一臂被拉向零位
        self._last_joint_cmd_positions: Optional[List[float]] = None
        # 刷新初始值后：前 N 次示教在 _build_hold_baseline_for_jog 里先按反馈 merge 再 hold
        self._hold_baseline_feedback_merges_remaining: int = 0
        self._jog_fk_operating_side: Optional[str] = None
        # 单臂 jog 期间：对侧整条 FK 链关节快照（含头/腰等共享关节），避免仅冻 7 臂关节时末端显示仍随反馈抖
        self._jog_fk_freeze_opposite_chain_q: Optional[Dict[str, float]] = None
        # 操作侧链上除该侧 7 臂关节外的快照（头/腰等），减轻躯干微抖在「只沿 Y」时耦合进 X/Z 偏移显示
        self._jog_fk_freeze_operating_chain_except_arm: Optional[Dict[str, float]] = None
        if not self._fk_ok:
            self.get_logger().warn(
                f"正运动学未加载: {self._fk.load_error}（当前值将显示为 —），模型文件: {urdf_path}"
            )
        else:
            mode = getattr(self._fk, "_mode", "")
            self.get_logger().info(
                f"末端 FK 已加载 ({mode}): {urdf_path}，左臂 {len(self._fk._left_chain)} 关节，右臂 {len(self._fk._right_chain)} 关节"
            )

        # 用于直线 IK：7 关节名（按 URDF 链顺序，取链末端 7 个关节）与其限位
        self._left_arm_joint_names: List[str] = (
            list(self._fk._left_chain[-7:]) if self._fk_ok else []
        )
        self._right_arm_joint_names: List[str] = (
            list(self._fk._right_chain[-7:]) if self._fk_ok else []
        )
        self._arm_joint_limits: Dict[str, Tuple[float, float]] = {}
        if self._fk_ok:
            self._arm_joint_limits = parse_joint_limits(
                urdf_path, self._left_arm_joint_names + self._right_arm_joint_names
            )
            self.get_logger().info(
                f"[IK] 左臂链({len(self._fk._left_chain)}关节): {self._fk._left_chain}"
            )
            self.get_logger().info(
                f"[IK] 右臂链({len(self._fk._right_chain)}关节): {self._fk._right_chain}"
            )
            self.get_logger().info(f"[IK] 左臂 7 关节: {self._left_arm_joint_names}")
            self.get_logger().info(f"[IK] 右臂 7 关节: {self._right_arm_joint_names}")
            for jn in self._left_arm_joint_names + self._right_arm_joint_names:
                lo, hi = self._arm_joint_limits.get(jn, (float("nan"), float("nan")))
                self.get_logger().info(f"[IK] 限位 {jn}: [{lo:+.3f}, {hi:+.3f}] rad")

        self._use_trac_ik = bool(self.get_parameter("use_trac_ik").value)
        self._kin_trac: Optional[KinematicsPinTrac] = None
        if self._fk_ok and self._fk is not None:
            self._kin_trac = KinematicsPinTrac(
                urdf_path,
                list(self._fk._left_chain),
                list(self._fk._right_chain),
            )
            if self._kin_trac.available:
                self.get_logger().info(
                    "[Kin] Pinocchio 已加载：末端 FK 与示教 IK（use_trac_ik=true 时）均走 Pinocchio"
                )
            else:
                self.get_logger().warn(
                    f"[Kin] Pinocchio 未启用: {self._kin_trac.load_error}；"
                    "示教 IK 将使用 arm_fk 数值 DLS，末端 FK 以 UrdfArmFk 为主"
                )
        if not self._use_trac_ik:
            self.get_logger().info(
                "[Kin] use_trac_ik=false：笛卡尔示教使用 arm_fk 数值 DLS（若 Pinocchio 已加载仍可用于 FK）"
            )

        # 直线示教参数（可通过参数调节）
        #   _jog_step_mm_per_waypoint：每个 IK 路点的笛卡尔步长（mm）；大一点 → Δq 大，
        #                              电机明显可见，但越过 ~5mm 直线误差开始变大
        #   _jog_ticks_per_waypoint： 每个路点在 100Hz 定时器里重复发几次（=N），
        #                              Cartesian 速度 = step_mm/(N*10ms)
        self.declare_parameter("jog_step_mm_per_waypoint", 2.0)
        self.declare_parameter("jog_ticks_per_waypoint", 10)
        # 姿态示教：每个 IK 子步绕 base_link 轴的最大转角（度），与直线示教的 mm/子步对应
        self.declare_parameter("jog_rot_step_deg_per_waypoint", 0.1)
        self._jog_step_mm_per_waypoint = float(self.get_parameter("jog_step_mm_per_waypoint").value)
        self._jog_ticks_per_waypoint = max(1, int(self.get_parameter("jog_ticks_per_waypoint").value))
        self._jog_rot_step_deg_per_waypoint = float(
            self.get_parameter("jog_rot_step_deg_per_waypoint").value
        )
        if self._jog_rot_step_deg_per_waypoint <= 0.0:
            self._jog_rot_step_deg_per_waypoint = 0.1
        # True：非操作臂等用 /joint_states merge（适合终端只发部分 joint_cmd 后与真机对齐）。
        # False（默认）：全身与 base_hold 一致，标定刚结束时避免反馈微抖写进示教轨迹导致对侧末端偏移显示乱跳。
        self.declare_parameter("jog_merge_joint_states_into_non_operating_arm", False)
        # 单臂示教开始时把命令里的腰与当前 /joint_states 对齐一次，减轻控制器与 FK 假设不一致
        self.declare_parameter("jog_sync_waist_from_joint_states_on_jog_start", True)
        # 腰角：仅当与 hold 差值超过本弧度时才用 /joint_states 覆盖（默认 ~0.057°）；≤0 则始终对齐
        self.declare_parameter("jog_waist_resync_min_delta_rad", 1.0e-3)

        self._joint_state_sub = self.create_subscription(
            JointState,
            js_topic,
            self._on_joint_states,
            10,
        )

        self._web_root = _web_root()
        if not (self._web_root / "index.html").is_file():
            self.get_logger().warn(f"Web 资源未找到: {self._web_root}")

        self._resource_root = _resource_root()
        self._debug_cli = self.create_client(SetBool, "/motion/upper_body_debug")
        # 与 /api/upper_body_debug、刷新初始值、band 自动关调试同步；为 True 时刷新不再重复 SetBool(true)
        self._upper_body_debug_enabled_local: bool = False

        self._band_csv_playing = False
        self._band_csv_state_lock = threading.Lock()
        ap_topic = str(self.get_parameter("action_play_topic").value)
        self._band_csv_action_timeout_sec = float(
            self.get_parameter("band_csv_action_timeout_sec").value
        )
        if ActionPlayMsg is not None:
            self._action_play_client = ActionClient(self, ActionPlayMsg, ap_topic)
            self.get_logger().info(f"[band_csv] ActionPlay 客户端: {ap_topic}")
        else:
            self._action_play_client = None
            self.get_logger().warn(
                "未导入 crb_ros_msg.action.ActionPlay：「乐队 CSV」Action 播放不可用；请编译 crb_ros_msg 后重新 source。"
            )

        if UpperJointData is not None:
            self._joint_pub = self.create_publisher(UpperJointData, joint_cmd_topic, 10)
            self.create_subscription(
                UpperJointData,
                joint_cmd_topic,
                self._on_incoming_joint_cmd,
                _JOINT_CMD_SUB_QOS,
            )
            self.get_logger().info(f"joint_cmd 发布/订阅: {joint_cmd_topic}（订阅 QoS: BEST_EFFORT）")
            self.get_logger().info(
                f"joint_cmd 发布默认: time_ref={self._joint_cmd_time_ref}s vel_scale={self._joint_cmd_vel_scale} "
                "（与终端 ros2 topic pub 一致可减轻示教前后整身插补突变）"
            )
        else:
            self._joint_pub = None
            self.get_logger().error(
                "无法导入 crb_ros_msg.msg.UpperJointData，关节命令将无法发布；请安装 crb_ros_msg 并 source 环境。"
            )

        self._traj_lock = threading.Lock()
        self._traj_rows: List[List[float]] = []
        self._traj_index = 0
        self._playing = False
        self._traj_mode: str = ""  # 当前播放来源：start | end | ""

        self._recent_generated_offset_lock = threading.Lock()
        self._recent_generated_offset_rels: List[str] = []

        self._timer = self.create_timer(0.01, self._on_timer_100hz)

        self._app = create_app(self)
        self._thread = threading.Thread(
            target=lambda: self._app.run(
                host=host,
                port=port,
                threaded=True,
                use_reloader=False,
            ),
            daemon=True,
        )
        self._thread.start()
        _log_calibration_web_urls(self.get_logger(), host, port, self._resource_root)

    def _action_play_sync(self, action_name: str, timeout_sec: float) -> Tuple[bool, str]:
        """阻塞直到 ActionPlay 执行结束或超时；依赖 MultiThreadedExecutor 处理 Action 回调。"""
        if ActionPlayMsg is None or self._action_play_client is None:
            return False, "ActionPlay 不可用"
        cli = self._action_play_client
        if not cli.wait_for_server(timeout_sec=10.0):
            ap = str(self.get_parameter("action_play_topic").value)
            return False, f"Action 服务端未就绪: {ap}"
        goal = ActionPlayMsg.Goal()
        goal.start_time = 0.0
        goal.action_name = action_name
        goal.cancel_action_name = ""
        goal.rl_name = ""

        def _fb(fb: Any) -> None:
            self.get_logger().info(
                f"[band_csv] ActionPlay 反馈: state={fb.state} "
                f"action_index={fb.action_index} exec_time={fb.exec_time:.3f} ({action_name})"
            )

        send_fut = cli.send_goal_async(goal, feedback_callback=_fb)
        t0 = time.time()
        while rclpy.ok() and not send_fut.done() and (time.time() - t0) < 30.0:
            time.sleep(0.02)
        if not send_fut.done():
            return False, "send_goal 超时"
        goal_handle = send_fut.result()
        if goal_handle is None:
            return False, "Goal handle 为空"
        if not goal_handle.accepted:
            return False, "Goal 被拒绝"
        res_fut = goal_handle.get_result_async()
        t0 = time.time()
        while rclpy.ok() and not res_fut.done() and (time.time() - t0) < max(1.0, timeout_sec):
            time.sleep(0.05)
        if not res_fut.done():
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            return False, "等待动作结束超时"
        res = res_fut.result()
        if res is None:
            return False, "结果为空"
        st = int(getattr(res, "status", -1))
        if_success = bool(getattr(getattr(res, "result", None), "if_success", False))
        succeeded = GoalStatus is not None and st == GoalStatus.STATUS_SUCCEEDED
        if GoalStatus is None:
            succeeded = st == 4
        ok_exec = succeeded and if_success
        return ok_exec, ("完成" if ok_exec else f"未成功 status={st} if_success={if_success}")

    def _on_incoming_joint_cmd(self, msg: Any) -> None:
        """非轨迹播放时，把终端/其他节点发布的部分关节 UpperJointData 合并进 hold 缓存。"""
        if self._playing:
            return
        js = getattr(msg, "joint", None)
        if js is None:
            return
        names = getattr(js, "name", None) or []
        positions = getattr(js, "position", None) or []
        if not names or not positions:
            return
        m = min(len(names), len(positions))
        if m <= 0:
            return
        row = self._merge_joint_state_from_cache_and_last_cmd()

        # 若上一轮 topic 没带某关节而用 merge（反馈缺省）补了占位，本条消息可把相应关节再写精确
        for i in range(m):
            raw = str(names[i]).strip()
            jn = raw if raw.endswith("_joint") else f"{raw}_joint"
            idx = self._joint_index_by_urdf.get(jn)
            if idx is None:
                continue
            if 0 <= idx < len(row):
                row[idx] = float(positions[i])
        with self._fk_lock:
            self._last_joint_cmd_positions = row

    def _on_joint_states(self, msg: JointState) -> None:
        if not msg.name or not msg.position:
            return
        n_name = len(msg.name)
        n_pos = len(msg.position)
        if n_name != n_pos:
            self.get_logger().debug(
                f"/joint_states name/position 长度不一致 ({n_name}/{n_pos})，按 min 成对更新缓存"
            )
        m = min(n_name, n_pos)
        if m <= 0:
            return
        with self._fk_lock:
            for i in range(m):
                n = msg.name[i]
                key = n if str(n).endswith("_joint") else f"{n}_joint"
                self._joint_positions_by_name[key] = float(msg.position[i])
        if not self._fk_ok:
            return
        try:
            fk = self._fk
            q_l = build_q_map_for_chain(fk._left_chain, msg.name, msg.position)
            q_r = build_q_map_for_chain(fk._right_chain, msg.name, msg.position)
            op = self._jog_fk_operating_side
            jog_ex = self._jog_fk_freeze_operating_chain_except_arm or {}
            jog_opp = self._jog_fk_freeze_opposite_chain_q or {}
            with self._traj_lock:
                jog_disp = self._playing and (self._traj_mode == "jog")
            if jog_disp:
                if op == "left":
                    q_l = dict(q_l)
                    for jn, v in jog_ex.items():
                        q_l[jn] = float(v)
                    q_r = dict(q_r)
                    for jn, v in jog_opp.items():
                        q_r[jn] = float(v)
                elif op == "right":
                    q_r = dict(q_r)
                    for jn, v in jog_ex.items():
                        q_r[jn] = float(v)
                    q_l = dict(q_l)
                    for jn, v in jog_opp.items():
                        q_l[jn] = float(v)
            kin = self._kin_trac
            Tl = None
            Tr = None
            if kin is not None and kin.available:
                Tl = kin.fk_tip_T("left", q_l)
                Tr = kin.fk_tip_T("right", q_r)
            if Tl is None:
                Tl = fk.fk_chain_transform(fk._left_chain, q_l)
            if Tr is None:
                Tr = fk.fk_chain_transform(fk._right_chain, q_r)
            if Tl is None or Tr is None:
                return
            pl = np.asarray(Tl[:3, 3], dtype=float)
            pr = np.asarray(Tr[:3, 3], dtype=float)
            rpy_l = rotation_matrix_to_rpy(np.asarray(Tl[:3, :3], dtype=float))
            rpy_r = rotation_matrix_to_rpy(np.asarray(Tr[:3, :3], dtype=float))
            with self._fk_lock:
                if pl is not None:
                    self._ee_left = (float(pl[0]), float(pl[1]), float(pl[2]))
                if pr is not None:
                    self._ee_right = (float(pr[0]), float(pr[1]), float(pr[2]))
                self._ee_left_rpy = (float(rpy_l[0]), float(rpy_l[1]), float(rpy_l[2]))
                self._ee_right_rpy = (float(rpy_r[0]), float(rpy_r[1]), float(rpy_r[2]))
                self._fk_pose_ready = True
        except Exception as e:
            self.get_logger().debug(f"关节 FK 更新失败: {e}")

    def get_linear_offsets_mm(
        self,
    ) -> Tuple[Optional[Tuple[float, float, float]], Optional[Tuple[float, float, float]], str]:
        """
        相对「初始值」的直线位移（毫米），用于 arm_trajectory_transform_jacobian 的 offset-left/right。
        初始值来自「开始标定」末帧播完后等待若干秒，再将当时界面所用的末端位姿拷入初值（参数
        ``calibration_initial_ee_joint_resample_sec``，默认 3s）；或用户点击「刷新初始值」。
        """
        with self._fk_lock:
            ready = self._fk_pose_ready and self._fk_ok
            il, ir = self._initial_ee_left, self._initial_ee_right
            el, er = self._ee_left, self._ee_right
        if not ready:
            return None, None, "末端位姿未就绪，请确认 /joint_states 与 URDF 加载正常"
        if il is None or ir is None:
            return (
                None,
                None,
                "左右臂「初始值」未设置：请先播放「开始标定」至结束并等待约 3s 写入初值，或点左/右任一侧「刷新初始值」（会同步双臂）",
            )
        left_mm = (
            (el[0] - il[0]) * 1000.0,
            (el[1] - il[1]) * 1000.0,
            (el[2] - il[2]) * 1000.0,
        )
        right_mm = (
            (er[0] - ir[0]) * 1000.0,
            (er[1] - ir[1]) * 1000.0,
            (er[2] - ir[2]) * 1000.0,
        )
        return left_mm, right_mm, ""

    def get_pose_offsets_6(
        self,
    ) -> Tuple[
        Optional[Tuple[float, float, float, float, float, float]],
        Optional[Tuple[float, float, float, float, float, float]],
        str,
    ]:
        """
        相对「初始值」的 6D 位姿偏移：(x_mm, y_mm, z_mm, rx_rad, ry_rad, rz_rad)。

        - 平移：``当前 − 初始``，单位毫米。
        - 旋转：``R_cur·R_init^T`` 转为 **scipy 外旋 XYZ** 欧拉角 (rx, ry, rz)，与
          ``arm_trajectory_transform_jacobian --euler-seq XYZ --angle-unit rad``
          保持一致（即 ``R_rel = Rx(rx)·Ry(ry)·Rz(rz)``，而 **非** URDF 的 ``Rz·Ry·Rx``）。

        注意：UI 中「偏移 RX/RY/RZ」显示的是 ``R_rel`` 的**旋转向量**分量（度，matrix log）；
        本函数返回的是同一 ``R_rel`` 的 **欧拉角**（rad）——两者表示同一旋转，但数值一般不相等。
        """
        from scipy.spatial.transform import Rotation as SciRotation  # noqa: PLC0415

        with self._fk_lock:
            ready = self._fk_pose_ready and self._fk_ok
            il, ir = self._initial_ee_left, self._initial_ee_right
            ilr, irr = self._initial_rpy_left, self._initial_rpy_right
            el, er = self._ee_left, self._ee_right
            elr, err_rpy = self._ee_left_rpy, self._ee_right_rpy
        if not ready:
            return None, None, "末端位姿未就绪，请确认 /joint_states 与 URDF 加载正常"
        if il is None or ir is None or ilr is None or irr is None:
            return (
                None,
                None,
                "左右臂「初始值」未设置：请先播放「开始标定」至结束并等待约 3s 写入初值，或点左/右任一侧「刷新初始值」（会同步双臂）",
            )

        def _rel_euler_xyz(cur_rpy: Tuple[float, float, float], init_rpy: Tuple[float, float, float]) -> Tuple[float, float, float]:
            R_c = rpy_to_R(np.array(cur_rpy, dtype=float))
            R_i = rpy_to_R(np.array(init_rpy, dtype=float))
            R_rel = R_c @ R_i.T
            eul = SciRotation.from_matrix(R_rel).as_euler("XYZ", degrees=False)
            return float(eul[0]), float(eul[1]), float(eul[2])

        lrx, lry, lrz = _rel_euler_xyz(elr, ilr)
        rrx, rry, rrz = _rel_euler_xyz(err_rpy, irr)
        left_6 = (
            (el[0] - il[0]) * 1000.0,
            (el[1] - il[1]) * 1000.0,
            (el[2] - il[2]) * 1000.0,
            lrx,
            lry,
            lrz,
        )
        right_6 = (
            (er[0] - ir[0]) * 1000.0,
            (er[1] - ir[1]) * 1000.0,
            (er[2] - ir[2]) * 1000.0,
            rrx,
            rry,
            rrz,
        )
        return left_6, right_6, ""

    def saved_offsets_path(self) -> Path:
        """持久化的 6D 偏移量 JSON：与 ``new_offset_data`` 同根（:func:`calibration_web_package_root`）。"""
        return calibration_web_package_root() / "web_saved_offsets.json"

    def persist_saved_offsets(
        self,
        left_6: Tuple[float, float, float, float, float, float],
        right_6: Tuple[float, float, float, float, float, float],
    ) -> Path:
        """把 6D 偏移量写入 JSON（覆盖）；返回落地文件路径。"""
        import json
        from datetime import datetime

        p = self.saved_offsets_path()
        p.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "saved_at": datetime.now().isoformat(timespec="seconds"),
            "unit": {"length": "mm", "angle": "rad", "euler_seq": "XYZ"},
            "left": {
                "x": left_6[0],
                "y": left_6[1],
                "z": left_6[2],
                "rx": left_6[3],
                "ry": left_6[4],
                "rz": left_6[5],
            },
            "right": {
                "x": right_6[0],
                "y": right_6[1],
                "z": right_6[2],
                "rx": right_6[3],
                "ry": right_6[4],
                "rz": right_6[5],
            },
        }
        p.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
        return p

    def load_saved_offsets(
        self,
    ) -> Optional[
        Tuple[
            Tuple[float, float, float, float, float, float],
            Tuple[float, float, float, float, float, float],
        ]
    ]:
        """读取持久化的 6D 偏移量；不存在/损坏返回 ``None``。"""
        import json

        p = self.saved_offsets_path()
        if not p.is_file():
            return None
        try:
            d = json.loads(p.read_text(encoding="utf-8"))
            l = d["left"]
            r = d["right"]
            left = (float(l["x"]), float(l["y"]), float(l["z"]), float(l["rx"]), float(l["ry"]), float(l["rz"]))
            right = (float(r["x"]), float(r["y"]), float(r["z"]), float(r["rx"]), float(r["ry"]), float(r["rz"]))
            return left, right
        except Exception as ex:
            self.get_logger().warn(f"读取持久化偏移失败: {ex}")
            return None

    def record_generated_offset_rel(self, rel: str) -> None:
        """记录本次进程内生成的 ``offset_data`` 下拉值（如 ``new_offset_data/xxx.data``），最新在前。"""
        r = (rel or "").strip().replace("\\", "/")
        if not r or ".." in r:
            return
        with self._recent_generated_offset_lock:
            if r in self._recent_generated_offset_rels:
                self._recent_generated_offset_rels.remove(r)
            self._recent_generated_offset_rels.insert(0, r)
            del self._recent_generated_offset_rels[48:]

    def list_recent_generated_offset_rels(self) -> List[str]:
        with self._recent_generated_offset_lock:
            return list(self._recent_generated_offset_rels)

    def resolve_urdf_path_for_transform(self) -> str:
        """解析 action_data_offset 包内 URDF 路径（Pinocchio FK/IK 使用）。"""
        rel = str(self.get_parameter("action_data_urdf_relative").value).strip()
        if not rel:
            rel = "urdf/CASBOT02_ENCOS_7dof_shell_20251015_P1L.urdf"
        try:
            from action_data_paths import resolve_model_path

            return resolve_model_path(rel)
        except Exception as e:
            try:
                from ament_index_python.packages import get_package_share_directory

                share = Path(get_package_share_directory("action_data_offset"))
                cand = (share / rel).resolve()
                if cand.is_file():
                    return str(cand)
            except Exception:
                pass
            raise RuntimeError(f"无法解析 URDF（请安装并 source action_data_offset）: {e}") from e

    def get_ui_state(self) -> Dict[str, Any]:
        """供 /api/state。

        - **线性格子 X/Y/Z**：``current`` / ``initial`` / ``delta`` 均为 **毫米 (mm)**，三位小数；
          ``delta`` = 当前 − 初始，与步进单位一致，避免原先用米显示成 ``0.000166`` 易被误解。
        - **姿态 RX/RY/RZ**：初始/当前为 identity→该姿态的旋转向量分量（度）；偏移为
          ``vee_log(R_cur·R_init^T)`` 分量（度），与绕固定轴示教一致。
        """
        axis_base = {"step": 1.0, "initial": 0.0, "delta": 0.0}
        dash = "—"
        rad2deg = 180.0 / math.pi
        with self._fk_lock:
            ready = self._fk_pose_ready and self._fk_ok
            lx, ly, lz = self._ee_left
            rx, ry, rz = self._ee_right
            il = self._initial_ee_left
            ir = self._initial_ee_right
            ilr = self._initial_rpy_left
            irr = self._initial_rpy_right
            lrx, lry, lrz = self._ee_left_rpy
            rrx, rry, rrz = self._ee_right_rpy
        if not ready:

            def ax() -> Dict[str, Any]:
                return {**axis_base, "current": dash, "initial": dash, "delta": dash}

            return {
                "left": {
                    "x": ax(),
                    "y": ax(),
                    "z": ax(),
                    "rx": ax(),
                    "ry": ax(),
                    "rz": ax(),
                },
                "right": {
                    "x": ax(),
                    "y": ax(),
                    "z": ax(),
                    "rx": ax(),
                    "ry": ax(),
                    "rz": ax(),
                },
                "euler_offset_deg": {"left": None, "right": None},
            }

        def fmt_lin_mm(m: float) -> str:
            """base_link 下平移，米 → 毫米展示。"""
            return f"{float(m) * 1000.0:.3f}"

        def axis_row(
            cur: float,
            initial_comp: Optional[float],
        ) -> Dict[str, Any]:
            if initial_comp is None:
                return {**axis_base, "current": fmt_lin_mm(cur), "initial": dash, "delta": dash}
            d_mm = (float(cur) - float(initial_comp)) * 1000.0
            return {
                **axis_base,
                "current": fmt_lin_mm(cur),
                "initial": fmt_lin_mm(initial_comp),
                "delta": f"{d_mm:+.3f}",
            }

        def rpy_rows(
            cur: Tuple[float, float, float],
            initial: Optional[Tuple[float, float, float]],
        ) -> Tuple[Dict[str, Any], Dict[str, Any], Dict[str, Any]]:
            """初始/当前：vee_log(R) 分量（度）。偏移：vee_log(R_cur·R_init^T)（度），非「当前−初始」。"""
            R_c = rpy_to_R(np.array(cur, dtype=float))
            w_cur = _vee_log_so3(R_c) * float(rad2deg)
            if initial is None:
                return (
                    {
                        **axis_base,
                        "current": f"{float(w_cur[0]):.3f}",
                        "initial": dash,
                        "delta": dash,
                    },
                    {
                        **axis_base,
                        "current": f"{float(w_cur[1]):.3f}",
                        "initial": dash,
                        "delta": dash,
                    },
                    {
                        **axis_base,
                        "current": f"{float(w_cur[2]):.3f}",
                        "initial": dash,
                        "delta": dash,
                    },
                )
            R_i = rpy_to_R(np.array(initial, dtype=float))
            w_init = _vee_log_so3(R_i) * float(rad2deg)
            R_rel = R_c @ R_i.T
            w_rel = _vee_log_so3(R_rel) * float(rad2deg)
            rows = []
            for k in range(3):
                ini = float(w_init[k])
                curc = float(w_cur[k])
                d = float(w_rel[k])
                rows.append(
                    {
                        **axis_base,
                        "initial": f"{ini:.3f}",
                        "current": f"{curc:.3f}",
                        "delta": f"{d:+.3f}",
                    }
                )
            return rows[0], rows[1], rows[2]

        lrx_row, lry_row, lrz_row = rpy_rows((lrx, lry, lrz), ilr)
        rrx_row, rry_row, rrz_row = rpy_rows((rrx, rry, rrz), irr)

        out_ui = {
            "left": {
                "x": axis_row(lx, il[0] if il else None),
                "y": axis_row(ly, il[1] if il else None),
                "z": axis_row(lz, il[2] if il else None),
                "rx": lrx_row,
                "ry": lry_row,
                "rz": lrz_row,
            },
            "right": {
                "x": axis_row(rx, ir[0] if ir else None),
                "y": axis_row(ry, ir[1] if ir else None),
                "z": axis_row(rz, ir[2] if ir else None),
                "rx": rrx_row,
                "ry": rry_row,
                "rz": rrz_row,
            },
        }
        # 与「保存 / 生成偏移数据」一致：R_cur·R_init^T 的外旋 XYZ 欧拉角（度），便于对照示教轴角
        le6, re6, _euler_err = self.get_pose_offsets_6()

        def _euler_blk(t6: Optional[Tuple[float, float, float, float, float, float]]) -> Optional[Dict[str, float]]:
            if t6 is None:
                return None
            return {
                "rx": float(t6[3] * rad2deg),
                "ry": float(t6[4] * rad2deg),
                "rz": float(t6[5] * rad2deg),
            }

        out_ui["euler_offset_deg"] = {
            "left": _euler_blk(le6),
            "right": _euler_blk(re6),
        }
        return out_ui

    def _load_calibration_rows(self, instruments: List[str], kind: str) -> List[List[float]]:
        """kind: start | end → ``BiaoDingA_*.data`` / ``BiaoDingB_*.data``，按乐器顺序拼接。"""
        all_rows: List[List[float]] = []
        for inst in instruments:
            subdir = INSTRUMENT_TO_SUBDIR.get(str(inst))
            if not subdir:
                self.get_logger().warn(f"未知乐器 id: {inst}")
                continue
            path, err = find_resource_biaoding_file(self._resource_root, subdir, kind)
            if path is None:
                self.get_logger().warn(err or f"缺少标定文件: {self._resource_root / subdir}")
                continue
            matches = list_resource_biaoding_files(self._resource_root, subdir, kind)
            if len(matches) > 1:
                pat = "BiaoDingA_*.data" if kind == "start" else "BiaoDingB_*.data"
                self.get_logger().info(
                    f"[calib] {subdir} 下发现 {len(matches)} 个 {pat}，使用 {path.name}"
                )
            try:
                rows = load_calibration_trajectory(path)
            except Exception as e:
                self.get_logger().error(f"读取失败 {path}: {e}")
                continue
            all_rows.extend(rows)
        return all_rows

    def _publish_joint_cmd(
        self,
        positions: List[float],
        time_ref: Optional[float] = None,
        vel_scale: Optional[float] = None,
    ) -> None:
        if self._joint_pub is None or UpperJointData is None:
            return
        tr = float(self._joint_cmd_time_ref if time_ref is None else time_ref)
        vs = float(self._joint_cmd_vel_scale if vel_scale is None else vel_scale)
        msg = UpperJointData()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.time_ref = tr
        msg.vel_scale = vs
        js = JointState()
        js.name = list(JOINT_NAMES_PUBLISH)
        pos = [float(x) for x in positions]
        js.position = pos
        msg.joint = js
        self._joint_pub.publish(msg)
        with self._fk_lock:
            self._last_joint_cmd_positions = pos

    def _baseline_vector_joint_states_then_last_cmd(self) -> Tuple[List[float], int, int]:
        """
        与 ``JOINTS_WITH_SUFFIX`` 同序：每关节优先 ``/joint_states`` 缓存，否则上一条 ``joint_cmd``。
        返回 ``(向量, n_from_joint_states, n_fallback)``。
        """
        with self._fk_lock:
            cache = dict(self._joint_positions_by_name)
            last = list(self._last_joint_cmd_positions) if self._last_joint_cmd_positions is not None else None
        out: List[float] = []
        n_js = 0
        n_fb = 0
        for i, jn in enumerate(JOINTS_WITH_SUFFIX):
            if jn in cache:
                out.append(float(cache[jn]))
                n_js += 1
            elif last is not None and i < len(last):
                out.append(float(last[i]))
                n_fb += 1
            else:
                out.append(0.0)
                n_fb += 1
        return out, n_js, n_fb

    def _merge_joint_state_from_cache_and_last_cmd(self) -> List[float]:
        """示教 merge：优先关节反馈，缺一臂时用历史 joint_cmd（含本节点订阅合并）补齐。"""
        row, _, _ = self._baseline_vector_joint_states_then_last_cmd()
        return row

    def _build_hold_baseline_for_jog(self) -> List[float]:
        """
        单臂笛卡尔示教用全身 hold 向量。

        默认优先整条 ``_last_joint_cmd_positions``（减 sag）。

        「刷新初始值」后计数参数 ``jog_hold_baseline_feedback_merges_after_refresh``（默认 0）：
        若 >0，则前 N 次示教每次都先 merge(反馈 ∪ last_cmd) 并写回 last，再以当前姿态为起点。
        """
        with self._fk_lock:
            remain = int(self._hold_baseline_feedback_merges_remaining)
        if remain > 0:
            merged = self._merge_joint_state_from_cache_and_last_cmd()
            with self._fk_lock:
                self._last_joint_cmd_positions = list(merged)
                self._hold_baseline_feedback_merges_remaining = remain - 1
            self.get_logger().info(
                f"[IK] refresh 后按 /joint_states 合并 hold 基线（余 {remain - 1} 次示教仍先合并）"
            )
            return list(merged)

        with self._fk_lock:
            last = list(self._last_joint_cmd_positions) if self._last_joint_cmd_positions is not None else None
            cache = dict(self._joint_positions_by_name)
        if last is not None and len(last) == len(JOINTS_WITH_SUFFIX):
            return list(last)
        baseline: List[float] = []
        for i, jn in enumerate(JOINTS_WITH_SUFFIX):
            if jn in cache:
                baseline.append(float(cache[jn]))
            else:
                baseline.append(0.0)
        with self._fk_lock:
            self._last_joint_cmd_positions = list(baseline)
        self.get_logger().info(
            "[IK] 首次直线示教：已把当前 /joint_states 快照作为 hold 基线 "
            "（后续示教不再读取实时 cache 的 sag 值）"
        )
        return baseline

    def _sync_joint_states_into_base_full_except_operating_arm(
        self,
        base_full: List[float],
        operating_side: str,
        cache: Dict[str, float],
    ) -> None:
        """
        单臂示教前：除操作侧 7 个臂关节外，凡 /joint_states 里有的关节一律写成当前反馈。

        **对侧 7 个臂关节不覆盖**：保持 ``base_hold``/指令基线中的值，与「开始标定」末帧后
        单臂笛卡尔示教一致，避免对侧腕 XYZ 随编码器微抖被写进 ``UpperJointData``，以及 Pin
        全模型种子中对侧角抖动导致本侧「纯 Y」出现 X/Z 分量。

        **头、腰**与**对侧 7 臂**均不覆盖：保持 hold，避免外部 topic 只动一侧后首段示教用反馈微差经运动学树传到对侧腕。**灵巧手**仍用反馈对齐。
        """
        if len(base_full) != len(JOINTS_WITH_SUFFIX):
            return
        os = str(operating_side).strip().lower()
        if os == "left":
            keep = _LEFT_ARM_JOINT_INDICES
            opp = _RIGHT_ARM_JOINT_INDICES
        elif os == "right":
            keep = _RIGHT_ARM_JOINT_INDICES
            opp = _LEFT_ARM_JOINT_INDICES
        else:
            return
        for i, jn in enumerate(JOINTS_WITH_SUFFIX):
            if keep.start <= i < keep.stop:
                continue
            if opp.start <= i < opp.stop:
                continue
            if jn in _JOG_SYNC_SKIP_FROM_JOINT_STATES_NAMES:
                continue
            if jn in cache:
                base_full[i] = float(cache[jn])

    def _finalize_jog_publish_row(
        self,
        row: List[float],
        base_full: List[float],
        arm_slice: slice,
        wi_pub: Optional[int],
        w_rad_pub: Optional[float],
    ) -> None:
        """
        单臂示教每一帧下发向量：除操作侧 7 个臂关节外与 ``base_full`` 完全一致，再写腰（若启用示教腰对齐）。

        仅依赖「未改写的槽位仍等于 base_full」不够稳健；此处显式回写，避免任何误索引污染头/对侧/手。
        """
        if len(row) != len(base_full):
            return
        for i in range(len(row)):
            if arm_slice.start <= i < arm_slice.stop:
                continue
            row[i] = float(base_full[i])
        if wi_pub is not None and w_rad_pub is not None:
            wi = int(wi_pub)
            if 0 <= wi < len(row):
                row[wi] = float(w_rad_pub)

    def _jog_hold_with_waist_optional_sync(self, base_hold: List[float]) -> List[float]:
        """示教起点：可选把 ``waist_yaw_joint`` 与当前 /joint_states 对齐一次（整段轨迹保持该值）。"""
        out = list(base_hold)
        if not bool(self.get_parameter("jog_sync_waist_from_joint_states_on_jog_start").value):
            return out
        try:
            wi = JOINTS_WITH_SUFFIX.index("waist_yaw_joint")
        except ValueError:
            return out
        if wi >= len(out):
            return out
        with self._fk_lock:
            wv = self._joint_positions_by_name.get("waist_yaw_joint")
        if wv is not None:
            thr = float(self.get_parameter("jog_waist_resync_min_delta_rad").value)
            if thr <= 0.0 or abs(float(wv) - float(out[wi])) >= thr:
                out[wi] = float(wv)
        return out

    def _clear_jog_fk_display_override(self) -> None:
        self._jog_fk_operating_side = None
        self._jog_fk_freeze_opposite_chain_q = None
        self._jog_fk_freeze_operating_chain_except_arm = None

    def _apply_live_waist_to_chain_q_base(
        self, chain: List[str], chain_q_base: Dict[str, float], cache: Dict[str, float]
    ) -> None:
        """IK 链上 ``waist_yaw_joint``：仅当与 ``chain_q_base`` 已有值差异足够大再用 /joint_states，避免亚 mrad 噪声。"""
        wz = "waist_yaw_joint"
        if wz not in chain or wz not in cache:
            return
        liv = float(cache[wz])
        cur = float(chain_q_base.get(wz, liv))
        thr = float(self.get_parameter("jog_waist_resync_min_delta_rad").value)
        if thr <= 0.0 or abs(liv - cur) >= thr:
            chain_q_base[wz] = liv

    def _jog_publish_waist_index_and_rad(
        self, chain_q_base: Dict[str, float], base_full: List[float]
    ) -> Tuple[Optional[int], Optional[float]]:
        """与 IK 一致的腰角及在 UpperJointData 向量中的下标，用于每帧下发。"""
        wz = "waist_yaw_joint"
        try:
            wi = JOINTS_WITH_SUFFIX.index(wz)
        except ValueError:
            return None, None
        if wi < 0 or wi >= len(base_full):
            return None, None
        w = chain_q_base.get(wz)
        if w is None:
            w = float(base_full[wi])
        return wi, float(w)

    def _setup_jog_fk_display_overrides(
        self, side_l: str, base_full: List[float], chain_q_base: Dict[str, float]
    ) -> None:
        """
        单臂 jog：界面 FK 用示教起点的链快照。

        左右腕 FK 链共享头、腰等；若只冻对侧 7 个臂关节，共享关节仍随 /joint_states 微抖，
        会表现为「只动左臂 Y」时右臂偏移 X/Y/Z 全变、左臂也出现正交方向偏移。此处对侧整条链
        与操作侧链上「非本侧 7 臂关节」均用 ``chain_q_base``/``base_full`` 快照，仅操作侧 7
        臂关节继续用实时反馈。
        """
        self._jog_fk_operating_side = str(side_l).strip().lower()
        self._jog_fk_freeze_opposite_chain_q = None
        self._jog_fk_freeze_operating_chain_except_arm = None
        fk = self._fk
        if fk is None or not getattr(fk, "_loaded", False):
            return
        os = self._jog_fk_operating_side
        if os == "left":
            op_chain = list(fk._left_chain)
            opp_chain = list(fk._right_chain)
            arm_set = set(self._left_arm_joint_names)
        elif os == "right":
            op_chain = list(fk._right_chain)
            opp_chain = list(fk._left_chain)
            arm_set = set(self._right_arm_joint_names)
        else:
            return

        def snap_val(jn: str) -> Optional[float]:
            if jn in chain_q_base:
                return float(chain_q_base[jn])
            pid = self._joint_index_by_urdf.get(jn)
            if pid is not None and 0 <= pid < len(base_full):
                return float(base_full[pid])
            return None

        opp: Dict[str, float] = {}
        for jn in opp_chain:
            v = snap_val(jn)
            if v is not None:
                opp[jn] = v
        self._jog_fk_freeze_opposite_chain_q = opp if opp else None

        ex: Dict[str, float] = {}
        for jn in op_chain:
            if jn in arm_set:
                continue
            v = snap_val(jn)
            if v is not None:
                ex[jn] = v
        self._jog_fk_freeze_operating_chain_except_arm = ex if ex else None

    def _pin_ik_available_for_jog(self) -> bool:
        """Pinocchio 示教后端可用且参数允许使用（use_trac_ik）。"""
        return bool(
            self._use_trac_ik
            and self._kin_trac is not None
            and self._kin_trac.available
        )

    def _compose_jog_publish_baseline(self, base_hold: List[float], operating_side: str) -> List[float]:
        """
        构造示教下发的全身向量。

        默认（``jog_merge_joint_states_into_non_operating_arm=false``）：**整身与 ``base_hold`` 相同**。
        标定刚一结束即用 /joint_states 填非操作臂时，微小 sag/滤波摆幅会送进 100Hz 轨迹，
        再经由腰与各臂耦合，易出现「只点左臂 Y」却见右臂偏移量在 X/Y/Z 变化的现象。

        若需在用终端只对一侧发局部 ``joint_cmd`` 后让非操作臂跟随反馈，可把参数置为 ``true``
        （并照常点刷新初始值）。
        """
        _ = operating_side  # 保留签名便于调用方与未来扩展

        if len(base_hold) != len(JOINTS_WITH_SUFFIX):
            return list(base_hold)
        merge_non_op = bool(
            self.get_parameter("jog_merge_joint_states_into_non_operating_arm").value
        )
        if not merge_non_op:
            return list(base_hold)
        merged_fb = self._merge_joint_state_from_cache_and_last_cmd()
        if len(merged_fb) != len(JOINTS_WITH_SUFFIX):
            return list(base_hold)
        row = list(merged_fb)
        os = str(operating_side).strip().lower()
        if os == "left":
            sl = _LEFT_ARM_JOINT_INDICES
        elif os == "right":
            sl = _RIGHT_ARM_JOINT_INDICES
        else:
            return list(base_hold)
        for i in range(sl.start, sl.stop):
            row[i] = float(base_hold[i])
        return row

    def _arm_joint_positions_rad_from_row(
        self, arm_names: List[str], row: List[float]
    ) -> Dict[str, float]:
        """从即将下发的 UpperJointData 向量中取出操作臂各关节角（弧度）。"""
        out: Dict[str, float] = {}
        for jn in arm_names:
            ix = self._joint_index_by_urdf.get(jn)
            if ix is not None and 0 <= ix < len(row):
                out[jn] = float(row[ix])
        return out

    def adjust_arm_cartesian(
        self,
        side: str,
        axis: str,
        direction: Optional[str],
        step_mm: float,
    ) -> Tuple[bool, str, Optional[Dict[str, Any]]]:
        """
        单手臂沿 ``base_link`` 系某轴的 **笛卡尔直线示教**。

        前端 ``step_mm`` 单位毫米（如 1 表示 1 mm）；内部按 0.1 mm/路点展开得到一串
        直线路点，整条排入 100Hz 轨迹队列，由 :meth:`_on_timer_100hz` 依次发布——
        下游控制器每 10 ms 收到一个新目标，相邻路点之间在笛卡尔空间仅差 0.1 mm，
        宏观上末端沿直线移动，速度约 10 mm/s。
        """
        if self._joint_pub is None or UpperJointData is None:
            return False, "无法发布关节命令：UpperJointData 未加载（请安装 crb_ros_msg 并 source）", None
        side_l = str(side).strip().lower()
        if side_l not in ("left", "right"):
            return False, "side 应为 left 或 right", None
        axis_l = str(axis).strip().lower()
        if axis_l not in ("x", "y", "z"):
            return False, "axis 应为 x、y 或 z", None
        if direction not in ("+", "-"):
            return False, "direction 应为 '+' 或 '-'", None
        if not isinstance(step_mm, (int, float)) or step_mm <= 0.0 or step_mm > 1e4:
            return False, "步进值须为正数且合理（单位 mm）", None
        if not self._fk_ok or self._fk is None:
            return False, "URDF FK 未加载，无法做笛卡尔 IK", None

        # base_link 系下位移，单位 0.1 mm
        sgn = 1.0 if direction == "+" else -1.0
        mag_01mm = float(step_mm) * 10.0 * sgn
        dx_01mm = mag_01mm if axis_l == "x" else 0.0
        dy_01mm = mag_01mm if axis_l == "y" else 0.0
        dz_01mm = mag_01mm if axis_l == "z" else 0.0

        if side_l == "left":
            arm_names = self._left_arm_joint_names
            chain = self._fk._left_chain
            arm_slice = _LEFT_ARM_JOINT_INDICES
            arm_label = "左臂"
        else:
            arm_names = self._right_arm_joint_names
            chain = self._fk._right_chain
            arm_slice = _RIGHT_ARM_JOINT_INDICES
            arm_label = "右臂"

        if len(arm_names) != 7 or not chain:
            return False, f"{arm_label} URDF 关节链异常（未解析到 7 关节）", None

        # hold 基线（指令侧）；可选把腰与 /joint_states 对齐一次，再合成下发向量
        base_hold = self._build_hold_baseline_for_jog()
        base_hold = self._jog_hold_with_waist_optional_sync(base_hold)
        base_full = list(self._compose_jog_publish_baseline(base_hold, side_l))
        with self._fk_lock:
            cache = dict(self._joint_positions_by_name)
        self._sync_joint_states_into_base_full_except_operating_arm(base_full, side_l, cache)
        missing = [jn for jn in arm_names if jn not in cache]
        if missing:
            return False, f"joint_states 缺少{arm_label}关节: {missing}", None

        # 关键：操作臂的 "当前 q" 也从 hold 基线里取（= 控制器正在 hold 的点），
        # 而不是 /joint_states 缓存（= 被重力拉下的点）。
        # 这样 IK 输出的 q = hold 点 + 微小 Δq，下游跟踪只引入 Δq 对应的笛卡尔位移，
        # 不会因为基线切换额外引入 sag 位移。
        current_arm_q: List[float] = []
        for jn in arm_names:
            pub_idx = self._joint_index_by_urdf.get(jn)
            if pub_idx is not None and 0 <= pub_idx < len(base_full):
                current_arm_q.append(float(base_full[pub_idx]))
            else:
                current_arm_q.append(float(cache.get(jn, 0.0)))

        # 供 FK/Jacobian 使用的链上关节角（含 waist 等非臂段），也优先取 hold 基线
        chain_q_base: Dict[str, float] = {}
        for jn in chain:
            pub_idx = self._joint_index_by_urdf.get(jn)
            if pub_idx is not None and 0 <= pub_idx < len(base_full):
                chain_q_base[jn] = float(base_full[pub_idx])
            else:
                chain_q_base[jn] = float(cache.get(jn, 0.0))
        self._apply_live_waist_to_chain_q_base(chain, chain_q_base, cache)

        # 诊断：对比 hold 基线 vs 实时 cache，看 sag 有多大
        sag_deg: List[float] = []
        for k, jn in enumerate(arm_names):
            cache_v = float(cache.get(jn, current_arm_q[k]))
            sag_deg.append((cache_v - current_arm_q[k]) * 180.0 / 3.141592653589793)
        if any(abs(v) > 0.1 for v in sag_deg):
            self.get_logger().info(
                f"[IK] {arm_label} hold vs cache Δ(°): "
                + ", ".join(
                    f"{arm_names[k].replace('_joint','').replace(side_l + '_','')}={v:+.2f}"
                    for k, v in enumerate(sag_deg)
                )
                + "  (hold = 上次命令点；cache = /joint_states 实时反馈)"
            )

        # IK 单步长（0.1 mm 单位）。例如参数 2.0 mm → step_size_01mm=20
        step_size_01mm = max(1.0, float(self._jog_step_mm_per_waypoint) * 10.0)

        use_pin_ik = self._pin_ik_available_for_jog()
        pin_chain_q = _pin_seed_joint_dict(base_full, chain_q_base) if use_pin_ik else chain_q_base

        # ---- 诊断：FK on 当前 q，得到 base_link 系下当前末端位置 ----
        # Pin 示教时路点 ee 来自 Pin FK；若此处仍用 UrdfArmFk 作 p_start，两套 FK 偏差会导致
        # 「沿请求方向投影」≈0 而误报奇异/限位。
        try:
            seed = pin_chain_q if use_pin_ik else chain_q_base
            start_name_q = dict(seed)
            for k, jn in enumerate(arm_names):
                start_name_q[jn] = current_arm_q[k]
            p_start = None
            if use_pin_ik and self._kin_trac is not None:
                p_pin = self._kin_trac.fk_tip_xyz_trac(side_l, start_name_q)
                if p_pin is not None:
                    p_start = [float(p_pin[0]), float(p_pin[1]), float(p_pin[2])]
            if p_start is None:
                ps = self._fk.fk_tip_xyz(chain, start_name_q)
                if ps is None:
                    return False, f"{arm_label} 起始 FK 返回 None（URDF 链异常）", None
                p_start = [float(ps[0]), float(ps[1]), float(ps[2])]
        except Exception as e:
            self.get_logger().error(f"{arm_label} 起始 FK 失败: {e}")
            return False, f"起始 FK 失败: {e}", None

        self.get_logger().info(
            f"[IK] {arm_label} start axis={axis_l} dir={direction} step={step_mm}mm "
            f"→ base_link 位移请求 dx={dx_01mm/10:+.2f} dy={dy_01mm/10:+.2f} dz={dz_01mm/10:+.2f} mm"
        )
        self.get_logger().info(
            f"[IK] {arm_label} 起始末端(base_link): x={p_start[0]:+.4f} y={p_start[1]:+.4f} z={p_start[2]:+.4f} m"
        )
        self.get_logger().info(
            f"[IK] {arm_label} 起始 q(°): "
            + ", ".join(
                f"{jn.replace('_joint','').replace(side_l + '_','')}={q*180.0/3.141592653589793:+.2f}"
                for jn, q in zip(arm_names, current_arm_q)
            )
        )

        try:
            if use_pin_ik:
                waypoints = cartesian_linear_waypoints_trac(
                    kin=self._kin_trac,
                    side=side_l,
                    chain_base_to_tip=chain,
                    arm_joint_names=arm_names,
                    chain_joint_q_base=pin_chain_q,
                    current_arm_q_rad=current_arm_q,
                    dx_01mm=dx_01mm,
                    dy_01mm=dy_01mm,
                    dz_01mm=dz_01mm,
                    joint_limits=self._arm_joint_limits,
                    step_size_01mm=step_size_01mm,
                )
            else:
                waypoints = cartesian_linear_waypoints(
                    fk=self._fk,
                    chain_base_to_tip=chain,
                    arm_joint_names=arm_names,
                    chain_joint_q_base=chain_q_base,
                    current_arm_q_rad=current_arm_q,
                    dx_01mm=dx_01mm,
                    dy_01mm=dy_01mm,
                    dz_01mm=dz_01mm,
                    joint_limits=self._arm_joint_limits,
                    step_size_01mm=step_size_01mm,
                    damping=0.005,
                )
        except Exception as e:
            self.get_logger().error(f"{arm_label}笛卡尔 IK 失败: {e}")
            return False, f"笛卡尔 IK 失败: {e}", None

        if not waypoints:
            return False, "位移为零或 IK 无有效步", None

        # ---- 诊断：IK 结束后理论末端位置 + 各关节累计 Δq ----
        last_q = [float(waypoints[-1]["joint_angles_rad"].get(jn, current_arm_q[k])) for k, jn in enumerate(arm_names)]
        total_dq_deg = [(last_q[k] - current_arm_q[k]) * 180.0 / 3.141592653589793 for k in range(7)]
        last_ee = waypoints[-1]["ee_position_m"]
        # 用路点内记录的末端（与 IK 循环内 Pin FK 一致）减 p_start；勿再用 end_name_q 重算 pe，
        # 否则 pin_chain_q 与路点内 merged 在手指/头等细节上难完全一致，易出现数百毫米假 Δ 与误拒绝。
        achieved = (
            float(last_ee["x"]) - float(p_start[0]),
            float(last_ee["y"]) - float(p_start[1]),
            float(last_ee["z"]) - float(p_start[2]),
        )
        requested = (dx_01mm * 1e-4, dy_01mm * 1e-4, dz_01mm * 1e-4)
        req_norm = max(1e-9, (requested[0] ** 2 + requested[1] ** 2 + requested[2] ** 2) ** 0.5)
        # 请求方向上的投影（正=同向，负=反向）
        proj = (
            achieved[0] * requested[0]
            + achieved[1] * requested[1]
            + achieved[2] * requested[2]
        ) / req_norm

        self.get_logger().info(
            f"[IK] {arm_label} 理论末端(IK末): x={last_ee['x']:+.4f} y={last_ee['y']:+.4f} z={last_ee['z']:+.4f} m "
            f"| 实际 Δee = ({achieved[0]*1000:+.2f}, {achieved[1]*1000:+.2f}, {achieved[2]*1000:+.2f}) mm, "
            f"沿请求方向投影 = {proj*1000:+.2f} mm / 请求 {req_norm*1000:+.2f} mm"
        )
        self.get_logger().info(
            f"[IK] {arm_label} 各关节 Δq(°): "
            + ", ".join(
                f"{jn.replace('_joint','').replace(side_l + '_','')}={total_dq_deg[k]:+.3f}"
                for k, jn in enumerate(arm_names)
            )
        )
        # 检查是否撞限位（以及起始就已在限位的"粘住"关节）
        clipped: List[str] = []
        stuck_at_start: List[str] = []
        for k, jn in enumerate(arm_names):
            lo, hi = self._arm_joint_limits.get(jn, (-3.1416, 3.1416))
            if abs(last_q[k] - lo) < 1e-6 or abs(last_q[k] - hi) < 1e-6:
                clipped.append(f"{jn.replace('_joint','')}({last_q[k]*180/3.141592653589793:+.1f}°)")
            if abs(current_arm_q[k] - lo) < 1e-3 or abs(current_arm_q[k] - hi) < 1e-3:
                stuck_at_start.append(f"{jn.replace('_joint','')}({current_arm_q[k]*180/3.141592653589793:+.1f}°)")
        if clipped:
            self.get_logger().warn(f"[IK] {arm_label} 触碰关节限位: {clipped}")
        if stuck_at_start:
            self.get_logger().warn(
                f"[IK] {arm_label} 起始时已贴近限位: {stuck_at_start} "
                f"— 该方向可能无法移动，建议先把手臂摆离限位（例如肘弯曲）后再示教"
            )

        # ---- 关键保护：若 IK 能达成的位移远小于请求，则拒绝发轨迹 ----
        # 原因：肘/肩若撞上限位，IK 会产生"几乎原地"的轨迹，配合下游重力补偿不到位，
        # 真机会看上去"无论哪个方向都往下沉"。此时直接拒绝更安全。
        min_ratio = 0.5
        if proj < req_norm * min_ratio:
            reason = (
                f"IK 只能沿请求方向达成 {proj*1000:+.2f} mm / {req_norm*1000:.2f} mm "
                f"({proj/req_norm*100:.1f}%)"
            )
            if stuck_at_start:
                reason += f"；起始已贴近限位：{stuck_at_start}"
            reason += "。请先让手臂离开当前奇异/限位姿态（例如肘弯曲 ~90°）再示教。"
            self.get_logger().error(f"[IK] {arm_label} 拒绝下发: {reason}")
            return False, reason, None

        # base_full 已在本函数开头用 _build_hold_baseline_for_jog() 构造，
        # 此处不再从 cache 重取；仅操作臂的 7 关节会按 IK waypoint 覆盖

        # 为让下游控制器有足够时间跟踪，每个 IK 路点在 100Hz 定时器里重复 N 次
        #   Cartesian 速度 = step_mm / (N × 10ms)
        ticks_per_wp = self._jog_ticks_per_waypoint
        jog_rows: List[List[float]] = []
        arm_idx_in_pub = list(range(arm_slice.start, arm_slice.stop))
        wi_pub, w_rad_pub = self._jog_publish_waist_index_and_rad(chain_q_base, base_full)
        for wp in waypoints:
            row = list(base_full)
            for k, jn in enumerate(arm_names):
                val = wp["joint_angles_rad"].get(jn)
                if val is None:
                    continue
                pub_idx = self._joint_index_by_urdf.get(jn)
                if pub_idx is None and k < len(arm_idx_in_pub):
                    pub_idx = arm_idx_in_pub[k]
                if pub_idx is not None:
                    row[pub_idx] = float(val)
            self._finalize_jog_publish_row(row, base_full, arm_slice, wi_pub, w_rad_pub)
            for _ in range(ticks_per_wp):
                jog_rows.append(list(row))

        with self._traj_lock:
            if self._playing:
                return False, "有轨迹正在播放，请稍后再试", None
            self._setup_jog_fk_display_overrides(side_l, base_full, chain_q_base)
            self._traj_rows = jog_rows
            self._traj_index = 0
            self._playing = True
            self._traj_mode = "jog"

        duration_s = len(jog_rows) * 0.01
        cartesian_speed_mms = (
            self._jog_step_mm_per_waypoint / (ticks_per_wp * 0.01)
        )
        self.get_logger().info(
            f"[IK] {arm_label} 排入 {len(waypoints)} 个路点 × {ticks_per_wp} 次重复 "
            f"= {len(jog_rows)} 帧，~{duration_s:.2f}s，笛卡尔速度≈{cartesian_speed_mms:.1f} mm/s"
        )
        msg = (
            f"{arm_label}沿 {axis_l.upper()}{direction}{step_mm}mm 直线示教："
            f"{len(waypoints)}路点×{ticks_per_wp}重复，~{duration_s:.2f}s"
        )
        return (
            True,
            msg,
            {
                "waypoints": len(waypoints),
                "frames": len(jog_rows),
                "duration_s": duration_s,
                "target_ee_m": last_ee,
                "joint_cmd_arm_last_rad": self._arm_joint_positions_rad_from_row(
                    arm_names, jog_rows[-1]
                ),
            },
        )

    def adjust_arm_cartesian_rotate(
        self,
        side: str,
        axis: str,
        direction: Optional[str],
        step_deg: float,
    ) -> Tuple[bool, str, Optional[Dict[str, Any]]]:
        """
        单手臂绕 ``base_link`` 的 RX/RY/RZ **定轴姿态示教**（腕部连杆原点位置尽量保持）。

        ``step_deg`` 为本次总转角（度），范围 0.1°～10.0°（测试用上限）；内部分子步不超过
        ``jog_rot_step_deg_per_waypoint``（默认 0.1°），轨迹排入 100Hz 队列，与直线示教相同播放方式。
        """
        if self._joint_pub is None or UpperJointData is None:
            return False, "无法发布关节命令：UpperJointData 未加载（请安装 crb_ros_msg 并 source）", None
        side_l = str(side).strip().lower()
        if side_l not in ("left", "right"):
            return False, "side 应为 left 或 right", None
        axis_l = str(axis).strip().lower()
        if axis_l not in ("rx", "ry", "rz"):
            return False, "axis 应为 rx、ry 或 rz", None
        if direction not in ("+", "-"):
            return False, "direction 应为 '+' 或 '-'", None
        try:
            step_v = float(step_deg)
        except (TypeError, ValueError):
            return False, "step 须为数值（度）", None
        step_q = round(step_v * 10.0) / 10.0
        if step_q < 0.1 - 1e-9 or step_q > 10.0 + 1e-9:
            return False, "角度步进须在 0.1°～10.0° 之间（按 0.1° 刻度）", None
        if not self._fk_ok or self._fk is None:
            return False, "URDF FK 未加载，无法做笛卡尔 IK", None

        sgn = 1.0 if direction == "+" else -1.0
        total_angle_rad = math.radians(step_q) * sgn
        step_angle_rad = math.radians(float(self._jog_rot_step_deg_per_waypoint))

        if side_l == "left":
            arm_names = self._left_arm_joint_names
            chain = self._fk._left_chain
            arm_slice = _LEFT_ARM_JOINT_INDICES
            arm_label = "左臂"
        else:
            arm_names = self._right_arm_joint_names
            chain = self._fk._right_chain
            arm_slice = _RIGHT_ARM_JOINT_INDICES
            arm_label = "右臂"

        if len(arm_names) != 7 or not chain:
            return False, f"{arm_label} URDF 关节链异常（未解析到 7 关节）", None

        base_hold = self._build_hold_baseline_for_jog()
        base_hold = self._jog_hold_with_waist_optional_sync(base_hold)
        base_full = list(self._compose_jog_publish_baseline(base_hold, side_l))
        with self._fk_lock:
            cache = dict(self._joint_positions_by_name)
        self._sync_joint_states_into_base_full_except_operating_arm(base_full, side_l, cache)
        missing = [jn for jn in arm_names if jn not in cache]
        if missing:
            return False, f"joint_states 缺少{arm_label}关节: {missing}", None

        current_arm_q: List[float] = []
        for jn in arm_names:
            pub_idx = self._joint_index_by_urdf.get(jn)
            if pub_idx is not None and 0 <= pub_idx < len(base_full):
                current_arm_q.append(float(base_full[pub_idx]))
            else:
                current_arm_q.append(float(cache.get(jn, 0.0)))

        chain_q_base: Dict[str, float] = {}
        for jn in chain:
            pub_idx = self._joint_index_by_urdf.get(jn)
            if pub_idx is not None and 0 <= pub_idx < len(base_full):
                chain_q_base[jn] = float(base_full[pub_idx])
            else:
                chain_q_base[jn] = float(cache.get(jn, 0.0))
        self._apply_live_waist_to_chain_q_base(chain, chain_q_base, cache)

        use_pin_ik = self._pin_ik_available_for_jog()
        pin_chain_q = _pin_seed_joint_dict(base_full, chain_q_base) if use_pin_ik else chain_q_base

        try:
            start_name_q = dict(chain_q_base)
            for k, jn in enumerate(arm_names):
                start_name_q[jn] = current_arm_q[k]
            T_start = self._fk.fk_chain_transform(chain, start_name_q)
            p_start = self._fk.fk_tip_xyz(chain, start_name_q)
        except Exception as e:
            self.get_logger().error(f"{arm_label} 起始 FK 失败: {e}")
            return False, f"起始 FK 失败: {e}", None
        if p_start is None:
            return False, f"{arm_label} 起始 FK 返回 None（URDF 链异常）", None
        R_start = np.asarray(T_start[:3, :3], dtype=float)

        u = _rot_unit_axis(axis_l)
        self.get_logger().info(
            f"[IK] {arm_label} 姿态示教 axis={axis_l} dir={direction} step={step_q}° "
            f"→ Δθ={total_angle_rad:+.5f} rad, 子步≤{self._jog_rot_step_deg_per_waypoint}°"
        )

        try:
            if use_pin_ik:
                waypoints = cartesian_rotate_waypoints_trac(
                    kin=self._kin_trac,
                    side=side_l,
                    chain_base_to_tip=chain,
                    arm_joint_names=arm_names,
                    chain_joint_q_base=pin_chain_q,
                    current_arm_q_rad=current_arm_q,
                    axis=axis_l,
                    total_angle_rad=total_angle_rad,
                    joint_limits=self._arm_joint_limits,
                    step_angle_rad=step_angle_rad,
                )
            else:
                waypoints = cartesian_rotate_waypoints(
                    fk=self._fk,
                    chain_base_to_tip=chain,
                    arm_joint_names=arm_names,
                    chain_joint_q_base=chain_q_base,
                    current_arm_q_rad=current_arm_q,
                    axis=axis_l,
                    total_angle_rad=total_angle_rad,
                    joint_limits=self._arm_joint_limits,
                    step_angle_rad=step_angle_rad,
                    damping=0.005,
                )
        except Exception as e:
            self.get_logger().error(f"{arm_label} 姿态 IK 失败: {e}")
            return False, f"姿态 IK 失败: {e}", None

        if not waypoints:
            return False, "转角为零或 IK 无有效步", None

        last_q = [float(waypoints[-1]["joint_angles_rad"].get(jn, current_arm_q[k])) for k, jn in enumerate(arm_names)]
        last_ee = waypoints[-1]["ee_position_m"]
        end_name_q = dict(pin_chain_q if use_pin_ik else chain_q_base)
        for k, jn in enumerate(arm_names):
            end_name_q[jn] = last_q[k]
        T_end = None
        if self._kin_trac is not None and self._kin_trac.available:
            T_end = self._kin_trac.fk_tip_T(side_l, end_name_q)
        if T_end is None:
            T_end = self._fk.fk_chain_transform(chain, end_name_q)
        if T_end is None:
            return False, f"{arm_label} 末端姿态校验时 FK 返回 None", None
        R_end = np.asarray(T_end[:3, :3], dtype=float)
        R_rel = R_end @ R_start.T
        w_ach = _vee_log_so3(R_rel)
        proj = float(np.dot(w_ach, u))
        req_mag = abs(total_angle_rad)
        min_ratio = 0.5
        if req_mag > 1e-9:
            if proj * total_angle_rad < 0.0 or abs(proj) < req_mag * min_ratio:
                reason = (
                    f"绕 {axis_l.upper()} 实际达成转角分量约 {math.degrees(proj):+.2f}° / "
                    f"请求 {math.degrees(total_angle_rad):+.2f}°（<{min_ratio * 100:.0f}%）"
                )
                reason += "。请先让手臂离开奇异/限位姿态再试。"
                self.get_logger().error(f"[IK] {arm_label} 拒绝下发: {reason}")
                return False, reason, None

        pos_drift = (
            last_ee["x"] - float(p_start[0]),
            last_ee["y"] - float(p_start[1]),
            last_ee["z"] - float(p_start[2]),
        )
        drift_mm = (pos_drift[0] ** 2 + pos_drift[1] ** 2 + pos_drift[2] ** 2) ** 0.5 * 1000.0
        self.get_logger().info(
            f"[IK] {arm_label} 姿态示教末位置漂移 |Δp|={drift_mm:.2f} mm "
            f"(保持原点近似不变；冗余臂会有少量位移)"
        )

        ticks_per_wp = self._jog_ticks_per_waypoint
        jog_rows: List[List[float]] = []
        arm_idx_in_pub = list(range(arm_slice.start, arm_slice.stop))
        wi_pub, w_rad_pub = self._jog_publish_waist_index_and_rad(chain_q_base, base_full)
        for wp in waypoints:
            row = list(base_full)
            for k, jn in enumerate(arm_names):
                val = wp["joint_angles_rad"].get(jn)
                if val is None:
                    continue
                pub_idx = self._joint_index_by_urdf.get(jn)
                if pub_idx is None and k < len(arm_idx_in_pub):
                    pub_idx = arm_idx_in_pub[k]
                if pub_idx is not None:
                    row[pub_idx] = float(val)
            self._finalize_jog_publish_row(row, base_full, arm_slice, wi_pub, w_rad_pub)
            for _ in range(ticks_per_wp):
                jog_rows.append(list(row))

        with self._traj_lock:
            if self._playing:
                return False, "有轨迹正在播放，请稍后再试", None
            self._setup_jog_fk_display_overrides(side_l, base_full, chain_q_base)
            self._traj_rows = jog_rows
            self._traj_index = 0
            self._playing = True
            self._traj_mode = "jog"

        duration_s = len(jog_rows) * 0.01
        msg = (
            f"{arm_label}绕 {axis_l.upper()}{direction}{step_q}° 姿态示教："
            f"{len(waypoints)}路点×{ticks_per_wp}重复，~{duration_s:.2f}s"
        )
        return (
            True,
            msg,
            {
                "waypoints": len(waypoints),
                "frames": len(jog_rows),
                "duration_s": duration_s,
                "target_ee_m": last_ee,
                "joint_cmd_arm_last_rad": self._arm_joint_positions_rad_from_row(
                    arm_names, jog_rows[-1]
                ),
            },
        )

    def enable_upper_body_debug(self, wait_srv_sec: float = 2.0, call_timeout_sec: float = 5.0) -> Tuple[bool, str, bool]:
        """
        在需要时将 ``/motion/upper_body_debug`` 置为 true。

        若本节点已记录调试为开启（例如用户此前已点「开始标定」并调过该服务），则**不再重复**
        调用服务，返回 ``(True, message, True)`` 第三项 ``skipped_service_call``。

        Returns:
            ``(success, message, skipped_service_call)``；失败时第三项恒为 ``False``。
        """
        if self._upper_body_debug_enabled_local:
            return (
                True,
                "本节点已记录上半身调试为开启，跳过重复调用 /motion/upper_body_debug(true)",
                True,
            )
        cli = getattr(self, "_debug_cli", None)
        if cli is None:
            return False, "服务客户端未初始化", False
        if not cli.wait_for_service(timeout_sec=float(wait_srv_sec)):
            return False, "服务 /motion/upper_body_debug 不可用", False
        req = SetBool.Request()
        req.data = True
        fut = cli.call_async(req)
        t0 = time.time()
        while rclpy.ok() and not fut.done() and (time.time() - t0) < float(call_timeout_sec):
            time.sleep(0.01)
        if not fut.done():
            return False, "调用上身调试服务超时", False
        try:
            resp = fut.result()
        except Exception as ex:
            return False, str(ex), False
        if resp is None:
            return False, "服务无响应", False
        if not resp.success:
            return False, str(resp.message or "服务返回失败"), False
        self._upper_body_debug_enabled_local = True
        return True, str(resp.message or "ok"), False

    def _write_initial_ee_rpy_from_current_display_locked(self) -> None:
        """在已持有 ``_fk_lock`` 时，把界面当前左右腕末端位姿写入双臂初始值（与末帧标定快照数值一致）。"""
        self._initial_ee_left = (
            float(self._ee_left[0]),
            float(self._ee_left[1]),
            float(self._ee_left[2]),
        )
        self._initial_ee_right = (
            float(self._ee_right[0]),
            float(self._ee_right[1]),
            float(self._ee_right[2]),
        )
        self._initial_rpy_left = (
            float(self._ee_left_rpy[0]),
            float(self._ee_left_rpy[1]),
            float(self._ee_left_rpy[2]),
        )
        self._initial_rpy_right = (
            float(self._ee_right_rpy[0]),
            float(self._ee_right_rpy[1]),
            float(self._ee_right_rpy[2]),
        )

    def refresh_initial_ee(
        self, side: str
    ) -> Tuple[bool, str, Optional[Dict[str, Any]]]:
        """
        用 ``/joint_states`` 与当前界面 FK 更新 **左右臂**「初始值」，**覆盖**此前「开始标定」
        末帧写入的初值；并把 **以关节反馈为主** 的全身向量写入 ``_last_joint_cmd_positions``，
        供后续单臂笛卡尔示教作 hold 起点（与仅播放标定末帧后示教等价）。

        左/右面板「刷新初始值」任一按钮均执行上述双臂初值同步。

        流程：通过检查后可选 ``/motion/upper_body_debug(true)`` → 写 ``_last_joint_cmd`` 为
        baseline → 可选下发同内容 joint_cmd hold → **等待** ``refresh_initial_ee_resample_sec``
        （默认 **3s**）让关节反馈稳定 → 再按此时界面左右腕 FK 写入初值。避免「先写初值再发
        hold」或控制器未跟上时，示教后出现约数毫米假偏移。

        单臂示教时 ``_sync_joint_states_into_base_full_except_operating_arm`` 对 **头/腰/对侧臂**
        保持 hold，左/右面板分别只驱动对应侧腕部运动（与标定末帧后行为一致）。
        """
        if not self._fk_ok or self._fk is None:
            return False, "URDF FK 未加载，无法刷新初始值", None
        side_l = str(side).strip().lower()
        if side_l not in ("left", "right"):
            return False, "side 应为 left 或 right", None
        with self._traj_lock:
            busy = self._playing
        if busy:
            return False, "有轨迹或示教正在执行，请等待结束后再刷新初始值", None

        with self._fk_lock:
            ready = self._fk_pose_ready
            cache = dict(self._joint_positions_by_name)
        if not ready:
            return False, "尚未收到 /joint_states 或 FK 未就绪，请稍后再试", None
        need_arms = list(self._left_arm_joint_names) + list(self._right_arm_joint_names)
        missing_fb = [jn for jn in need_arms if jn not in cache]
        if missing_fb:
            sample = ", ".join(missing_fb[:6])
            if len(missing_fb) > 6:
                sample += ", …"
            return (
                False,
                "双臂关节尚未全部出现在 /joint_states 中，无法以当前关节状态刷新示教基准：缺失 "
                f"{sample}",
                None,
            )

        dbg: Optional[Dict[str, Any]] = None
        if bool(self.get_parameter("refresh_initial_auto_enable_upper_body_debug").value):
            ok_dbg, msg_dbg, skipped_dbg = self.enable_upper_body_debug()
            dbg = {
                "ok": ok_dbg,
                "message": msg_dbg,
                "skipped_service_call": skipped_dbg,
            }
            if not ok_dbg:
                return (
                    False,
                    "无法开启「上半身调试模式」，已取消刷新（与开始标定前置条件一致；"
                    "无该服务时可把参数 refresh_initial_auto_enable_upper_body_debug 设为 false）。"
                    f" 详情：{msg_dbg}",
                    None,
                )

        baseline, n_js, n_fb = self._baseline_vector_joint_states_then_last_cmd()
        n_merge = max(0, int(self.get_parameter("jog_hold_baseline_feedback_merges_after_refresh").value))
        with self._fk_lock:
            if not (self._fk_pose_ready and self._fk_ok):
                return False, "刷新瞬间末端位姿未就绪，请稍后再试", None
            self._last_joint_cmd_positions = list(baseline)
            self._hold_baseline_feedback_merges_remaining = n_merge

        pub_hold = False
        pub_n = 0
        if bool(self.get_parameter("refresh_initial_publish_hold_baseline").value):
            if self._joint_pub is not None and UpperJointData is not None:
                try:
                    pub_n = max(1, int(self.get_parameter("refresh_initial_publish_hold_repeat_count").value))
                except Exception:
                    pub_n = 1
                for ri in range(pub_n):
                    self._publish_joint_cmd(list(baseline))
                    if ri + 1 < pub_n:
                        time.sleep(0.01)
                pub_hold = True
                self.get_logger().info(
                    f"[Web] 刷新后已向 joint_cmd 下发 {pub_n} 帧全身 hold，与示教基线一致"
                )
            else:
                self.get_logger().warn(
                    "[Web] refresh_initial_publish_hold_baseline=true 但未下发：joint_cmd 发布器不可用"
                )

        try:
            resample = float(self.get_parameter("refresh_initial_ee_resample_sec").value)
        except Exception:
            resample = 3.0
        resample = max(0.0, min(resample, 10.0))
        if pub_hold and resample <= 1e-9:
            self.get_logger().warn(
                "[Web] 已下发 hold 但 refresh_initial_ee_resample_sec≈0："
                "初值可能与关节反馈不一致，建议默认 3s 或 0.5～5.0"
            )
        if resample > 0.0:
            time.sleep(resample)
        if not rclpy.ok():
            return False, "刷新过程中节点已关闭", None

        with self._fk_lock:
            if not (self._fk_pose_ready and self._fk_ok):
                return False, "刷新后采样瞬间末端位姿未就绪，请稍后再试", None
            self._write_initial_ee_rpy_from_current_display_locked()
            el, er = self._initial_ee_left, self._initial_ee_right
            rl, rr = self._initial_rpy_left, self._initial_rpy_right

        if el is None or er is None or rl is None or rr is None:
            return False, "刷新后初值为空，请确认末端 FK 与 /joint_states 正常", None
        label = "左臂" if side_l == "left" else "右臂"
        deg = 180.0 / math.pi
        ee_sel, rpy_sel = (el, rl) if side_l == "left" else (er, rr)
        self.get_logger().info(
            f"[Web] 经{label}面板刷新双臂初始值（hold 后等待 {resample:.3f}s 再采样，已覆盖标定写入的初值）。"
            f"左腕 m x={el[0]:+.6f} y={el[1]:+.6f} z={el[2]:+.6f}；"
            f"右腕 m x={er[0]:+.6f} y={er[1]:+.6f} z={er[2]:+.6f}。"
            f" hold：{n_js} 关节来自 /joint_states，{n_fb} 由历史指令或缺省补齐；"
            + (
                f"后续 {n_merge} 次示教每次先 merge 反馈再 hold。"
                if n_merge > 0
                else "后续示教直接使用本次 hold（左/右面板分别只动对应侧腕）。"
            )
        )
        return (
            True,
            "左右臂初始值已刷新：已按当前关节状态覆盖标定初值，hold 后等待再采样，可与「开始标定」末帧后同样单臂示教",
            {
                "trigger_side": side_l,
                "upper_body_debug": dbg
                if dbg is not None
                else {"skipped": True, "message": "未自动开启（refresh_initial_auto_enable_upper_body_debug=false）"},
                "initial_ee_m": {
                    "x": float(ee_sel[0]),
                    "y": float(ee_sel[1]),
                    "z": float(ee_sel[2]),
                },
                "initial_rpy_deg": {
                    "rx": float(rpy_sel[0] * deg),
                    "ry": float(rpy_sel[1] * deg),
                    "rz": float(rpy_sel[2] * deg),
                },
                "left": {
                    "initial_ee_m": {"x": float(el[0]), "y": float(el[1]), "z": float(el[2])},
                    "initial_rpy_deg": {
                        "rx": float(rl[0] * deg),
                        "ry": float(rl[1] * deg),
                        "rz": float(rl[2] * deg),
                    },
                },
                "right": {
                    "initial_ee_m": {"x": float(er[0]), "y": float(er[1]), "z": float(er[2])},
                    "initial_rpy_deg": {
                        "rx": float(rr[0] * deg),
                        "ry": float(rr[1] * deg),
                        "rz": float(rr[2] * deg),
                    },
                },
                "hold_baseline": {
                    "joints_from_joint_states": int(n_js),
                    "joints_fallback": int(n_fb),
                    "followup_merged_jogs": int(n_merge),
                },
                "published_hold_baseline": pub_hold,
                "published_hold_frames": int(pub_n) if pub_hold else 0,
                "initial_ee_resample_sec": float(resample),
            },
        )

    def _snapshot_initial_ee_from_current_display(self) -> bool:
        """
        在 ``_fk_lock`` 内把 ``_ee_left`` / ``_ee_right`` 及 RPY 直接拷入初始值。

        与「用关节缓存再算一遍 FK」相比，数值与界面「当前」完全一致，避免构链顺序/缺省 0 与
        :meth:`_on_joint_states` 路径有细微差导致约数毫米假偏移。
        """
        with self._fk_lock:
            if not (self._fk_pose_ready and self._fk_ok):
                return False
            self._write_initial_ee_rpy_from_current_display_locked()
        return True

    def _schedule_upper_body_debug_off_after_band_play(self) -> None:
        """乐队顺序播放结束后自动关闭上半身调试（异步，避免阻塞 100Hz 定时器）。"""

        def wait_fut(fut, timeout_sec: float = 5.0) -> bool:
            t0 = time.time()
            while rclpy.ok() and not fut.done() and (time.time() - t0) < timeout_sec:
                time.sleep(0.01)
            return fut.done()

        def run() -> None:
            if not rclpy.ok():
                return
            if not self._debug_cli.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn(
                    "[band] 播放结束：/motion/upper_body_debug 不可用，未自动关闭调试"
                )
                return
            req = SetBool.Request()
            req.data = False
            fut = self._debug_cli.call_async(req)
            if not wait_fut(fut, timeout_sec=5.0):
                self.get_logger().warn("[band] 播放结束：关闭上半身调试调用超时")
                return
            try:
                resp = fut.result()
            except Exception as ex:
                self.get_logger().warn(f"[band] 播放结束：关闭上半身调试异常: {ex}")
                return
            if resp and resp.success:
                self._upper_body_debug_enabled_local = False
                self.get_logger().info("[band] 顺序播放已结束，已自动关闭上半身调试模式")
            else:
                self.get_logger().warn(
                    f"[band] 播放结束：关闭上半身调试失败: "
                    f"{getattr(resp, 'message', '') or 'unknown'}"
                )

        threading.Thread(target=run, daemon=True).start()

    def _on_timer_100hz(self) -> None:
        row: Optional[List[float]] = None
        finished_start_playback = False
        finished_band_playback = False
        with self._traj_lock:
            if not self._playing or not self._traj_rows:
                return
            if self._traj_index >= len(self._traj_rows):
                self._playing = False
                self._traj_mode = ""
                self._clear_jog_fk_display_override()
                return
            row = self._traj_rows[self._traj_index]
            self._traj_index += 1
            if self._traj_index >= len(self._traj_rows):
                self._playing = False
                if self._traj_mode == "start":
                    finished_start_playback = True
                elif self._traj_mode == "band_play":
                    finished_band_playback = True
                self._traj_mode = ""
                self._clear_jog_fk_display_override()
        if row is not None:
            self._publish_joint_cmd(row)
        if finished_start_playback:

            def settle_then_snapshot_initial_from_joint_states() -> None:
                try:
                    d = float(self.get_parameter("calibration_initial_ee_joint_resample_sec").value)
                except Exception:
                    d = 3.0
                d = max(0.0, min(d, 5.0))
                if d > 0.0:
                    time.sleep(d)
                if not rclpy.ok():
                    return
                if self._snapshot_initial_ee_from_current_display():
                    self.get_logger().info(
                        f"开始标定已播完：已等待 {d:.3f}s 后将左右腕「当前」末端位姿拷入初始值 "
                        "（与 /api/state 当前值数值一致）"
                    )
                else:
                    self.get_logger().warn(
                        "开始标定已播完：等待后写入初始腕位姿失败（请检查 joint_states 与 URDF）"
                    )

            threading.Thread(target=settle_then_snapshot_initial_from_joint_states, daemon=True).start()
        if finished_band_playback:
            self._schedule_upper_body_debug_off_after_band_play()


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node = ArmCalibrationWebNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
