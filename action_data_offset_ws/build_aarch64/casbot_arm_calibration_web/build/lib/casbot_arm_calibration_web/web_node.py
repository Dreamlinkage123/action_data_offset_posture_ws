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
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import SetBool

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
    list_action_data_basenames,
    list_offset_data_relative_paths,
    new_offset_data_write_targets,
    offset_data_root,
    resolve_trajectory_path,
)

# 前端 value -> resource 子目录名（与文件名 keyboard_start_calibration.data 等一致）
INSTRUMENT_TO_SUBDIR = {
    "drum": "drum",
    "bass": "bass",
    "guitar": "guitar",
    "keyboard": "keyboard",
}

# JOINTS_WITH_SUFFIX 中左/右臂 7 关节下标（头、腰之后，灵巧手之前）
_LEFT_ARM_JOINT_INDICES = slice(3, 10)
_RIGHT_ARM_JOINT_INDICES = slice(10, 17)

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
        """把当前末端 FK 位置写入左/右臂的「初始值」。"""
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
        # 空字符串：使用 share/casbot_arm_calibration_web/urdf/<默认 .urdf>（见 _package_urdf_dir）
        self.declare_parameter("robot_urdf_path", "")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("use_trac_ik", True)
        # 单次示教发布到 /upper_body_debug/joint_cmd 时的 time_ref（秒）
        self.declare_parameter("joint_cmd_time_ref", 1.0)
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
        # 「开始标定」轨迹播放结束并延迟后写入，供界面「初始值」；None 表示尚未采样
        self._initial_ee_left: Optional[Tuple[float, float, float]] = None
        self._initial_ee_right: Optional[Tuple[float, float, float]] = None
        self._initial_rpy_left: Optional[Tuple[float, float, float]] = None
        self._initial_rpy_right: Optional[Tuple[float, float, float]] = None
        self._initial_capture_seq = 0
        # 上次发布的全身关节指令；joint_states 缺某臂关节时用它回退，避免误发 0 导致另一臂被拉向零位
        self._last_joint_cmd_positions: Optional[List[float]] = None
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
        if self._fk_ok and self._fk is not None and self._use_trac_ik:
            self._kin_trac = KinematicsPinTrac(
                urdf_path,
                list(self._fk._left_chain),
                list(self._fk._right_chain),
            )
            if self._kin_trac.available:
                if self._kin_trac.pin_ok:
                    self.get_logger().info(
                        "[Kin] 示教 IK：TRAC‑IK；关节状态 FK：Pinocchio（失败时回退 tracikpy.fk）"
                    )
                else:
                    self.get_logger().info(
                        "[Kin] 示教 IK / 关节状态 FK：tracikpy（Pinocchio 未启用或不可用）"
                    )
            else:
                self.get_logger().warn(
                    f"[Kin] TRAC‑IK 未启用: {self._kin_trac.load_error}；示教仍用数值 DLS"
                )
        elif not self._use_trac_ik:
            self.get_logger().info("[Kin] 参数 use_trac_ik=false，示教使用数值 DLS IK")

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

        if UpperJointData is not None:
            self._joint_pub = self.create_publisher(
                UpperJointData, "/upper_body_debug/joint_cmd", 10
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

    def _on_joint_states(self, msg: JointState) -> None:
        if not msg.name or not msg.position or len(msg.name) != len(msg.position):
            return
        with self._fk_lock:
            for i, n in enumerate(msg.name):
                if i >= len(msg.position):
                    break
                key = n if str(n).endswith("_joint") else f"{n}_joint"
                self._joint_positions_by_name[key] = float(msg.position[i])
        if not self._fk_ok:
            return
        try:
            fk = self._fk
            q_l = build_q_map_for_chain(fk._left_chain, msg.name, msg.position)
            q_r = build_q_map_for_chain(fk._right_chain, msg.name, msg.position)
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
        初始值来自开始标定播完约 1s 后的采样，或用户点击「刷新初始值」。
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
                "左右臂「初始值」未设置：请先播放「开始标定」并等待约 1s，或分别点「刷新初始值」",
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
                "左右臂「初始值」未设置：请先播放「开始标定」并等待约 1s，或分别点「刷新初始值」",
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
        """供 /api/state：位置为 base_link 下腕部原点 (x,y,z)（米）；姿态 RX/RY/RZ 的初始/当前值为
        identity→该姿态的旋转向量分量（度）；**偏移量**为 R_cur·R_init^T 的相对旋转向量分量（度），
        与绕固定轴示教一致；因旋转不可按分量相加，偏移一般≠当前−初始。
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

        def fmt(v: float) -> str:
            return f"{v:.6f}"

        def axis_row(
            cur: float,
            initial_comp: Optional[float],
        ) -> Dict[str, Any]:
            if initial_comp is None:
                return {**axis_base, "current": fmt(cur), "initial": dash, "delta": dash}
            return {
                **axis_base,
                "current": fmt(cur),
                "initial": fmt(initial_comp),
                "delta": fmt(cur - initial_comp),
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
        """kind: start | end -> 读取 {subdir}_{kind}_calibration.data，按乐器顺序拼接。"""
        all_rows: List[List[float]] = []
        for inst in instruments:
            subdir = INSTRUMENT_TO_SUBDIR.get(str(inst))
            if not subdir:
                self.get_logger().warn(f"未知乐器 id: {inst}")
                continue
            fname = f"{subdir}_{kind}_calibration.data"
            path = self._resource_root / subdir / fname
            if not path.is_file():
                self.get_logger().warn(f"缺少标定文件: {path}")
                continue
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
        time_ref: float = 0.01,
        vel_scale: float = 1.0,
    ) -> None:
        if self._joint_pub is None or UpperJointData is None:
            return
        msg = UpperJointData()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""
        msg.time_ref = float(time_ref)
        msg.vel_scale = float(vel_scale)
        js = JointState()
        js.name = list(JOINT_NAMES_PUBLISH)
        pos = [float(x) for x in positions]
        js.position = pos
        msg.joint = js
        self._joint_pub.publish(msg)
        with self._fk_lock:
            self._last_joint_cmd_positions = pos

    def _merge_joint_state_from_cache_and_last_cmd(self) -> List[float]:
        """
        与 JOINTS_WITH_SUFFIX 顺序一致。优先用 joint_states 缓存；缺失则用上一条 joint_cmd，
        避免单臂示教时未出现在 joint_states 里的关节被填 0。
        """
        with self._fk_lock:
            cache = dict(self._joint_positions_by_name)
            last = list(self._last_joint_cmd_positions) if self._last_joint_cmd_positions is not None else None
        out: List[float] = []
        for i, jn in enumerate(JOINTS_WITH_SUFFIX):
            if jn in cache:
                out.append(float(cache[jn]))
            elif last is not None and i < len(last):
                out.append(float(last[i]))
            else:
                out.append(0.0)
        return out

    def _build_hold_baseline_for_jog(self) -> List[float]:
        """
        为直线示教构造 "除操作臂外全部冻结" 的基线全身姿态。

        与 :meth:`_merge_joint_state_from_cache_and_last_cmd` 的关键差异：
        **优先沿用上一条 joint_cmd，而不是 /joint_states 缓存**。

        原因：真机在关节 PID + 重力补偿下，``/joint_states`` 反馈的角度会比
        "控制器在跟踪的指令点" 低一小段（重力 sag，通常 1–2 cm 级）。如果每次
        示教都把 cache 当作基线下发，等于主动把全身命令拉到 sag 状态 →
        **下游控制器会把整条手臂整体再往下拽一次**——与用户描述"不管哪个方向
        肘部都会下掉 1–2 cm" 完全吻合。

        策略：
        - 若曾经发过 joint_cmd（如播过标定轨迹或此前跳过直线示教）：沿用上次值
        - 否则首次示教：退回到 cache + 立即把本次 cache 记为基线，后续命令
          的非操作段不会再被 joint_states 的 sag 影响
        """
        with self._fk_lock:
            last = list(self._last_joint_cmd_positions) if self._last_joint_cmd_positions is not None else None
            cache = dict(self._joint_positions_by_name)
        if last is not None and len(last) == len(JOINTS_WITH_SUFFIX):
            return list(last)
        # 首次：用 cache 构造，并立刻把它固化为 "hold 基线"（last_joint_cmd_positions），
        # 这样下一次示教就不会再读 cache 的 sag 值
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

        # hold 基线：优先 last_cmd，首次退回 cache（并立刻固化成 last_cmd）
        base_full = self._build_hold_baseline_for_jog()
        with self._fk_lock:
            cache = dict(self._joint_positions_by_name)
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

        # ---- 诊断：FK on 当前 q，得到 base_link 系下当前末端位置 ----
        try:
            start_name_q = dict(chain_q_base)
            for k, jn in enumerate(arm_names):
                start_name_q[jn] = current_arm_q[k]
            p_start = self._fk.fk_tip_xyz(chain, start_name_q)
        except Exception as e:
            self.get_logger().error(f"{arm_label} 起始 FK 失败: {e}")
            return False, f"起始 FK 失败: {e}", None
        if p_start is None:
            return False, f"{arm_label} 起始 FK 返回 None（URDF 链异常）", None

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
            if self._kin_trac is not None and self._kin_trac.available:
                waypoints = cartesian_linear_waypoints_trac(
                    kin=self._kin_trac,
                    side=side_l,
                    chain_base_to_tip=chain,
                    arm_joint_names=arm_names,
                    chain_joint_q_base=chain_q_base,
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
        achieved = (
            last_ee["x"] - float(p_start[0]),
            last_ee["y"] - float(p_start[1]),
            last_ee["z"] - float(p_start[2]),
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
            for _ in range(ticks_per_wp):
                jog_rows.append(row)

        with self._traj_lock:
            if self._playing:
                return False, "有轨迹正在播放，请稍后再试", None
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

        base_full = self._build_hold_baseline_for_jog()
        with self._fk_lock:
            cache = dict(self._joint_positions_by_name)
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
            if self._kin_trac is not None and self._kin_trac.available:
                waypoints = cartesian_rotate_waypoints_trac(
                    kin=self._kin_trac,
                    side=side_l,
                    chain_base_to_tip=chain,
                    arm_joint_names=arm_names,
                    chain_joint_q_base=chain_q_base,
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
        end_name_q = dict(chain_q_base)
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
            for _ in range(ticks_per_wp):
                jog_rows.append(row)

        with self._traj_lock:
            if self._playing:
                return False, "有轨迹正在播放，请稍后再试", None
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
            },
        )

    def refresh_initial_ee(
        self, side: str
    ) -> Tuple[bool, str, Optional[Dict[str, Any]]]:
        """
        把当前末端 FK 位置（base_link，m）与姿态 RX/RY/RZ（Rz*Ry*Rx 欧拉，弧度存内部）写入「初始值」。

        - ``side`` 只能是 ``"left"`` 或 ``"right"``
        - 要求 FK 已加载且至少收到过一次 /joint_states（否则 `_fk_pose_ready=False`）
        - 同时会清除"开始标定结束后延迟 1s 自动采样"的序号，避免随后被意外覆盖
        """
        if not self._fk_ok or self._fk is None:
            return False, "URDF FK 未加载，无法刷新初始值", None
        with self._fk_lock:
            ready = self._fk_pose_ready
            ee = self._ee_left if side == "left" else self._ee_right
            rpy = self._ee_left_rpy if side == "left" else self._ee_right_rpy
        if not ready:
            return False, "尚未收到 /joint_states 或 FK 未就绪，请稍后再试", None
        with self._fk_lock:
            if side == "left":
                self._initial_ee_left = (float(ee[0]), float(ee[1]), float(ee[2]))
                self._initial_rpy_left = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
            else:
                self._initial_ee_right = (float(ee[0]), float(ee[1]), float(ee[2]))
                self._initial_rpy_right = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
            # 取消"开始标定播完后自动采样"的待处理序号，以免 1s 后把手动刷新的值覆盖
            self._initial_capture_seq += 1
        label = "左臂" if side == "left" else "右臂"
        deg = 180.0 / math.pi
        self.get_logger().info(
            f"[Web] 手动刷新{label}初始值：位置 m x={ee[0]:+.6f} y={ee[1]:+.6f} z={ee[2]:+.6f}；"
            f"姿态 ° RX={rpy[0]*deg:+.3f} RY={rpy[1]*deg:+.3f} RZ={rpy[2]*deg:+.3f}"
        )
        return (
            True,
            f"{label}初始值已刷新",
            {
                "initial_ee_m": {"x": float(ee[0]), "y": float(ee[1]), "z": float(ee[2])},
                "initial_rpy_deg": {
                    "rx": float(rpy[0] * deg),
                    "ry": float(rpy[1] * deg),
                    "rz": float(rpy[2] * deg),
                },
            },
        )

    def _schedule_initial_ee_after_start_calibration(self) -> None:
        """「开始标定」轨迹发完后延迟 1s，将此时手腕末端位姿写入初始值（供界面）。"""
        self._initial_capture_seq += 1
        seq = self._initial_capture_seq

        def run() -> None:
            time.sleep(1.0)
            if not rclpy.ok():
                return
            if seq != self._initial_capture_seq:
                return
            with self._fk_lock:
                if not (self._fk_pose_ready and self._fk_ok):
                    self.get_logger().warn(
                        "开始标定结束后 1s 时仍无有效末端 FK，未更新「初始值」（请检查 joint_states 与 URDF）"
                    )
                    return
                self._initial_ee_left = self._ee_left
                self._initial_ee_right = self._ee_right
                self._initial_rpy_left = self._ee_left_rpy
                self._initial_rpy_right = self._ee_right_rpy
            self.get_logger().info(
                "开始标定轨迹已播完，延迟 1s 后已更新左右腕末端位置与 RX/RY/RZ 初始值（界面「初始值」）"
            )

        threading.Thread(target=run, daemon=True).start()

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
        if row is not None:
            self._publish_joint_cmd(row)
        if finished_start_playback:
            self._schedule_initial_ee_after_start_calibration()
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
