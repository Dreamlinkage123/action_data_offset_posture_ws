"""标定 Web：轨迹文件路径解析（resource / action_data 包 / offest_data / 自定义）。"""

from __future__ import annotations

import os
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

# 与 web_node.INSTRUMENT_TO_SUBDIR 一致
INSTRUMENT_TO_SUBDIR = {
    "drum": "drum",
    "bass": "bass",
    "guitar": "guitar",
    "keyboard": "keyboard",
}


def package_share_dir(package_name: str) -> Optional[Path]:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory(package_name))
    except Exception:
        return None


def _package_data_parent_local() -> Path:
    """源码树中 ``casbot_arm_calibration_web`` 包目录（与 offest_data / new_offset_data 同级）。"""
    return Path(__file__).resolve().parent.parent


def calibration_web_package_root() -> Path:
    """
    标定 Web 的**统一包根**（与 ``new_offset_data/``、``web_saved_offsets.json`` 同级）。

    已 source 时固定为 ``get_package_share_directory('casbot_arm_calibration_web')``，
    **不得**使用 ``offset_data_root().parent``：当 ``share/offest_data`` 尚未安装时，
    ``offset_data_root()`` 会退回到 ``site-packages/.../offest_data``，与
    ``share/.../new_offset_data`` 脱节，导致第二次「保存」写入的 JSON 与「生成」读取的不是同一文件，
    表现为无法生成新的带时间戳轨迹（仍用旧偏移）。
    """
    share = package_share_dir("casbot_arm_calibration_web")
    if share is not None:
        return share.resolve()
    return _package_data_parent_local().resolve()


def offset_data_root_local_fallback() -> Path:
    """未 source 时退回源码树 ``.../casbot_arm_calibration_web/offest_data``。"""
    return _package_data_parent_local() / "offest_data"


def offset_data_root() -> Path:
    share = package_share_dir("casbot_arm_calibration_web")
    if share is not None:
        p = share / "offest_data"
        if p.is_dir():
            return p
    local = offset_data_root_local_fallback()
    local.mkdir(parents=True, exist_ok=True)
    return local


def new_offset_data_dir() -> Path:
    """
    安装空间下 Web 生成轨迹的主目录：``get_package_share_directory(...) / new_offset_data``。

    **不**再依赖 ``offest_data`` 目录是否已创建：若仅依赖 ``offset_data_root().parent``，
    在 ``share/offest_data`` 尚未安装时会错误回退到 ``site-packages/...``，导致下拉列表为空、
    源码 ``src/.../new_offset_data`` 也看不到文件。
    """
    share = package_share_dir("casbot_arm_calibration_web")
    if share is not None:
        d = (share / "new_offset_data").resolve()
        d.mkdir(parents=True, exist_ok=True)
        return d
    d = (_package_data_parent_local() / "new_offset_data").resolve()
    d.mkdir(parents=True, exist_ok=True)
    return d


def workspace_src_package_root() -> Optional[Path]:
    """
    标准 colcon 布局下：``install[_*]/share/<pkg>`` → 工作空间 ``src/casbot_arm_calibration_web``。

    另尝试**上一层目录**（常见嵌套：外层目录下再放一层带 ``src`` 的仓库），
    **不再**沿父路径一直走到文件系统根 ``/``。

    Docker 场景下镜像里常存在 ``/src/casbot_arm_calibration_web``（与挂载的
    ``/workspace/<仓库名>/src/...`` 不是同一路径）。若向上遍历到 ``/`` 再匹配
    ``src/casbot_arm_calibration_web``，会误把生成结果写到 ``/src/...``，而用户在
    ``/workspace/.../src/...`` 里看不到文件。此处将搜索范围限制在 install 的父目录及其**仅一层**父目录。
    """
    share = package_share_dir("casbot_arm_calibration_web")
    if share is None:
        return None
    share = share.resolve()
    install_prefix = share.parent.parent.resolve()

    def _try_under(base: Path) -> Optional[Path]:
        base_r = base.resolve()
        if base_r == Path("/"):
            return None
        rels = (
            Path("src") / "casbot_arm_calibration_web",
            Path("action_data_offset_ws") / "src" / "casbot_arm_calibration_web",
            Path("action_data_offset_ws_old") / "action_data_offset_ws" / "src" / "casbot_arm_calibration_web",
        )
        for rel in rels:
            cand = (base_r / rel).resolve()
            try:
                cand.relative_to(base_r)
            except ValueError:
                continue
            if cand.is_dir() and (cand / "package.xml").is_file():
                return cand
        return None

    raw_ws = (os.environ.get("CASBOT_CALIB_WORKSPACE_ROOT") or "").strip()
    if raw_ws:
        hit = _try_under(Path(raw_ws).expanduser())
        if hit is not None:
            return hit

    ws = install_prefix.parent.resolve()
    anchors: List[Path] = []
    if ws != Path("/"):
        anchors.append(ws)
        parent = ws.parent.resolve()
        if parent != ws and parent != Path("/"):
            anchors.append(parent)

    for anchor in anchors:
        hit = _try_under(anchor)
        if hit is not None:
            return hit
    return None


def module_ros_package_root() -> Optional[Path]:
    """
    若当前加载的 ``trajectory_sources`` 来自带 ``package.xml`` 的源码树（含 symlink / editable），
    返回该 ROS 包根目录。

    当 :func:`workspace_src_package_root` 因非标准安装前缀推断失败时，仍能把生成结果写入
    ``src/.../casbot_arm_calibration_web/new_offset_data``，并在下拉列表中列出。
    """
    pkg = Path(__file__).resolve().parent.parent
    if (pkg / "package.xml").is_file():
        return pkg
    return None


def env_override_src_package_root() -> Optional[Path]:
    """
    环境变量 ``CASBOT_CALIB_SRC_PACKAGE_ROOT`` 指向 ``casbot_arm_calibration_web`` 包根（含 package.xml）。

    若只需指定**工作空间根**（其下为 ``src/casbot_arm_calibration_web``），可用
    ``CASBOT_CALIB_WORKSPACE_ROOT``（由 :func:`workspace_src_package_root` 读取）。
    """
    raw = (os.environ.get("CASBOT_CALIB_SRC_PACKAGE_ROOT") or "").strip()
    if not raw:
        return None
    p = Path(raw).expanduser().resolve()
    if p.is_dir() and (p / "package.xml").is_file():
        return p
    return None


def new_offset_data_write_targets() -> List[Path]:
    """生成偏移文件时应写入的目录（与 :func:`new_offset_data_search_roots` 顺序一致，并确保目录存在）。"""
    out: List[Path] = []
    for r in new_offset_data_search_roots():
        r.mkdir(parents=True, exist_ok=True)
        out.append(r)
    return out


def new_offset_data_search_roots() -> Sequence[Path]:
    """
    解析 / 列出 / 生成写入 ``new_offset_data/…`` 时依次查找的目录。

    **顺序：优先源码树下的 ``.../src/casbot_arm_calibration_web/new_offset_data``**（与工程里打开的目录一致），
    再 ``install/.../share/casbot_arm_calibration_web/new_offset_data``。

    此前若先写 install 而后未能镜像到当前工作空间的 ``src/...``，会出现「生成成功但 IDE 里 src 无文件、
    下拉却依赖扫描路径」的割裂；将开发目录置前并让 Jacobian 的 ``data_out_copy`` 指向首项，可保证
    生成结果落在用户查看的 ``src/.../new_offset_data`` 中，且播放解析与同一下拉列表同源。
    """
    share_root: Optional[Path] = None
    share = package_share_dir("casbot_arm_calibration_web")
    if share is not None:
        share_root = (share / "new_offset_data").resolve()

    roots: List[Path] = []
    src_pkg = workspace_src_package_root()
    if src_pkg is not None:
        roots.append((src_pkg / "new_offset_data").resolve())
    mod_pkg = module_ros_package_root()
    if mod_pkg is not None:
        roots.append((mod_pkg / "new_offset_data").resolve())
    env_pkg = env_override_src_package_root()
    if env_pkg is not None:
        roots.append((env_pkg / "new_offset_data").resolve())
    if share_root is not None:
        roots.append(share_root)
    if not roots:
        roots.append((_package_data_parent_local() / "new_offset_data").resolve())
    # 去重保序
    seen: set[str] = set()
    uniq: List[Path] = []
    for r in roots:
        k = str(r)
        if k not in seen:
            seen.add(k)
            uniq.append(r)
    return uniq


def list_action_data_basenames() -> List[str]:
    share = package_share_dir("action_data_offset")
    if share is None:
        return []
    d = share / "data"
    if not d.is_dir():
        return []
    return sorted(
        p.name
        for p in d.iterdir()
        if p.is_file() and p.suffix.lower() in (".data", ".csv")
    )


def list_offset_data_relative_paths() -> List[str]:
    """列出可播放的偏移轨迹：``offest_data`` 下相对路径 + ``new_offset_data/<子路径>``。"""
    out: List[str] = []
    root = offset_data_root()
    if root.is_dir():
        for p in sorted(root.rglob("*")):
            if p.is_file() and p.suffix.lower() in (".data", ".csv"):
                out.append(str(p.relative_to(root)).replace("\\", "/"))
    listed_names: set[str] = set()
    for nroot in new_offset_data_search_roots():
        if not nroot.is_dir():
            continue
        for p in sorted(nroot.rglob("*")):
            if p.is_file() and p.suffix.lower() in (".data", ".csv"):
                rel = str(p.relative_to(nroot)).replace("\\", "/")
                key = f"new_offset_data/{rel}"
                if rel in listed_names:
                    continue
                listed_names.add(rel)
                out.append(key)
    return sorted(out)


def resource_calibration_path(resource_root: Path, instruments: List[str], kind: str) -> Optional[Path]:
    """单乐器时返回唯一路径；多乐器时拼接逻辑在调用方处理。"""
    if not instruments:
        return None
    if len(instruments) > 1:
        return None
    sub = INSTRUMENT_TO_SUBDIR.get(str(instruments[0]))
    if not sub:
        return None
    fname = f"{sub}_{kind}_calibration.data"
    path = resource_root / sub / fname
    return path if path.is_file() else None


def resolve_trajectory_path(
    *,
    mode: str,
    resource_root: Path,
    instruments: List[str],
    action_data_basename: str,
    offset_data_relative: str,
    custom_path: str,
) -> Tuple[Optional[Path], str]:
    """
    mode: resource_start | resource_end | action_data | offset_data | custom
    """
    m = (mode or "").strip().lower()
    if m in ("resource_start", "resource_end"):
        kind = "start" if m.endswith("start") else "end"
        if not instruments:
            return None, "请选择乐器"
        all_rows_path: List[Path] = []
        for inst in instruments:
            sub = INSTRUMENT_TO_SUBDIR.get(str(inst))
            if not sub:
                continue
            fname = f"{sub}_{kind}_calibration.data"
            path = resource_root / sub / fname
            if path.is_file():
                all_rows_path.append(path)
            else:
                return None, f"缺少标定文件: {path}"
        if not all_rows_path:
            return None, "未找到 resource 标定数据"
        if len(all_rows_path) > 1:
            return None, "当前实现请单选乐器以解析唯一文件；多乐器请改用合并流程"
        return all_rows_path[0], ""
    if m == "action_data":
        base = (action_data_basename or "").strip()
        if not base:
            return None, "请选择或填写 action_data 文件名"
        share = package_share_dir("action_data_offset")
        if share is None:
            return None, "未找到 action_data_offset 包（请 source 安装空间）"
        path = share / "data" / base
        if not path.is_file():
            return None, f"文件不存在: {path}"
        return path, ""
    if m == "offset_data":
        rel = (offset_data_relative or "").strip().replace("\\", "/")
        if not rel or ".." in rel:
            return None, "无效的偏移数据相对路径"
        if rel.startswith("new_offset_data/"):
            inner = rel[len("new_offset_data/") :].lstrip("/")
            if not inner or ".." in inner:
                return None, "无效的 new_offset_data 相对路径"
            path: Optional[Path] = None
            for root in new_offset_data_search_roots():
                root = root.resolve()
                if not root.is_dir():
                    continue
                cand = (root / inner).resolve()
                try:
                    cand.relative_to(root)
                except ValueError:
                    continue
                if cand.is_file():
                    path = cand
                    break
            if path is None:
                return None, f"文件不存在: {rel}"
        else:
            root = offset_data_root()
            path = (root / rel).resolve()
            try:
                path.relative_to(root.resolve())
            except ValueError:
                return None, "路径必须位于 offest_data 目录内"
        if not path.is_file():
            return None, f"文件不存在: {path}"
        return path, ""
    if m == "custom":
        raw = (custom_path or "").strip()
        if not raw:
            return None, "请填写自定义轨迹绝对路径"
        path = Path(raw)
        if not path.is_file():
            return None, f"自定义路径不是文件: {path}"
        return path, ""
    return None, f"未知 mode: {mode}"
