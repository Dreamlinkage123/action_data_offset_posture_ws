"""默认路径：包根下 ``data/`` 与 ``output/`` 同级（源码树或 share 安装树），避免手 export ADO_*。"""

from __future__ import annotations

from pathlib import Path
from typing import Optional


def _ament_prefix() -> Optional[Path]:
    try:
        from ament_index_python.packages import get_package_prefix

        return Path(get_package_prefix("action_data_offset")).resolve()
    except Exception:
        return None


def _ament_share() -> Optional[Path]:
    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("action_data_offset")).resolve()
    except Exception:
        return None


def find_source_package_dir() -> Optional[Path]:
    """若存在 colcon 工作空间，返回 .../src/action_data_offset；否则 None。"""
    prefix = _ament_prefix()
    if prefix is None:
        return None
    for base in (prefix, prefix.parent):
        cand = base / "src" / "action_data_offset"
        if (cand / "package.xml").is_file():
            return cand
    return None


def package_root() -> Path:
    """资源根：有源码树用 src/action_data_offset，否则用 install/share/action_data_offset。"""
    src = find_source_package_dir()
    if src is not None:
        return src
    share = _ament_share()
    if share is not None:
        return share
    raise RuntimeError(
        "无法解析 action_data_offset 包路径：请 source ROS 工作空间 install/setup.bash 后再运行。"
    )


def default_data_dir() -> Path:
    return package_root() / "data"


def default_output_dir() -> Path:
    """与 ``data/`` 同级：源码树为 ``src/.../output``，仅安装树为 ``share/.../action_data_offset/output``。

    若该目录只读无法创建，回退到 ``~/ado_output``。
    """
    out = package_root() / "output"
    try:
        out.mkdir(parents=True, exist_ok=True)
        return out
    except OSError:
        fallback = Path.home() / "ado_output"
        fallback.mkdir(parents=True, exist_ok=True)
        return fallback


def resolve_model_path(path_str: str) -> str:
    """--robot-model / --urdf：支持 urdf/xml/... 相对包根；当前目录优先。"""
    p = Path(path_str)
    if p.is_absolute():
        return str(p)
    cwd = Path.cwd()
    pkg = package_root()
    trials = [cwd / p, pkg / p]
    for t in trials:
        if t.is_file():
            return str(t.resolve())
    return str((pkg / p).resolve())


def resolve_data_path(path_str: str) -> str:
    """--data 解析规则：

    - ``data/...`` → 固定为包内 ``<package>/data/...``（Orin 上即 ``share/action_data_offset/data/``）。
    - ``output/...`` → 固定为包内 ``<package>/output/...``，**不会**再叠一层 ``output``。
    - 仅文件名 → 先 ``data/`` 再 ``output/``；另试 cwd 与包根相对路径。
    """
    raw = path_str.strip()
    p = Path(raw)
    if p.is_absolute():
        return str(p)

    norm = raw.replace("\\", "/")
    pkg = package_root()
    data_dir = default_data_dir()
    output_dir = default_output_dir()
    cwd = Path.cwd()

    if norm.startswith("data/"):
        return str((data_dir / norm[5:]).resolve())
    if norm == "data":
        return str(data_dir.resolve())
    if norm.startswith("output/"):
        return str((output_dir / norm[7:]).resolve())
    if norm == "output":
        return str(output_dir.resolve())

    trials = [
        cwd / p,
        pkg / p,
        data_dir / p,
        output_dir / p,
        data_dir / p.name,
        output_dir / p.name,
    ]
    seen: set[Path] = set()
    for t in trials:
        t = t.resolve()
        if t in seen:
            continue
        seen.add(t)
        if t.is_file():
            return str(t)
    if p.parent == Path("."):
        return str((data_dir / p.name).resolve())
    return str((pkg / p).resolve())


def resolve_output_path(output: Optional[str], default_filename: str) -> str:
    """--output：仅文件名时写入 default_output_dir()；``output/...`` 相对包内 output/；其余相对 cwd。

    绝对路径**不**触发 ``default_output_dir()``，从而在未 source 工作空间时也能直接写入绝对路径。
    """
    if output:
        p_abs = Path(output)
        if p_abs.is_absolute():
            parent = p_abs.parent
            if not parent.exists():
                parent.mkdir(parents=True, exist_ok=True)
            return str(p_abs)
    out_base = default_output_dir()
    if not output:
        return str((out_base / default_filename).resolve())
    raw = output.strip().replace("\\", "/")
    if raw.startswith("output/"):
        target = (out_base / raw[7:]).resolve()
        target.parent.mkdir(parents=True, exist_ok=True)
        return str(target)
    p = Path(output)
    if p.parent == Path("."):
        return str((out_base / p.name).resolve())
    target = (Path.cwd() / p).resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    return str(target)
