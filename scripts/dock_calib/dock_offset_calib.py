#!/usr/bin/env python3
"""船坞 AprilTag dock_offset 现场自动标定工具。

用途
----
船正确停稳在坞内时，读取 TF ``base_link -> apriltag_<id>``（**不含**
dock_offset 的原始 Tag 位姿），反解每个 Tag 的 ``dock_offset_*``，使
``base_link -> dock_frame`` 与目标位姿重合（默认目标为 (0,0,0)），并把
结果写回 ``detection_cfg.yml``。

与 apriltag_node 的约定完全一致（见 apriltag_node.cpp）：

    tag_tf  = camera2camera_link * apriltagPoseToTf2(pose) * camera_tag2ros_
    dock_tf = tag_tf * dock_offset

因此，TF ``apriltag_<id>`` 就是 ``dock_offset`` 之前的 Tag 位姿；
当期望 ``T_base_dock = T_target`` 时：

    dock_offset = inv(T_base_tag) * T_target

坐标/角度约定：``tf2::Quaternion::setRPY(roll, pitch, yaw)``，YAML 中
roll/pitch/yaw 单位为度。

本工具只读 TF、可选开关检测通道/背光，并写 YAML；不会向船发任何运动指令。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import sys
import tempfile
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml


OFFSET_KEYS = (
    "dock_offset_x",
    "dock_offset_y",
    "dock_offset_z",
    "dock_offset_roll",
    "dock_offset_pitch",
    "dock_offset_yaw",
)

DEFAULT_CONFIG = (
    Path.home()
    / "usv-test/install/arm/apriltag_localization/config/detection_cfg.yml"
)


class CalibError(RuntimeError):
    """标定流程可预期错误（打印后退出，不抛 traceback）。"""


class ConfigError(CalibError):
    """配置文件读取/更新错误。"""


# ════════════════════════════════════════════════════════════════════
# 1. 刚体变换数学（与 tf2::setRPY / tf2::getRPY 一致）
# ════════════════════════════════════════════════════════════════════


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """R = Rz(yaw) * Ry(pitch) * Rx(roll)，与 tf2::Quaternion::setRPY 一致。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=float,
    )


def rpy_from_rotation_matrix(R: np.ndarray) -> Tuple[float, float, float]:
    """与 tf2::Matrix3x3::getRPY 一致的固定轴 XYZ 提取。"""
    sp = max(-1.0, min(1.0, -float(R[2, 0])))
    pitch = math.asin(sp)
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    else:  # gimbal lock，标定场景不会出现，保底处理
        roll = math.atan2(-float(R[1, 2]), float(R[1, 1]))
        yaw = 0.0
    return wrap_pi(roll), wrap_pi(pitch), wrap_pi(yaw)


def transform_from_xyz_rpy(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = rotation_matrix_from_rpy(roll, pitch, yaw)
    T[:3, 3] = (x, y, z)
    return T


def transform_to_xyz_rpy(
    T: np.ndarray, *, degrees: bool = True
) -> Tuple[float, float, float, float, float, float]:
    x, y, z = (float(v) for v in T[:3, 3])
    roll, pitch, yaw = rpy_from_rotation_matrix(T[:3, :3])
    if degrees:
        roll, pitch, yaw = math.degrees(roll), math.degrees(pitch), math.degrees(yaw)
    return x, y, z, roll, pitch, yaw


def invert_transform(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4, dtype=float)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def quat_from_matrix(R: np.ndarray) -> np.ndarray:
    """返回 [w, x, y, z]。"""
    m = R
    trace = float(m[0, 0] + m[1, 1] + m[2, 2])
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2, 1] - m[1, 2]) / s
        y = (m[0, 2] - m[2, 0]) / s
        z = (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=float)
    n = float(np.linalg.norm(q))
    return q / n if n > 0.0 else np.array([1.0, 0.0, 0.0, 0.0])


def matrix_from_quat(q: np.ndarray) -> np.ndarray:
    """q = [w, x, y, z]。"""
    w, x, y, z = (float(v) for v in q)
    n = w * w + x * x + y * y + z * z
    if n <= 0.0:
        return np.eye(3)
    s = 2.0 / n
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=float,
    )


def average_transform(samples: Sequence[np.ndarray]) -> np.ndarray:
    """平移取中位数，旋转取四元数 Markley 平均。"""
    if not samples:
        raise ValueError("average_transform: empty samples")
    translations = np.array([T[:3, 3] for T in samples], dtype=float)
    mean_t = np.median(translations, axis=0)

    quats = np.array([quat_from_matrix(T[:3, :3]) for T in samples], dtype=float)
    ref = quats[0]
    for i in range(len(quats)):
        if float(np.dot(quats[i], ref)) < 0.0:
            quats[i] = -quats[i]
    # Markley：最大特征值对应的特征向量
    M = quats.T @ quats
    eigvals, eigvecs = np.linalg.eigh(M)
    q = eigvecs[:, int(np.argmax(eigvals))]
    if q[0] < 0.0:
        q = -q
    q = q / np.linalg.norm(q)

    T = np.eye(4, dtype=float)
    T[:3, :3] = matrix_from_quat(q)
    T[:3, 3] = mean_t
    return T


def rotation_angle_rad(R: np.ndarray) -> float:
    c = max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0))
    return math.acos(c)


def transform_error(A: np.ndarray, B: np.ndarray) -> Tuple[float, float]:
    """返回 (平移误差 m, 旋转误差 deg)，A/B 同一参考系。"""
    dT = invert_transform(A) @ B
    return float(np.linalg.norm(dT[:3, 3])), math.degrees(rotation_angle_rad(dT[:3, :3]))


def transform_spread(samples: Sequence[np.ndarray]) -> Tuple[float, float]:
    """样本相对均值的平移/旋转 1σ 指标，用于判断船是否稳定。"""
    if not samples:
        return float("inf"), float("inf")
    avg = average_transform(samples)
    t_err = [float(np.linalg.norm(T[:3, 3] - avg[:3, 3])) for T in samples]
    r_err = [math.degrees(rotation_angle_rad(invert_transform(avg)[:3, :3] @ T[:3, :3])) for T in samples]
    return float(np.sqrt(np.mean(np.square(t_err)))), float(np.sqrt(np.mean(np.square(r_err))))


# ════════════════════════════════════════════════════════════════════
# 2. detection_cfg.yml 读取 / 行级更新（保留注释与格式）
# ════════════════════════════════════════════════════════════════════


@dataclass
class DockConfig:
    path: Path
    params: Dict[str, Any]
    tag_ids: List[int]
    offsets: Dict[int, Dict[str, float]]
    raw_text: str


def _infer_tag_ids(params: Dict[str, Any]) -> List[int]:
    ids: List[int] = []
    for key in params:
        m = re.match(r"tag_(\d+)$", str(key))
        if m:
            ids.append(int(m.group(1)))
    return sorted(ids)


def load_config(path: Path) -> DockConfig:
    path = path.expanduser()
    if not path.is_file():
        raise ConfigError(f"找不到配置文件：{path}")
    raw = path.read_text(encoding="utf-8")
    try:
        data = yaml.safe_load(raw) or {}
    except yaml.YAMLError as exc:
        raise ConfigError(f"YAML 解析失败：{path}: {exc}") from exc

    params = (data.get("apriltag_node") or {}).get("ros__parameters") or {}
    if not isinstance(params, dict) or not params:
        raise ConfigError(f"{path} 里没有 apriltag_node.ros__parameters")

    raw_ids = params.get("tag_ids") or []
    tag_ids = [int(v) for v in raw_ids]
    if not tag_ids:
        tag_ids = _infer_tag_ids(params)
    if not tag_ids:
        raise ConfigError(f"{path} 里没有 tag_ids / tag_<id> 配置")

    offsets: Dict[int, Dict[str, float]] = {}
    for tid in tag_ids:
        block = params.get(f"tag_{tid}")
        if not isinstance(block, dict):
            raise ConfigError(f"{path} 缺少 tag_{tid} 配置块")
        miss = [k for k in OFFSET_KEYS if k not in block]
        if miss:
            raise ConfigError(f"{path} 的 tag_{tid} 缺少字段：{', '.join(miss)}")
        offsets[tid] = {k: float(block[k]) for k in OFFSET_KEYS}

    return DockConfig(path=path, params=params, tag_ids=tag_ids, offsets=offsets, raw_text=raw)


def offsets_to_transform(off: Dict[str, float]) -> np.ndarray:
    return transform_from_xyz_rpy(
        float(off["dock_offset_x"]),
        float(off["dock_offset_y"]),
        float(off["dock_offset_z"]),
        math.radians(float(off["dock_offset_roll"])),
        math.radians(float(off["dock_offset_pitch"])),
        math.radians(float(off["dock_offset_yaw"])),
    )


def transform_to_offsets(T: np.ndarray) -> Dict[str, float]:
    x, y, z, roll, pitch, yaw = transform_to_xyz_rpy(T, degrees=True)
    return {
        "dock_offset_x": round(x, 6),
        "dock_offset_y": round(y, 6),
        "dock_offset_z": round(z, 6),
        "dock_offset_roll": round(roll, 6),
        "dock_offset_pitch": round(pitch, 6),
        "dock_offset_yaw": round(yaw, 6),
    }


_NUM_RE = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"


def update_config_text(
    text: str, new_offsets: Dict[int, Dict[str, float]]
) -> Tuple[str, List[str]]:
    """只改 dock_offset_* 的数值，保留缩进、行内注释、其余内容。"""
    lines = text.splitlines(keepends=True)
    changes: List[str] = []
    seen: Dict[int, set] = {int(tid): set() for tid in new_offsets}
    current_tag: Optional[int] = None
    tag_indent: Optional[int] = None

    for idx, line in enumerate(lines):
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        indent = len(line) - len(line.lstrip(" "))

        m_tag = re.match(r"^tag_(\d+)\s*:\s*(?:#.*)?$", stripped)
        if m_tag:
            tid = int(m_tag.group(1))
            if tid in new_offsets:
                current_tag = tid
                tag_indent = indent
            else:
                current_tag = None
                tag_indent = None
            continue

        if current_tag is None:
            continue
        if indent <= int(tag_indent):
            current_tag = None
            tag_indent = None
            continue

        for key in OFFSET_KEYS:
            # 先剥离行尾换行，替换完再原样补回，避免把多行粘在一起
            if line.endswith("\r\n"):
                ending, body = "\r\n", line[:-2]
            elif line.endswith("\n"):
                ending, body = "\n", line[:-1]
            else:
                ending, body = "", line
            m = re.match(
                rf"^(\s*{re.escape(key)}\s*:\s*)({_NUM_RE})(.*)$",
                body,
            )
            if not m:
                continue
            old_value = float(m.group(2))
            new_value = float(new_offsets[current_tag][key])
            lines[idx] = f"{m.group(1)}{new_value:.6f}{m.group(3)}{ending}"
            seen[current_tag].add(key)
            changes.append(
                f"tag_{current_tag}.{key}: {old_value:+.6f} -> {new_value:+.6f}"
            )
            break

    missing = {
        tid: [k for k in OFFSET_KEYS if k not in keys]
        for tid, keys in seen.items()
        if len(keys) != len(OFFSET_KEYS)
    }
    if missing:
        raise ConfigError(f"没有在文本中找到完整的 dock_offset 字段：{missing}")
    return "".join(lines), changes


def write_config(cfg: DockConfig, new_offsets: Dict[int, Dict[str, float]]) -> Tuple[Path, List[str]]:
    new_text, changes = update_config_text(cfg.raw_text, new_offsets)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup = cfg.path.with_name(cfg.path.name + f".bak_{stamp}")
    shutil.copy2(cfg.path, backup)
    fd, tmp_name = tempfile.mkstemp(prefix=cfg.path.name + ".", suffix=".tmp", dir=str(cfg.path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(new_text)
        os.replace(tmp_name, cfg.path)
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)
    return backup, changes


def print_offsets(cfg: DockConfig) -> None:
    print(f"配置文件：{cfg.path}")
    print(f"tag_ids：{cfg.tag_ids}")
    for tid in cfg.tag_ids:
        off = cfg.offsets[tid]
        print(
            f"  tag_{tid}: "
            f"x={off['dock_offset_x']:+.6f} y={off['dock_offset_y']:+.6f} "
            f"z={off['dock_offset_z']:+.6f} "
            f"r={off['dock_offset_roll']:+.6f}° p={off['dock_offset_pitch']:+.6f}° "
            f"yaw={off['dock_offset_yaw']:+.6f}°"
        )


# ════════════════════════════════════════════════════════════════════
# 3. 标定计算（纯函数，便于离线自测）
# ════════════════════════════════════════════════════════════════════


@dataclass
class TagCalibResult:
    tag_id: int
    samples: int
    fit_samples: int
    val_samples: int
    t_std_m: float
    r_std_deg: float
    val_t_m: float
    val_r_deg: float
    offsets: Dict[str, float]
    transform: np.ndarray


def calibrate_from_samples(
    samples: Dict[int, Sequence[np.ndarray]],
    target: np.ndarray,
    *,
    min_samples: int = 20,
) -> Dict[int, TagCalibResult]:
    """从每个 tag 的 T_base_tag 样本反解 dock_offset。"""
    results: Dict[int, TagCalibResult] = {}
    for tid, raw in samples.items():
        if len(raw) < min_samples:
            raise CalibError(f"tag_{tid} 有效样本只有 {len(raw)} 个，至少需要 {min_samples}")
        data = list(raw)
        fit_n = max(1, int(round(len(data) * 0.7)))
        fit = data[:fit_n]
        val = data[fit_n:] if len(data) > fit_n else data

        fit_avg = average_transform(fit)
        val_offsets_T = invert_transform(fit_avg) @ target

        val_errors = [transform_error(target, T @ val_offsets_T) for T in val]
        val_t = max(e[0] for e in val_errors) if val_errors else 0.0
        val_r = max(e[1] for e in val_errors) if val_errors else 0.0

        all_avg = average_transform(data)
        offsets_T = invert_transform(all_avg) @ target
        t_std, r_std = transform_spread(data)

        results[tid] = TagCalibResult(
            tag_id=tid,
            samples=len(data),
            fit_samples=len(fit),
            val_samples=len(val),
            t_std_m=t_std,
            r_std_deg=r_std,
            val_t_m=val_t,
            val_r_deg=val_r,
            offsets=transform_to_offsets(offsets_T),
            transform=offsets_T,
        )
    return results


def print_results(
    results: Dict[int, TagCalibResult],
    cfg: DockConfig,
    target: np.ndarray,
) -> None:
    print("\n采样与拟合质量：")
    print("  tag  n    t_std(mm)  r_std(deg)  val_t(mm)  val_r(deg)")
    for tid in sorted(results):
        r = results[tid]
        print(
            f"  {tid:<4d} {r.samples:<4d} {r.t_std_m * 1000.0:>9.1f} "
            f"{r.r_std_deg:>11.3f} {r.val_t_m * 1000.0:>10.1f} {r.val_r_deg:>11.3f}"
        )

    print("\n新 dock_offset（已按 node 的 setRPY 约定换算）：")
    for tid in sorted(results):
        off = results[tid].offsets
        old = cfg.offsets.get(tid, {})
        print(
            f"  tag_{tid}: "
            f"x={off['dock_offset_x']:+.6f} (旧 {old.get('dock_offset_x', float('nan')):+.6f}) "
            f"y={off['dock_offset_y']:+.6f} (旧 {old.get('dock_offset_y', float('nan')):+.6f}) "
            f"z={off['dock_offset_z']:+.6f} (旧 {old.get('dock_offset_z', float('nan')):+.6f})"
        )
        print(
            f"          r={off['dock_offset_roll']:+.5f}° p={off['dock_offset_pitch']:+.5f}° "
            f"yaw={off['dock_offset_yaw']:+.5f}°"
        )

    # 目标位姿提示
    tx, ty, tz, tr, tp, tyaw = transform_to_xyz_rpy(target, degrees=True)
    print(
        f"\n标定目标：base_link -> dock_frame = "
        f"({tx:+.6f}, {ty:+.6f}, {tz:+.6f}) m, "
        f"({tr:+.4f}°, {tp:+.4f}°, {tyaw:+.4f}°)"
    )


# ════════════════════════════════════════════════════════════════════
# 4. ROS 采样（只在船端/有 ROS 环境时导入 rclpy）
# ════════════════════════════════════════════════════════════════════


class RosSampler:
    def __init__(self, args: argparse.Namespace):
        import rclpy
        from rclpy.node import Node
        from tf2_ros import Buffer, TransformListener

        self.rclpy = rclpy
        self.args = args
        rclpy.init(args=None)
        self.node = Node("dock_offset_calib")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node, spin_thread=False)

        self.channel_client = None
        self.do_client = None
        self.channel_touched = False
        self.backlight_touched = False
        self.channel_enabled = False
        self.backlight_on = False

        if args.enable_channel:
            try:
                from m_common.srv import SetChannelEnabled
            except ImportError as exc:
                print(f"[WARN] 无法导入 m_common SetChannelEnabled，跳过通道自动开关：{exc}")
            else:
                self.channel_client = self.node.create_client(
                    SetChannelEnabled, args.channel_service
                )

        if args.backlight:
            try:
                from usv_rs485_driver.srv import SetDO
            except ImportError as exc:
                print(f"[WARN] 无法导入 usv_rs485_driver SetDO，跳过背光自动开关：{exc}")
            else:
                self.do_client = self.node.create_client(SetDO, args.set_do_service)

        self.dock_status_topic = args.dock_status_topic
        self.dock_state: Optional[str] = None
        try:
            from std_msgs.msg import String

            self.node.create_subscription(
                String,
                self.dock_status_topic,
                self._dock_status_cb,
                10,
            )
        except Exception:
            pass

    def _dock_status_cb(self, msg) -> None:
        try:
            data = json.loads(msg.data)
            self.dock_state = str(data.get("state", ""))
        except Exception:
            pass

    def spin(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)

    def check_idle(self, force: bool) -> None:
        self.spin(1.0)
        print(f"dock_mission 状态：{self.dock_state or '未知'}")
        if self.dock_state and self.dock_state != "IDLE" and not force:
            raise CalibError(
                f"当前 dock_mission 状态是 {self.dock_state}，不是 IDLE；"
                "标定会改检测通道/背光，可能与归港/出港任务冲突。"
                "确认安全后加 --force。"
            )

    def call_service(self, client, request, timeout: float = 3.0):
        if not client.wait_for_service(timeout_sec=2.0):
            return None
        fut = client.call_async(request)
        end = time.monotonic() + timeout
        while time.monotonic() < end and not fut.done():
            self.rclpy.spin_once(self.node, timeout_sec=0.05)
        if not fut.done():
            return None
        try:
            return fut.result()
        except Exception as exc:
            print(f"[WARN] 服务调用异常：{exc}")
            return None

    def set_channel(self, enabled: bool) -> None:
        if self.channel_client is None:
            return
        from m_common.srv import SetChannelEnabled

        req = SetChannelEnabled.Request()
        req.channel = self.args.channel
        req.enabled = bool(enabled)
        resp = self.call_service(self.channel_client, req)
        if resp is None:
            print(f"[WARN] 检测通道 {self.args.channel} -> {enabled} 无响应")
            return
        self.channel_touched = True
        self.channel_enabled = bool(enabled)
        print(f"检测通道 {self.args.channel} -> {enabled}: success={resp.success} {resp.message}")

    def set_backlight(self, on: bool) -> None:
        if self.do_client is None:
            return
        from usv_rs485_driver.srv import SetDO

        req = SetDO.Request()
        req.digital_output = int(self.args.backlight_mask) if on else 0
        resp = self.call_service(self.do_client, req)
        if resp is None:
            print(f"[WARN] 背光 -> {on} 无响应")
            return
        self.backlight_touched = True
        self.backlight_on = bool(on)
        print(f"背光 -> {on} (DO=0x{req.digital_output:04X}): success={resp.success} {resp.message}")

    def lookup(self, frame: str, max_age: float) -> Optional[np.ndarray]:
        from rclpy.duration import Duration
        from rclpy.time import Time

        try:
            tf = self.buffer.lookup_transform(
                self.args.base_frame,
                frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception:
            return None

        stamp = float(tf.header.stamp.sec) + float(tf.header.stamp.nanosec) * 1e-9
        now = self.node.get_clock().now().nanoseconds * 1e-9
        if now - stamp > max_age:
            return None

        T = np.eye(4, dtype=float)
        q = tf.transform.rotation
        T[:3, :3] = matrix_from_quat(np.array([q.w, q.x, q.y, q.z], dtype=float))
        t = tf.transform.translation
        T[:3, 3] = (t.x, t.y, t.z)
        return T

    def tag_frame(self, tid: int) -> str:
        return self.args.tag_frame_prefix + self.args.tag_frame_fmt.format(id=tid)

    def sample_tags(
        self,
        tag_ids: Sequence[int],
        samples: int,
        rate: float,
        max_age: float,
        timeout: float,
    ) -> Dict[int, List[np.ndarray]]:
        data: Dict[int, List[np.ndarray]] = {int(tid): [] for tid in tag_ids}
        start = time.monotonic()
        last_report = 0
        while time.monotonic() - start < timeout:
            self.rclpy.spin_once(self.node, timeout_sec=1.0 / max(rate, 1.0))
            got: Dict[int, np.ndarray] = {}
            for tid in tag_ids:
                T = self.lookup(self.tag_frame(int(tid)), max_age)
                if T is not None:
                    got[int(tid)] = T
            if len(got) == len(tag_ids):
                for tid, T in got.items():
                    data[tid].append(T)
                done = min(len(v) for v in data.values())
                if done - last_report >= 10:
                    print(f"  采样：{done}/{samples}")
                    last_report = done
                if all(len(v) >= samples for v in data.values()):
                    break
            elif self.args.verbose:
                missing = [tid for tid in tag_ids if int(tid) not in got]
                print(f"  等待 tag：{missing}")
        return data

    def sample_frame(
        self, frame: str, samples: int, rate: float, max_age: float, timeout: float
    ) -> List[np.ndarray]:
        data: List[np.ndarray] = []
        start = time.monotonic()
        while time.monotonic() - start < timeout and len(data) < samples:
            self.rclpy.spin_once(self.node, timeout_sec=1.0 / max(rate, 1.0))
            T = self.lookup(frame, max_age)
            if T is not None:
                data.append(T)
        return data

    def close(self) -> None:
        try:
            if self.channel_touched and self.channel_enabled and not self.args.keep_channel:
                self.set_channel(False)
            if self.backlight_touched and self.backlight_on and not self.args.keep_backlight:
                self.set_backlight(False)
        finally:
            try:
                self.node.destroy_node()
            except Exception:
                pass
            try:
                self.rclpy.shutdown()
            except Exception:
                pass


# ════════════════════════════════════════════════════════════════════
# 5. 运行模式
# ════════════════════════════════════════════════════════════════════


def target_transform(args: argparse.Namespace) -> np.ndarray:
    return transform_from_xyz_rpy(
        float(args.target_x),
        float(args.target_y),
        0.0,
        0.0,
        0.0,
        math.radians(float(args.target_yaw_deg)),
    )


def check_quality(
    results: Dict[int, TagCalibResult], args: argparse.Namespace
) -> List[str]:
    problems: List[str] = []
    for tid, r in results.items():
        if r.t_std_m > args.max_std_m:
            problems.append(
                f"tag_{tid} 平移抖动 {r.t_std_m * 1000:.1f}mm > {args.max_std_m * 1000:.1f}mm"
                "（船可能没停稳）"
            )
        if r.r_std_deg > args.max_std_deg:
            problems.append(
                f"tag_{tid} 旋转抖动 {r.r_std_deg:.2f}° > {args.max_std_deg:.2f}°"
                "（船可能没停稳）"
            )
        if r.val_t_m > args.max_residual_m:
            problems.append(
                f"tag_{tid} 验证残差 {r.val_t_m * 1000:.1f}mm > {args.max_residual_m * 1000:.1f}mm"
            )
        if r.val_r_deg > args.max_residual_deg:
            problems.append(
                f"tag_{tid} 验证残差 {r.val_r_deg:.2f}° > {args.max_residual_deg:.2f}°"
            )
    return problems


def run_calibrate(args: argparse.Namespace, cfg: DockConfig) -> int:
    sampler = RosSampler(args)
    try:
        sampler.check_idle(args.force)
        if args.backlight:
            sampler.set_backlight(True)
        if args.enable_channel:
            sampler.set_channel(True)
            # 给检测器/相机几帧时间建立 TF
            sampler.spin(args.warmup_sec)

        print(
            f"开始采样：tags={cfg.tag_ids} samples={args.samples} "
            f"rate={args.rate}Hz timeout={args.timeout}s base={args.base_frame}"
        )
        data = sampler.sample_tags(
            cfg.tag_ids, args.samples, args.rate, args.max_age, args.timeout
        )
        counts = {tid: len(v) for tid, v in data.items()}
        print(f"采样完成：{counts}")
        missing = [tid for tid, n in counts.items() if n < args.min_samples]
        if missing:
            raise CalibError(
                f"以下 tag 有效样本不足 {args.min_samples} 个：{missing}；"
                "检查相机、检测通道、Tag 可见性、TF frame 名。"
            )

        target = target_transform(args)
        results = calibrate_from_samples(data, target, min_samples=args.min_samples)
        print_results(results, cfg, target)

        problems = check_quality(results, args)
        if problems:
            print("\n[质量检查未通过]")
            for p in problems:
                print(f"  - {p}")
            if not args.force:
                print("未写配置。确认现场条件后可加 --force 强制写入。")
                return 2

        new_offsets = {tid: r.offsets for tid, r in results.items()}
        if args.dry_run or not args.write:
            print("\n当前为 dry-run，未写配置。确认无误后加 --write。")
            return 0

        backup, changes = write_config(cfg, new_offsets)
        print(f"\n已写入：{cfg.path}")
        print(f"备份：{backup}")
        for line in changes:
            print(f"  {line}")
        print(
            "\n下一步：重启感知/apriltag 节点（或整栈）使新 offset 生效，然后运行：\n"
            f"  {Path(sys.argv[0]).name} --config {cfg.path} --verify"
        )
        print(
            "注意：如需长期生效，还要同步更新源码配置：\n"
            "  ~/usv-main/src/perception/apriltag_localization/config/detection_cfg.yml\n"
            "或主机 /home/ght/usv/src/perception/apriltag_localization/config/detection_cfg.yml"
        )
        return 0
    finally:
        sampler.close()


def run_verify(args: argparse.Namespace, cfg: DockConfig) -> int:
    sampler = RosSampler(args)
    try:
        sampler.check_idle(args.force)
        if args.backlight:
            sampler.set_backlight(True)
        if args.enable_channel:
            sampler.set_channel(True)
            sampler.spin(args.warmup_sec)

        print(f"验证：采样 tags={cfg.tag_ids} + dock_frame={args.dock_frame}")
        data = sampler.sample_tags(
            cfg.tag_ids, args.samples, args.rate, args.max_age, args.timeout
        )
        target = target_transform(args)
        ok = True
        for tid in cfg.tag_ids:
            samples = data.get(tid, [])
            if len(samples) < args.min_samples:
                print(f"  tag_{tid}: 有效样本不足（{len(samples)}）")
                ok = False
                continue
            T_avg = average_transform(samples)
            T_dock = T_avg @ offsets_to_transform(cfg.offsets[tid])
            t_err, r_err = transform_error(target, T_dock)
            print(
                f"  tag_{tid}: 配置 offset 预测 base_link->dock "
                f"平移误差 {t_err * 1000:.1f} mm, 旋转误差 {r_err:.2f}°"
            )
            if t_err > args.max_residual_m or r_err > args.max_residual_deg:
                ok = False

        dock_samples = sampler.sample_frame(
            args.dock_frame, args.samples, args.rate, args.max_age, args.timeout
        )
        if len(dock_samples) >= args.min_samples:
            dock_avg = average_transform(dock_samples)
            t_err, r_err = transform_error(target, dock_avg)
            print(
                f"  dock_frame（运行中的节点）: 平移误差 {t_err * 1000:.1f} mm, "
                f"旋转误差 {r_err:.2f}°"
            )
            print(
                "  说明：dock_frame 使用节点启动时加载的 offset；"
                "如果刚改完配置还没重启，这个值可能仍是旧值。"
            )
        else:
            print(f"  dock_frame：有效样本不足（{len(dock_samples)}），无法验证运行中节点。")

        print("\n验证通过" if ok else "\n验证未通过，请检查配置/相机/安装")
        return 0 if ok else 2
    finally:
        sampler.close()


# ════════════════════════════════════════════════════════════════════
# 6. 自测（无需 ROS）
# ════════════════════════════════════════════════════════════════════


def run_self_test() -> int:
    rng = np.random.default_rng(20261019)

    # 6.1 RPY 往返
    for _ in range(200):
        rpy = rng.uniform(-math.pi, math.pi, 3)
        T = transform_from_xyz_rpy(1.2, -0.3, 0.4, *rpy)
        x, y, z, r, p, yaw = transform_to_xyz_rpy(T, degrees=False)
        R = rotation_matrix_from_rpy(r, p, yaw)
        if not np.allclose(R, T[:3, :3], atol=1e-9):
            raise AssertionError("RPY 往返不一致")

    # 6.2 合成标定：给定真实 offset，生成无噪声样本，反解应一致
    true_offsets = {
        0: transform_from_xyz_rpy(
            -0.666, 0.724, -0.649,
            math.radians(-0.19), math.radians(-0.25), math.radians(-0.26),
        ),
        1: transform_from_xyz_rpy(
            -0.688, -0.540, -0.649,
            math.radians(0.21), math.radians(0.34), math.radians(0.15),
        ),
    }
    target = np.eye(4)
    samples: Dict[int, List[np.ndarray]] = {}
    for tid, T_tag_dock in true_offsets.items():
        T_base_tag = invert_transform(T_tag_dock) @ target
        samples[tid] = [T_base_tag.copy() for _ in range(60)]

    results = calibrate_from_samples(samples, target, min_samples=10)
    for tid, expected in true_offsets.items():
        got = results[tid].transform
        t_err, r_err = transform_error(expected, got)
        if t_err > 1e-6 or r_err > 1e-4:
            raise AssertionError(f"合成标定 tag_{tid} 不一致：{t_err}, {r_err}")

    # 6.3 YAML 行级更新保留注释
    text = """apriltag_node:
  ros__parameters:
    tag_ids: [1, 0]
    tag_1: # tag id 1
      dock_offset_x: -0.688 #单位m
      dock_offset_y: -0.540
      dock_offset_z: -0.649
      dock_offset_roll: 0.21
      dock_offset_pitch: 0.34
      dock_offset_yaw: 0.15
    tag_0:
      dock_offset_x: -0.666
      dock_offset_y: 0.724
      dock_offset_z: -0.649
      dock_offset_roll: -0.19
      dock_offset_pitch: -0.25
      dock_offset_yaw: -0.26
"""
    new_offsets = {
        1: {k: 1.0 for k in OFFSET_KEYS},
        0: {k: 2.0 for k in OFFSET_KEYS},
    }
    updated, changes = update_config_text(text, new_offsets)
    if "#单位m" not in updated or "tag_1: # tag id 1" not in updated:
        raise AssertionError("YAML 更新破坏了注释")
    parsed = yaml.safe_load(updated)["apriltag_node"]["ros__parameters"]
    for tid, expected in ((0, 2.0), (1, 1.0)):
        block = parsed[f"tag_{tid}"]
        for key in OFFSET_KEYS:
            if abs(float(block[key]) - expected) > 1e-9:
                raise AssertionError(f"yaml tag_{tid}.{key} 期望 {expected}，实际 {block[key]}")
    if len(changes) != 12:
        raise AssertionError(f"期望 12 条变更，实际 {len(changes)}")

    print("SELFTEST OK")
    return 0


# ════════════════════════════════════════════════════════════════════
# 7. CLI
# ════════════════════════════════════════════════════════════════════


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="船坞 AprilTag dock_offset 现场自动标定/验证工具",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--config", default=str(DEFAULT_CONFIG), help="detection_cfg.yml 路径")
    p.add_argument("--print", dest="print_only", action="store_true", help="只打印当前 offset 后退出")
    p.add_argument("--verify", action="store_true", help="不改配置，验证当前 offset 的残差")
    p.add_argument("--self-test", action="store_true", help="运行纯数学/YAML 自测，无需 ROS")
    p.add_argument("--write", action="store_true", help="写入配置（默认 dry-run）")
    p.add_argument("--dry-run", action="store_true", help="不写配置（默认行为，显式声明用）")
    p.add_argument("--force", action="store_true", help="跳过 dock 状态/质量门限（慎用）")

    p.add_argument("--samples", type=int, default=60, help="每个 tag 的目标有效样本数")
    p.add_argument("--min-samples", type=int, default=20, help="最少有效样本数")
    p.add_argument("--rate", type=float, default=10.0, help="采样频率 Hz")
    p.add_argument("--timeout", type=float, default=40.0, help="采样超时 s")
    p.add_argument("--max-age", type=float, default=0.5, help="TF 最大龄期 s")
    p.add_argument("--warmup-sec", type=float, default=1.0, help="开通道后等待秒数")

    p.add_argument("--base-frame", default="base_link", help="参考系")
    p.add_argument("--tag-frame-fmt", default="apriltag_{id}", help="Tag TF 名格式")
    p.add_argument("--tag-frame-prefix", default="", help="多通道模式的 TF 前缀，如 back_uvc/")
    p.add_argument("--dock-frame", default="dock_frame", help="融合 dock TF 名（verify 用）")

    p.add_argument("--target-x", type=float, default=0.0, help="目标 dock 在 base_link 的 x（m）")
    p.add_argument("--target-y", type=float, default=0.0, help="目标 dock 在 base_link 的 y（m）")
    p.add_argument("--target-yaw-deg", type=float, default=0.0, help="目标 dock 相对 base_link 的 yaw（度）")

    p.add_argument("--enable-channel", dest="enable_channel", action="store_true", default=True,
                   help="自动开启 apriltag 检测通道")
    p.add_argument("--no-enable-channel", dest="enable_channel", action="store_false",
                   help="不自动开关检测通道")
    p.add_argument("--channel-service", default="/apriltag_node/set_channel_enabled",
                   help="SetChannelEnabled 服务名")
    p.add_argument("--channel", default="all", help="通道名（all=全部）")
    p.add_argument("--keep-channel", action="store_true", help="结束后不关闭检测通道")

    p.add_argument("--backlight", action="store_true", help="采样时点亮船坞背光 DO bit1")
    p.add_argument("--backlight-mask", type=int, default=2, help="背光 DO 掩码")
    p.add_argument("--set-do-service", default="set_do", help="SetDO 服务名")
    p.add_argument("--keep-backlight", action="store_true", help="结束后不关闭背光")

    p.add_argument("--dock-status-topic", default="/dock/status", help="dock_mission 状态话题")
    p.add_argument("--max-std-m", type=float, default=0.02, help="允许的平移抖动 1σ（m）")
    p.add_argument("--max-std-deg", type=float, default=1.0, help="允许的旋转抖动 1σ（度）")
    p.add_argument("--max-residual-m", type=float, default=0.05, help="允许的验证平移残差（m）")
    p.add_argument("--max-residual-deg", type=float, default=2.0, help="允许的验证旋转残差（度）")
    p.add_argument("--verbose", action="store_true", help="输出等待细节")
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.dry_run and args.write:
        print("[ERROR] --dry-run 与 --write 不能同时使用", file=sys.stderr)
        return 1
    if args.self_test:
        return run_self_test()

    try:
        cfg = load_config(Path(args.config))
    except CalibError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1

    if args.print_only:
        print_offsets(cfg)
        return 0

    try:
        if args.verify:
            return run_verify(args, cfg)
        return run_calibrate(args, cfg)
    except KeyboardInterrupt:
        print("\n已中断。")
        return 130
    except CalibError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    except Exception as exc:  # noqa: BLE001
        print(f"[ERROR] {type(exc).__name__}: {exc}", file=sys.stderr)
        if args.verbose:
            import traceback

            traceback.print_exc()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
