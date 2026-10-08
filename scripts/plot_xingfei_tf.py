#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
兴飞船（xingfei）base_link → 传感器 外参示意图。

数据来源：直接解析 usv_transforms 的 cleaning_boat_xingfei.xacro（xacro 展开后的
URDF），因此图与 xacro 不会各自漂移。改完 xacro 重跑本脚本即可出图：

    source /opt/ros/humble/setup.bash
    python3 scripts/plot_xingfei_tf.py \
        --xacro /home/ght/usv/src/usv_transforms/urdf/cleaning_boat_xingfei.xacro \
        --out   docs/xingfei_tf_layout.png
"""

import argparse
import math
import os
import re
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager as fm
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, Rectangle


def use_cjk_font():
    for name in ("Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Zen Hei"):
        if any(f.name == name for f in fm.fontManager.ttflist):
            plt.rcParams["font.sans-serif"] = [name]
            break
    plt.rcParams["axes.unicode_minus"] = False


def load_urdf(xacro_path):
    try:
        import xacro  # noqa: WPS433  (ROS 环境提供)
    except ImportError:
        sys.exit("找不到 xacro 模块：先 source /opt/ros/humble/setup.bash")
    doc = xacro.process_file(os.path.expanduser(xacro_path))
    return doc.toxml(), doc


def parse_joints(xml):
    """返回 [(parent, child, (x,y,z), (r,p,y))]，fixed joint。"""
    out = []
    for m in re.finditer(r"<joint\b[^>]*>(.*?)</joint>", xml, re.S):
        body = m.group(1)
        if 'type="fixed"' not in xml[m.start():m.start(0) + 400] and "fixed" not in m.group(0):
            continue
        parent = re.search(r'<parent link="([^"]+)"', body)
        child = re.search(r'<child link="([^"]+)"', body)
        origin = re.search(r'<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"', body)
        if origin is None:  # xacro 可能调换属性顺序
            origin = re.search(r'<origin\s+rpy="([^"]+)"\s+xyz="([^"]+)"', body)
            if origin is None:
                continue
            rpy, xyz = origin.group(1), origin.group(2)
        else:
            xyz, rpy = origin.group(1), origin.group(2)
        if not (parent and child):
            continue
        out.append((
            parent.group(1),
            child.group(1),
            tuple(float(v) for v in xyz.split()),
            tuple(float(v) for v in rpy.split()),
        ))
    return out


def euler_to_matrix(rpy):
    from math import cos, sin
    r, p, y = rpy
    return [
        (cos(y) * cos(p), cos(y) * sin(p) * sin(r) - sin(y) * cos(r), cos(y) * sin(p) * cos(r) + sin(y) * sin(r)),
        (sin(y) * cos(p), sin(y) * sin(p) * sin(r) + cos(y) * cos(r), sin(y) * sin(p) * cos(r) - cos(y) * sin(r)),
        (-sin(p), cos(p) * sin(r), cos(p) * cos(r)),
    ]


def mat_vec(m, v):
    return tuple(sum(m[i][j] * v[j] for j in range(3)) for i in range(3))


ROWS = [
    # (frame, 族, 中文名, 说明后缀)
    ("livox_frame", "livox", "livox Mid360", "斜向后上方 0 -43.9° π（2026-09-28 标定）"),
    ("camera_front_hik_link", "cam_fwd", "海康 HIK（前）", "镜头朝 +x"),
    ("camera_front_gemini_link", "cam_fwd", "双目 Gemini 330", "镜头朝 +x"),
    ("camera_back_video_link", "cam_back", "后视 UVC", "朝后安装"),
    ("gps_link", "gps", "GPS / GNSS 天线", "见“待确认 1)”"),
]

COLOR = {
    "livox": "#d62728",
    "cam_fwd": "#e07b1f",
    "cam_back": "#8c564b",
    "gps": "#2ca02c",
}

# 俯视图里各标签的纵向层级：x 相近的传感器分到不同层，避免文字压在一起
TOP_LABEL_Y = {
    "camera_back_video_link": 0.22,
    "camera_front_hik_link": 0.56,
    "livox_frame": 0.86,
    "camera_front_gemini_link": -0.36,
    "gps_link": -0.36,
}


def parse_hull(xml):
    """从 base_link 的 visual 里读占位船体，返回 (长, 宽, 高, 中心 x)。"""
    m = re.search(r'<link name="base_link">.*?</link>', xml, re.S)
    if not m:
        return 2.72, 1.30, 1.07, 1.44
    o = re.search(r'<origin xyz="([^"]+)"', m.group(0))
    b = re.search(r'<box size="([^"]+)"', m.group(0))
    cx = float(o.group(1).split()[0]) if o else 1.44
    size = [float(v) for v in b.group(1).split()] if b else [2.72, 1.30, 1.07]
    return size[0], size[1], size[2], cx


def rpy_text(rpy):
    """rpy 一律按角度显示，便于和现场标定值对照。"""
    out = []
    for v in rpy:
        deg = math.degrees(v)
        out.append("0°" if abs(deg) < 0.05 else f"{deg:.1f}°")
    return " ".join(out)


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--xacro", default="/home/ght/usv/src/usv_transforms/urdf/cleaning_boat_xingfei.xacro")
    ap.add_argument("--out", default=os.path.join(here, "..", "docs", "xingfei_tf_layout.png"))
    args = ap.parse_args()

    use_cjk_font()
    xml, doc = load_urdf(args.xacro)
    joints = parse_joints(xml)
    root = "base_link"
    direct = {c: (p, xyz, rpy) for p, c, xyz, rpy in joints if p == root}
    by_parent = {}
    for p, c, xyz, rpy in joints:
        by_parent.setdefault(p, []).append((c, xyz, rpy))

    missing = [f for f, *_ in ROWS if f not in direct]
    if missing:
        sys.exit(f"xacro 里没有这些 base_link 直连帧：{missing}")

    # livox 的 +x（雷达光轴）在 base_link 系里的指向：斜装时不再沿船轴
    livox_rpy = direct["livox_frame"][2]
    livox_dir = mat_vec(euler_to_matrix(livox_rpy), (1.0, 0.0, 0.0))
    livox_tilt_deg = math.degrees(math.atan2(livox_dir[2], math.hypot(livox_dir[0], livox_dir[1])))

    # ── 数据表 ────────────────────────────────────────────────────────────
    table_rows = []
    for frame, kind, cn, note in ROWS:
        xyz, rpy = direct[frame][1], direct[frame][2]
        if kind == "livox":  # 姿态随实船标定变，说明文字跟着算
            pitch_deg = math.degrees(rpy[1])
            roll_deg = math.degrees(rpy[0])
            if abs(pitch_deg) > 1.0:
                note = f"斜装：X 轴仰 {abs(pitch_deg):.1f}°"
            elif abs(abs(roll_deg) - 180.0) < 5.0:
                note = "倒装（roll ≈ 180°）"
            else:
                note = "正装"
        table_rows.append([cn, f"{xyz[0]:+.2f} {xyz[1]:+.2f} {xyz[2]:+.2f}", rpy_text(rpy), note])

    # IMU（挂在 livox_frame 下，随 livox_frame 一起转）
    for child, xyz, rpy in by_parent.get("livox_frame", []):
        if child == "imu_link":
            lxyz = direct["livox_frame"][1]
            g = mat_vec(euler_to_matrix(direct["livox_frame"][2]), xyz)
            g = tuple(g[i] + lxyz[i] for i in range(3))
            table_rows.append([
                "livox 内置 IMU",
                f"{g[0]:+.3f} {g[1]:+.3f} {g[2]:+.3f}",
                "0 0 0",
                "雷达内置，硬件常数",
            ])

    # ── 画布 ──────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16.6, 11.6), dpi=150)
    fig.patch.set_facecolor("white")
    gs = fig.add_gridspec(3, 2, height_ratios=[2.25, 0.95, 1.20], width_ratios=[1.66, 1.0],
                          hspace=0.62, wspace=0.14, left=0.05, right=0.975, top=0.90, bottom=0.045)

    ax_top = fig.add_subplot(gs[0, 0])
    ax_side = fig.add_subplot(gs[1, 0], sharex=ax_top)
    ax_tbl = fig.add_subplot(gs[0:2, 1])
    ax_note = fig.add_subplot(gs[2, :])
    for ax in (ax_tbl, ax_note):
        ax.axis("off")

    fig.suptitle("兴飞船 Xingfei：base_link(= RTK/GI320 报告点) → 传感器 静态 TF"
                 "（外参取自 cleaning_boat_xingfei.xacro）",
                 fontsize=17, fontweight="bold", x=0.05, ha="left", y=0.968)

    # 船体占位外形：直接从 base_link 的 visual 读，改 xacro 就跟着变
    hull_l, hull_w, hull_h, hull_cx = parse_hull(xml)
    hull_x0, hull_x1 = hull_cx - hull_l / 2, hull_cx + hull_l / 2

    # ── 俯视图 ────────────────────────────────────────────────────────────
    X0, X1 = -0.95, 3.05
    ax_top.set_xlim(X0, X1)
    ax_top.set_ylim(-1.30, 1.46)
    ax_top.add_patch(Rectangle((hull_x0, -hull_w / 2), hull_l, hull_w, fill=False,
                               ls=(0, (7, 5)), lw=1.4, ec="#9aa4ad"))
    ax_top.text(hull_x0 + 0.05, hull_w / 2 + 0.06,
                f"船体外形（占位尺寸 {hull_l:g} × {hull_w:g}，待实测）",
                ha="left", va="bottom", fontsize=10, color="#6b7480")
    ax_top.axhline(0, color="#37474f", lw=1.0, ls=(0, (4, 4)))
    ax_top.text(X0 + 0.03, 0.045, "船体中线 y = 0", fontsize=9.5, color="#37474f")

    # 飞控基座：现场量取的参考点（= 海康 − (0.03, 0, 0.27)），不是 TF 帧
    fc_xyz = tuple(direct["camera_front_hik_link"][1][i] - (0.03, 0.0, 0.27)[i] for i in range(3))
    ax_top.axvline(fc_xyz[0], color="#1f77b4", lw=1.4, ls=(0, (5, 4)))
    ax_top.plot([fc_xyz[0]], [0], marker="s", ms=12, mfc="none", mec="#1f77b4", mew=2.0, zorder=6)
    ax_top.text(fc_xyz[0] - 0.05, 1.40, f"飞控基座 x = +{fc_xyz[0]:.2f} m（现场量取基准，非 TF 帧）",
                ha="right", va="center", fontsize=11, color="#1f77b4")

    # base_link
    ax_top.plot([0], [0], marker="*", ms=26, mfc="#111111", mec="white", mew=1.2, zorder=7)
    ax_top.annotate("base_link = RTK/GI320 报告点（TF 基准，原点）", xy=(0, 0), xytext=(0.10, -1.00),
                    fontsize=11.5, ha="left", va="center", color="#111111",
                    arrowprops=dict(arrowstyle="-", lw=1.0, color="#111111", ls=":"))

    # 传感器：中线上的点 + 引线标签（层级见 TOP_LABEL_Y）
    for frame, kind, cn, _ in ROWS:
        x, y, z = direct[frame][1]
        col = COLOR[kind]
        ax_top.plot([x], [0], marker="o", ms=11, mfc=col, mec="white", mew=1.6, zorder=6)
        yl = TOP_LABEL_Y[frame]
        ax_top.plot([x, x], [0, yl], color=col, lw=0.9, ls=":", zorder=3)
        ax_top.text(x, yl + (0.06 if yl > 0 else -0.06), f"{cn}\nx = {x:+.2f}   z = {z:+.2f}",
                    ha="center", va="bottom" if yl > 0 else "top", fontsize=11.5,
                    color=col, fontweight="bold", linespacing=1.45)

    # 镜头朝向
    for frame, kind, *_ in ROWS:
        x = direct[frame][1][0]
        if kind == "cam_fwd":
            ax_top.add_patch(FancyArrowPatch((x, -0.15), (x + 0.26, -0.15),
                                             arrowstyle="-|>", mutation_scale=15,
                                             lw=2.0, color=COLOR[kind], zorder=5))
        elif kind == "cam_back":
            ax_top.add_patch(FancyArrowPatch((x, -0.15), (x - 0.26, -0.15),
                                             arrowstyle="-|>", mutation_scale=15,
                                             lw=2.0, color=COLOR[kind], zorder=5))
            ax_top.text(x - 0.30, -0.15, "镜头朝 −x（后）", ha="right", va="center",
                        fontsize=10.5, color=COLOR[kind])

    ax_top.annotate("", xy=(X1 - 0.05, 1.34), xytext=(X1 - 0.65, 1.34),
                    arrowprops=dict(arrowstyle="-|>", lw=2.4, color="#111111"))
    ax_top.text(X1 - 0.68, 1.34, "船头 / +x", ha="right", va="center", fontsize=11.5)
    ax_top.set_title("俯视图（x–y）：传感器都装在船体中线上（y = 0）；livox 斜装，光轴朝船尾后上方",
                     loc="left", fontsize=12, color="#37474f", pad=8)
    ax_top.set_yticks([])
    ax_top.set_xticks([-0.5, 0, 0.5, 1.0, 1.5, 2.0])
    ax_top.tick_params(labelsize=11)
    ax_top.set_xlabel("base_link x (m) —— +x 为船头", fontsize=11)
    for s in ("top", "right", "left"):
        ax_top.spines[s].set_visible(False)
    ax_top.spines["bottom"].set_color("#b0bec5")

    # ── 侧视图 ────────────────────────────────────────────────────────────
    ax_side.set_ylim(-0.58, 0.62)
    ax_side.add_patch(Rectangle((hull_x0, -hull_h / 2), hull_l, hull_h, fill=False,
                                ls=(0, (7, 5)), lw=1.4, ec="#9aa4ad"))
    ax_side.axhline(0, color="#1f77b4", lw=1.4)
    ax_side.text(X0 + 0.03, 0.035, "z = 0：base_link（RTK/GI320 报告点）高度面", ha="left", va="bottom",
                 fontsize=10, color="#1f77b4")

    # 4 个传感器的标签排到右侧同一列，避免高度相近时压字；GPS 贴自己标记上方。
    front = sorted(((direct[f][1][0], direct[f][1][2], kind, cn)
                    for f, kind, cn, _ in ROWS if kind != "gps"), key=lambda t: -t[1])
    for (x, z, kind, cn), y_lab in zip(front, [0.52, 0.30, 0.06, -0.26]):
        col = COLOR[kind]
        ax_side.plot([x], [z], marker="o", ms=9, mfc=col, mec="white", mew=1.4, zorder=6)
        ax_side.plot([x, 2.12], [z, y_lab], color=col, lw=0.9, ls=":", zorder=3)
        ax_side.text(2.17, y_lab, f"{cn.split('（')[0]}  z = {z:+.2f}", fontsize=10.5,
                     color=col, fontweight="bold", ha="left", va="center")
    gx, gz = direct["gps_link"][1][0], direct["gps_link"][1][2]
    ax_side.plot([gx], [gz], marker="o", ms=9, mfc=COLOR["gps"], mec="white", mew=1.4, zorder=6)
    ax_side.text(gx, gz - 0.10, "GPS / GNSS 天线（基准点）  z = %+.2f" % gz, fontsize=10.5,
                 color=COLOR["gps"], fontweight="bold", ha="center", va="top")

    # livox 光轴（侧视投影）：斜装标定值一眼可见
    lx, lz = direct["livox_frame"][1][0], direct["livox_frame"][1][2]
    span = math.hypot(livox_dir[0], livox_dir[2]) or 1.0
    tip = (lx + 0.55 * livox_dir[0] / span, lz + 0.55 * livox_dir[2] / span)
    ax_side.add_patch(FancyArrowPatch((lx, lz), tip, arrowstyle="-|>", mutation_scale=15,
                                      lw=2.2, color=COLOR["livox"], zorder=6))
    ax_side.text(tip[0] - 0.03, tip[1] - 0.09, f"livox +x：仰 {abs(livox_tilt_deg):.1f}°",
                 ha="right", va="center", fontsize=10, color=COLOR["livox"], fontweight="bold")
    ax_side.set_title(f"侧视图（x–z）：基准点取 z = 0；livox 光轴仰 {abs(livox_tilt_deg):.1f}°"
                      "（2026-09-28 实船标定）",
                      loc="left", fontsize=12, color="#37474f", pad=8)
    ax_side.set_yticks([-0.3, 0, 0.3])
    ax_side.tick_params(labelsize=11)
    ax_side.set_ylabel("z (m)", fontsize=11)
    ax_side.set_xlabel("base_link x (m)", fontsize=11)
    for s in ("top", "right"):
        ax_side.spines[s].set_visible(False)
    ax_side.spines["bottom"].set_color("#b0bec5")
    ax_side.spines["left"].set_color("#b0bec5")

    # ── 数值表 ────────────────────────────────────────────────────────────
    ax_tbl.set_title("base_link → 各帧（单位 m / rad）", fontsize=14, fontweight="bold",
                     loc="left", pad=14)
    tab = ax_tbl.table(
        cellText=[[r[0], r[1], r[2], r[3]] for r in table_rows],
        colLabels=["帧（link）", "xyz（base_link 系）", "rpy（度）", "说明"],
        colWidths=[0.30, 0.26, 0.19, 0.27],
        cellLoc="left", loc="upper center", bbox=[0.0, 0.40, 1.0, 0.55],
    )
    tab.auto_set_font_size(False)
    tab.set_fontsize(9.5)
    for (r, c), cell in tab.get_celld().items():
        cell.set_edgecolor("#cfd8dc")
        cell.set_linewidth(0.8)
        cell.PAD = 0.03
        if r == 0:
            cell.set_facecolor("#eceff1")
            cell.set_text_props(fontweight="bold")
    ax_tbl.text(0.0, 0.35,
                "基准：base_link = RTK/GI320 报告点\n"
                "  （mavros gps_input 零杆臂直发，基准必须取它）\n"
                "  现场量的是“相对飞控”的偏移：\n"
                "  整体平移到 GPS 点后得本表，飞控落在 +1.67\n"
                "  除 livox（斜向后上方，2026-09-28 标定）外 rpy 一律 0（标准 ENU）",
                fontsize=10.5, va="top", color="#37474f", linespacing=1.6)

    # ── 待确认 ────────────────────────────────────────────────────────────
    note = (
        "待现场确认（4 条）\n"
        "1)  “GPS → 海康相机 = (1.7, 0, 0.58)”按“GPS 为原点、指向海康”取：海康在基准点前方 1.70 m、上方 0.58 m"
        "（与澄丰“相机在船头、天线在船尾”同号）。若实测相反（相机在天线后方/下方），全表 x、z 反号。\n"
        "2)  口述里有一处矛盾：说“飞控在 base_link 前 1.00 m”，但按 GPS↔海康 1.7 m 推出来是 +1.67 m。"
        "现场量 GPS 天线 ↔ 飞控 的实际距离：若是 1.67 则本表正确；若确实只有 1.0，本表整体再平移 0.67 m。\n"
        "3)  GI320 报的是天线相位中心，还是机器内部参考点？若报的是“飞控盒上那点”，基准应移到飞控（本表整体 −1.67 m）。\n"
        "4)  后视相机已按朝后安装设 rpy = 0 0 180°；若实船另有俯仰/横滚，改 camera_back_video_rpy 即可。"
    )
    ax_note.text(0.0, 1.05, note, fontsize=11.5, va="top", ha="left", color="#263238",
                 linespacing=1.7,
                 bbox=dict(boxstyle="round,pad=0.7", fc="#fff8e1", ec="#ffb300", lw=1.4))
    ax_note.text(0.0, -0.05,
                 f"数据源：{os.path.abspath(args.xacro)}    （改 xacro 后重跑本脚本即可刷新此图）",
                 fontsize=9.5, color="#78909c", va="top")

    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out, facecolor="white")
    print("OK ->", out)
    for row in table_rows:
        print("  ", " | ".join(row))


if __name__ == "__main__":
    main()
