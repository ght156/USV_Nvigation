#!/usr/bin/env python3
# 作业区/禁止区/硬边界的纯几何与栅格化工具（无 rclpy，可单测）。
#
# 约定：
# - 多边形（作业区/禁止区）：点依次连接，最后一点自动与第一点闭合；
# - 折线（硬边界）：点依次连接，不闭合首末点；
# - 栅格掩码为 flat list[int]，长度 width*height，行优先（iy*width+ix），
#   值 100 = 禁行（keepout），0 = 自由。

from __future__ import annotations

from typing import List, Sequence, Tuple

KEEPOUT = 100
FREE = 0

Point = Tuple[float, float]


def point_in_polygon(px: float, py: float, pts: Sequence[Point]) -> bool:
    """射线法点在多边形内判定。pts 为 (x, y) 顶点列表（无需重复首点闭合）。"""
    n = len(pts)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > py) != (yj > py):
            x_cross = (xj - xi) * (py - yi) / (yj - yi) + xi
            if px < x_cross:
                inside = not inside
        j = i
    return inside


def point_in_circle(px: float, py: float, cx: float, cy: float, r: float) -> bool:
    """点是否在圆内。"""
    return (px - cx) ** 2 + (py - cy) ** 2 <= r * r


def point_in_fence_map(px: float, py: float, fence: dict) -> bool:
    """点是否在单个围栏内。fence 为 map 系 dict：
    {"shape": "polygon", "points": [(x, y), ...]} 或
    {"shape": "circle", "center": (x, y), "radius_m": r}。"""
    if fence.get("shape") == "circle" and fence.get("center") is not None:
        return point_in_circle(px, py, fence["center"][0], fence["center"][1], fence["radius_m"])
    return point_in_polygon(px, py, fence.get("points") or [])


def fence_violation(px: float, py: float, fences: Sequence[dict]):
    """按 GeoFence 语义检查违规；返回 (fence, transition) 或 None。
    - 存在 inclusion（作业区）时点必须在至少一个内，否则视为驶出第一个 inclusion（EXIT）
    - 点落入任一 exclusion（禁航区）为闯入（ENTER）
    """
    inclusions = [f for f in fences if f.get("type") == "inclusion"]
    if inclusions and not any(point_in_fence_map(px, py, f) for f in inclusions):
        return inclusions[0], "EXIT"
    for f in fences:
        if f.get("type") == "exclusion" and point_in_fence_map(px, py, f):
            return f, "ENTER"
    return None


def fill_polygon_grid(
    mask: List[int], width: int, height: int, pts_grid: Sequence[Point], value: int
) -> int:
    """把多边形内部格点置为 value。pts_grid 为栅格浮点坐标（单位：格）。返回改动格数。"""
    if len(pts_grid) < 3:
        return 0
    min_x = max(0, int(min(p[0] for p in pts_grid)))
    max_x = min(width - 1, int(max(p[0] for p in pts_grid)) + 1)
    min_y = max(0, int(min(p[1] for p in pts_grid)))
    max_y = min(height - 1, int(max(p[1] for p in pts_grid)) + 1)
    changed = 0
    for iy in range(min_y, max_y + 1):
        row = iy * width
        cy = iy + 0.5
        for ix in range(min_x, max_x + 1):
            if point_in_polygon(ix + 0.5, cy, pts_grid):
                idx = row + ix
                if mask[idx] != value:
                    mask[idx] = value
                    changed += 1
    return changed


def _bresenham_cells(x0: int, y0: int, x1: int, y1: int):
    """整数 Bresenham 画线，逐格 yield (ix, iy)。"""
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            return
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy


def fill_circle_grid(
    mask: List[int],
    width: int,
    height: int,
    cx: float,
    cy: float,
    radius_cells: float,
    value: int,
) -> int:
    """把圆（栅格坐标圆心 + 半径格数）内部格点置为 value。返回改动格数。"""
    if radius_cells <= 0:
        return 0
    r2 = radius_cells * radius_cells
    min_x = max(0, int(cx - radius_cells))
    max_x = min(width - 1, int(cx + radius_cells) + 1)
    min_y = max(0, int(cy - radius_cells))
    max_y = min(height - 1, int(cy + radius_cells) + 1)
    changed = 0
    for iy in range(min_y, max_y + 1):
        row = iy * width
        dy2 = (iy + 0.5 - cy) ** 2
        for ix in range(min_x, max_x + 1):
            if (ix + 0.5 - cx) ** 2 + dy2 <= r2:
                idx = row + ix
                if mask[idx] != value:
                    mask[idx] = value
                    changed += 1
    return changed


def draw_polyline_grid(
    mask: List[int],
    width: int,
    height: int,
    pts_grid: Sequence[Point],
    value: int,
    dilation_cells: int = 0,
) -> int:
    """把折线（不闭合）画进掩码：相邻点间 Bresenham 连线，每个线上格点
    加盖 (2*dilation+1)^2 方形章，防止对角泄漏。返回改动格数。"""
    if len(pts_grid) < 2:
        return 0
    d = max(0, int(dilation_cells))
    changed = 0

    def stamp(cx: int, cy: int) -> None:
        nonlocal changed
        for iy in range(cy - d, cy + d + 1):
            if iy < 0 or iy >= height:
                continue
            row = iy * width
            for ix in range(cx - d, cx + d + 1):
                if ix < 0 or ix >= width:
                    continue
                idx = row + ix
                if mask[idx] != value:
                    mask[idx] = value
                    changed += 1

    for k in range(len(pts_grid) - 1):
        x0, y0 = pts_grid[k]
        x1, y1 = pts_grid[k + 1]
        for cx, cy in _bresenham_cells(int(round(x0)), int(round(y0)), int(round(x1)), int(round(y1))):
            stamp(cx, cy)
    return changed


def build_zone_mask(
    width: int,
    height: int,
    work_area_grid: Sequence[Point],
    forbidden_grids: Sequence[Sequence[Point]],
    boundary_grids: Sequence[Sequence[Point]],
    boundary_dilation_cells: int = 2,
) -> List[int]:
    """生成 keepout 掩码：
    1. 有作业区：全图禁行，作业区内部放行；无作业区：全图自由。
    2. 禁止区内部置禁行（覆盖作业区，禁止区优先）。
    3. 硬边界折线膨胀后叠加禁行。
    """
    if len(work_area_grid) >= 3:
        mask = [KEEPOUT] * (width * height)
        fill_polygon_grid(mask, width, height, work_area_grid, FREE)
    else:
        mask = [FREE] * (width * height)
    for poly in forbidden_grids:
        fill_polygon_grid(mask, width, height, poly, KEEPOUT)
    for line in boundary_grids:
        draw_polyline_grid(mask, width, height, line, KEEPOUT, boundary_dilation_cells)
    return mask


# 围栏填充描述：("polygon", [(gx, gy), ...]) 或 ("circle", cx, cy, radius_cells)
FenceFill = Tuple


def _apply_fill(mask: List[int], width: int, height: int, fill: FenceFill, value: int) -> None:
    if fill[0] == "polygon":
        fill_polygon_grid(mask, width, height, fill[1], value)
    elif fill[0] == "circle":
        fill_circle_grid(mask, width, height, fill[1], fill[2], fill[3], value)


def build_fence_mask(
    width: int,
    height: int,
    inclusion_fills: Sequence[FenceFill],
    exclusion_fills: Sequence[FenceFill],
    boundary_grids: Sequence[Sequence[Point]],
    boundary_dilation_cells: int = 2,
) -> List[int]:
    """按 GeoFence 模型生成 keepout 掩码：
    1. 存在 inclusion（作业区）围栏时：全图禁行，所有 inclusion 的并集放行；
       不存在时全图自由。
    2. exclusion（禁航区）内部置禁行（覆盖 inclusion）。
    3. 硬边界折线膨胀后叠加禁行。
    """
    if inclusion_fills:
        mask = [KEEPOUT] * (width * height)
        for f in inclusion_fills:
            _apply_fill(mask, width, height, f, FREE)
    else:
        mask = [FREE] * (width * height)
    for f in exclusion_fills:
        _apply_fill(mask, width, height, f, KEEPOUT)
    for line in boundary_grids:
        draw_polyline_grid(mask, width, height, line, KEEPOUT, boundary_dilation_cells)
    return mask
