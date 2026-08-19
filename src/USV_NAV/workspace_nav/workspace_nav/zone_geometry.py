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
