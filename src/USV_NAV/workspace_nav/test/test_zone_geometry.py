# zone_geometry 栅格化单元测试
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from workspace_nav.zone_geometry import (
    FREE,
    KEEPOUT,
    build_fence_mask,
    build_zone_mask,
    draw_polyline_grid,
    fence_violation,
    fill_circle_grid,
    fill_polygon_grid,
    point_in_polygon,
)

SQ = [(0, 0), (10, 0), (10, 10), (0, 10)]


def test_point_in_polygon():
    assert point_in_polygon(5, 5, SQ) is True
    assert point_in_polygon(15, 5, SQ) is False
    assert point_in_polygon(0, 15, SQ) is False
    assert point_in_polygon(5, 5, [(0, 0), (1, 1)]) is False  # 少于 3 点


def test_fill_polygon_grid():
    w = h = 20
    mask = [FREE] * (w * h)
    changed = fill_polygon_grid(mask, w, h, SQ, KEEPOUT)
    assert changed == 100
    assert mask[5 * w + 5] == KEEPOUT
    assert mask[15 * w + 15] == FREE


def test_draw_polyline_not_closed_and_dilation():
    w = h = 20
    mask = [FREE] * (w * h)
    draw_polyline_grid(mask, w, h, [(0, 10), (19, 10)], KEEPOUT, 0)
    assert all(mask[10 * w + x] == KEEPOUT for x in range(w))
    # 不闭合：其它行不受影响
    assert mask[0] == FREE and mask[9 * w] == FREE

    mask = [FREE] * (w * h)
    draw_polyline_grid(mask, w, h, [(0, 10), (19, 10)], KEEPOUT, 1)
    assert mask[9 * w + 5] == KEEPOUT and mask[11 * w + 5] == KEEPOUT
    assert mask[8 * w + 5] == FREE


def test_build_zone_mask_work_area_inverts():
    w = h = 20
    work = [(5, 5), (15, 5), (15, 15), (5, 15)]
    mask = build_zone_mask(w, h, work, [], [], 2)
    assert mask[2 * w + 2] == KEEPOUT   # 作业区外禁行
    assert mask[10 * w + 10] == FREE    # 作业区内放行
    free = sum(1 for v in mask if v == FREE)
    assert 90 <= free <= 110


def test_build_zone_mask_forbidden_overrides_work_area():
    w = h = 20
    work = [(5, 5), (15, 5), (15, 15), (5, 15)]
    fb = [[(8, 8), (12, 8), (12, 12), (8, 12)]]
    mask = build_zone_mask(w, h, work, fb, [], 2)
    assert mask[10 * w + 10] == KEEPOUT  # 禁止区优先
    assert mask[6 * w + 6] == FREE


def test_build_zone_mask_empty_is_all_free():
    mask = build_zone_mask(20, 20, [], [], [], 2)
    assert all(v == FREE for v in mask)


def test_fill_circle_grid():
    w = h = 20
    mask = [FREE] * (w * h)
    fill_circle_grid(mask, w, h, 10.0, 10.0, 4.0, KEEPOUT)
    assert mask[10 * w + 10] == KEEPOUT      # 圆心
    assert mask[10 * w + 13] == KEEPOUT      # 半径内（格心距 3.54 < 4）
    assert mask[10 * w + 14] == FREE         # 半径外（格心距 4.53 > 4）
    assert mask[0] == FREE


def test_fence_violation_inclusion_union():
    fences = [
        {"fence_id": "wa1", "type": "inclusion", "shape": "polygon",
         "points": [(0, 0), (10, 0), (10, 10), (0, 10)]},
        {"fence_id": "wa2", "type": "inclusion", "shape": "circle",
         "center": (20.0, 5.0), "radius_m": 5.0},
    ]
    assert fence_violation(5, 5, fences) is None            # 多边形作业区内
    assert fence_violation(20, 5, fences) is None           # 圆形作业区内（并集）
    v = fence_violation(0, 15, fences)                      # 两个作业区都外
    assert v is not None and v[1] == "EXIT" and v[0]["fence_id"] == "wa1"


def test_fence_violation_exclusion_overrides():
    fences = [
        {"fence_id": "wa", "type": "inclusion", "shape": "polygon",
         "points": [(0, 0), (10, 0), (10, 10), (0, 10)]},
        {"fence_id": "fz", "type": "exclusion", "shape": "circle",
         "center": (5.0, 5.0), "radius_m": 2.0},
    ]
    v = fence_violation(5, 5, fences)
    assert v is not None and v[1] == "ENTER" and v[0]["fence_id"] == "fz"
    assert fence_violation(1, 1, fences) is None


def test_build_fence_mask():
    w = h = 20
    inc = [("polygon", [(5, 5), (15, 5), (15, 15), (5, 15)])]
    exc = [("circle", 10.0, 10.0, 2.0)]
    mask = build_fence_mask(w, h, inc, exc, [], 2)
    assert mask[2 * w + 2] == KEEPOUT    # 作业区外
    assert mask[6 * w + 6] == FREE       # 作业区内
    assert mask[10 * w + 10] == KEEPOUT  # 禁止圆内
    mask = build_fence_mask(w, h, [], [], [], 2)
    assert all(v == FREE for v in mask)
