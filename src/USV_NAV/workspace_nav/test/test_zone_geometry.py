# zone_geometry 栅格化单元测试
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from workspace_nav.zone_geometry import (
    FREE,
    KEEPOUT,
    build_zone_mask,
    draw_polyline_grid,
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
