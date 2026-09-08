"""Tests for parse_task_event."""

from __future__ import annotations

import json

import pytest

from dock_mission.task_event import parse_task_event


def test_parse_json_event_with_detail():
    raw = json.dumps(
        {
            "event": "TASK_COMPLETED",
            "detail": {"mission_id": "dock_staging", "reason": "ok"},
        }
    )
    parsed = parse_task_event(raw)
    assert parsed is not None
    event, detail = parsed
    assert event == "TASK_COMPLETED"
    assert detail["mission_id"] == "dock_staging"


def test_parse_json_event_non_dict_detail():
    raw = json.dumps({"event": "TASK_FAILED", "detail": "bad"})
    event, detail = parse_task_event(raw)
    assert event == "TASK_FAILED"
    assert detail == {}


def test_parse_legacy_completed_string():
    parsed = parse_task_event("something TASK_COMPLETED happened")
    assert parsed == ("TASK_COMPLETED", {})


def test_parse_legacy_failed_string():
    parsed = parse_task_event("TASK_FAILED nav")
    assert parsed == ("TASK_FAILED", {})


def test_parse_invalid_returns_none():
    assert parse_task_event("not an event") is None
    assert parse_task_event("") is None


def test_parse_json_missing_event():
    raw = json.dumps({"detail": {"task_id": "x"}})
    event, detail = parse_task_event(raw)
    assert event == ""
    assert detail == {"task_id": "x"}
