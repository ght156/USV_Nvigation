"""Parse /task_event payloads from mission_bridge (JSON or legacy strings)."""

from __future__ import annotations

import json
from typing import Optional


def parse_task_event(raw: str) -> Optional[tuple[str, dict]]:
    """Return (event_name, detail_dict) or None if unparseable."""
    try:
        data = json.loads(raw)
    except json.JSONDecodeError:
        if "TASK_COMPLETED" in raw:
            return "TASK_COMPLETED", {}
        if "TASK_FAILED" in raw:
            return "TASK_FAILED", {}
        return None
    event = str(data.get("event", ""))
    detail = data.get("detail") if isinstance(data.get("detail"), dict) else {}
    return event, detail
