"""Panel registry — the manifest-driven contract between the rak-car
monitor service and the persistent frontend in rak-hri.

A *Panel* is a single unit the frontend can render. Each panel has a
type, an endpoint where its data lives, and a handful of layout hints.
The frontend fetches /api/panels, walks the list, and dispatches each
entry to a widget component based on `type`.

Adding a new feature to the platform is a three-line operation:

    from tools.panels import Panel, REGISTRY
    REGISTRY.register(Panel(
        id="thermal-cam",
        type="mjpeg",
        title="Thermal",
        endpoint="/api/cameras/thermal/mjpeg",
        size="md",
    ))

The frontend picks it up on the next manifest poll (default 30s).
"""

from __future__ import annotations

import hashlib
import json
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Optional


# Panel sizes drive the dashboard's grid placement. These map 1:1 to CSS
# grid column spans in the frontend (`span-3`, `span-4`, `span-6`, `span-12`).
SIZE_SM = "sm"  # 3 of 12 columns
SIZE_MD = "md"  # 4 of 12 columns
SIZE_LG = "lg"  # 6 of 12 columns
SIZE_XL = "xl"  # 12 of 12 columns (full row)


@dataclass
class Panel:
    id: str
    type: str
    title: str
    endpoint: str
    description: str = ""
    size: str = SIZE_MD
    poll_ms: int = 0  # 0 = no polling (streaming types like mjpeg)
    row_span: int = 1
    meta: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        d = asdict(self)
        return d


class PanelRegistry:
    """Thread-safe panel collection. Auto-orders panels so the layout is
    stable across requests: raw cameras first, then processed, then state
    panels, then widgets."""

    _PREFERRED_ORDER = {
        "mjpeg": 0,
        "detection_stream": 1,
        "value": 2,
        "gauge": 3,
        "sensor_grid": 4,
        "keypad": 5,
        "log": 6,
    }

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._panels: dict[str, Panel] = {}

    def register(self, panel: Panel) -> None:
        if not panel.id or not panel.type or not panel.endpoint:
            raise ValueError(f"panel requires id/type/endpoint, got {panel!r}")
        with self._lock:
            self._panels[panel.id] = panel

    def unregister(self, panel_id: str) -> None:
        with self._lock:
            self._panels.pop(panel_id, None)

    def get(self, panel_id: str) -> Optional[Panel]:
        with self._lock:
            return self._panels.get(panel_id)

    def list(self) -> list[Panel]:
        with self._lock:
            items = list(self._panels.values())
        items.sort(
            key=lambda p: (
                self._PREFERRED_ORDER.get(p.type, 99),
                p.id,
            )
        )
        return items

    def manifest(self) -> dict:
        panels = [p.to_dict() for p in self.list()]
        body = json.dumps(panels, sort_keys=True, ensure_ascii=False)
        h = hashlib.sha256(body.encode("utf-8")).hexdigest()[:12]
        return {
            "version": "1.0",
            "generated_at": int(time.time()),
            "hash": h,
            "count": len(panels),
            "panels": panels,
        }


# Module-level singleton; tools.streamer imports this and registers its
# built-in panels at import time. Tests can also register custom panels
# against the same singleton.
REGISTRY = PanelRegistry()
