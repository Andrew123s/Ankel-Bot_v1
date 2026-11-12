"""Optional uplink of locally stored data.

The vessel never waits for this. ``Uploader.try_sync`` is called occasionally
(e.g. while docked); it POSTs unsynced rows as JSON to a configured HTTP
endpoint and marks them synced only after a 2xx reply. With no endpoint, no
network or a server error it returns 0 and the data simply stays on disk.
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request

from .storage import TABLES, Store


class Uploader:
    def __init__(self, store: Store, endpoint: str = "", vessel_id: str = "ankelbot-01", timeout: float = 5.0):
        self.store, self.endpoint, self.vessel_id, self.timeout = store, endpoint, vessel_id, timeout

    def try_sync(self, batch: int = 500) -> int:
        """Upload one batch per table. Returns the number of rows confirmed."""
        if not self.endpoint:
            return 0
        done = 0
        for table in TABLES:
            rows = self.store.unsynced(table, batch)
            if not rows:
                continue
            body = json.dumps({"vessel": self.vessel_id, "table": table, "rows": rows}).encode("utf-8")
            req = urllib.request.Request(self.endpoint, data=body, headers={"Content-Type": "application/json"}, method="POST")
            try:
                with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                    if not 200 <= resp.status < 300:
                        return done
            except (urllib.error.URLError, TimeoutError, OSError):
                return done                                   # offline: keep the data, try another time
            self.store.mark_synced(table, [r["id"] for r in rows])
            done += len(rows)
        return done
