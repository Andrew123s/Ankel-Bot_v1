"""Local data store (SQLite).

Everything the vessel observes and decides is written to a file on the
onboard disk. Nothing depends on a network; ``sync.py`` can upload rows later
if a connection happens to exist. Rows carry a ``synced`` flag for that.
"""
from __future__ import annotations

import json
import sqlite3
import time
from pathlib import Path

SCHEMA = """
CREATE TABLE IF NOT EXISTS telemetry (
    id INTEGER PRIMARY KEY, wall REAL, t REAL, x REAL, y REAL, psi REAL, u REAL, soc REAL,
    state TEXT, cmd_port REAL, cmd_stbd REAL, gate INTEGER, wing_port REAL, wing_stbd REAL,
    n_tracks INTEGER, synced INTEGER DEFAULT 0);
CREATE TABLE IF NOT EXISTS detections (
    id INTEGER PRIMARY KEY, wall REAL, t REAL, label TEXT, conf REAL, lat REAL, fwd REAL,
    width REAL, truncated INTEGER, synced INTEGER DEFAULT 0);
CREATE TABLE IF NOT EXISTS events (
    id INTEGER PRIMARY KEY, wall REAL, t REAL, kind TEXT, detail TEXT, synced INTEGER DEFAULT 0);
"""
TABLES = ("telemetry", "detections", "events")


class Store:
    def __init__(self, path: str | Path, telemetry_every: int = 5):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(str(path))
        self.db.execute("PRAGMA journal_mode=WAL")      # survives power loss mid-write
        self.db.executescript(SCHEMA)
        self.every = telemetry_every
        self._k = 0
        self._last_commit = time.monotonic()

    def telemetry(self, s, out, cmd, n_tracks: int) -> None:
        self._k += 1
        if self._k % self.every:
            return
        self.db.execute(
            "INSERT INTO telemetry (wall,t,x,y,psi,u,soc,state,cmd_port,cmd_stbd,gate,wing_port,wing_stbd,n_tracks) "
            "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?)",
            (time.time(), s.t, s.x, s.y, s.psi, s.u, s.soc, out.state.name, cmd[0], cmd[1], out.gate, out.wing[0], out.wing[1], n_tracks))
        self._maybe_commit()

    def detection(self, t: float, m) -> None:
        self.db.execute("INSERT INTO detections (wall,t,label,conf,lat,fwd,width,truncated) VALUES (?,?,?,?,?,?,?,?)",
                        (time.time(), t, m.label, m.conf, m.lat, m.fwd, m.width, int(m.truncated)))

    def event(self, t: float, kind: str, **detail) -> None:
        self.db.execute("INSERT INTO events (wall,t,kind,detail) VALUES (?,?,?,?)", (time.time(), t, kind, json.dumps(detail)))
        self.db.commit()

    def _maybe_commit(self) -> None:
        if time.monotonic() - self._last_commit > 1.0:
            self.db.commit()
            self._last_commit = time.monotonic()

    # ---- used by sync ----
    def unsynced(self, table: str, limit: int = 500) -> list[dict]:
        assert table in TABLES
        cur = self.db.execute(f"SELECT * FROM {table} WHERE synced = 0 ORDER BY id LIMIT ?", (limit,))
        cols = [c[0] for c in cur.description]
        return [dict(zip(cols, row)) for row in cur.fetchall()]

    def mark_synced(self, table: str, ids: list[int]) -> None:
        assert table in TABLES
        self.db.executemany(f"UPDATE {table} SET synced = 1 WHERE id = ?", [(i,) for i in ids])
        self.db.commit()

    def count(self, table: str) -> int:
        assert table in TABLES
        return self.db.execute(f"SELECT COUNT(*) FROM {table}").fetchone()[0]

    def close(self) -> None:
        self.db.commit()
        self.db.close()
