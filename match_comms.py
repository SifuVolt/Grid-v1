
"""Small team communication protocol for Grid teammates.

Grid never assumes that an opponent or unrelated bot understands this protocol.
Messages are advisory; local perception always remains authoritative.
"""
from __future__ import annotations
from dataclasses import dataclass
import json
from time import monotonic

PROTOCOL = "GRID/1"

@dataclass(slots=True)
class TeamMessage:
    player_index: int
    role: str
    action: str
    intercept_time: float | None
    boost: float
    timestamp: float

class MatchComms:
    def __init__(self, stale_after: float = 0.65):
        self.stale_after = stale_after
        self.peers: dict[int, TeamMessage] = {}
        self.last_signature = None
        self.last_send = 0.0

    def encode(self, player_index: int, role: str, action: str,
               intercept_time: float | None, boost: float) -> bytes:
        payload = {"v": 1, "i": int(player_index), "r": str(role),
                   "a": str(action), "t": intercept_time, "b": round(float(boost), 1)}
        return json.dumps(payload, separators=(",", ":")).encode("utf-8")

    def receive(self, data: bytes, now: float | None = None):
        now = monotonic() if now is None else now
        try:
            payload = json.loads(data.decode("utf-8"))
            if payload.get("v") != 1:
                return False
            index = int(payload["i"])
            self.peers[index] = TeamMessage(index, str(payload["r"]), str(payload["a"]),
                                            payload.get("t"), float(payload.get("b", 0)), now)
            return True
        except (ValueError, TypeError, KeyError, UnicodeDecodeError):
            return False

    def active_peers(self, now: float | None = None):
        now = monotonic() if now is None else now
        return {i: p for i, p in self.peers.items() if now-p.timestamp <= self.stale_after}

    def should_send(self, signature, interval: float = 0.12,
                    now: float | None = None) -> bool:
        now = monotonic() if now is None else now
        if signature != self.last_signature or now-self.last_send >= interval:
            self.last_signature, self.last_send = signature, now
            return True
        return False
