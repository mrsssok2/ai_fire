"""Phone alerts via a Telegram bot (pure Python, stdlib only): message texts, rate limiting, the Telegram HTTP client.

The bot token and chat id are never stored here: `TelegramClient` gets them from the caller (the node reads them from the
environment) and strips the token from every error text it raises, so it cannot leak into logs.
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request
import uuid

API = "https://api.telegram.org"


def status_line(cnn=None, flames=(), gas_ppm=None, water_pct=None) -> str:
    """'CNN 100% | flame: L F | gas 1500 ppm | water 78%' (missing values are left out)."""
    parts = []
    if cnn is not None:
        parts.append(f"CNN {cnn * 100:.0f}%")
    parts.append("flame: " + (" ".join(flames) if flames else "none"))
    if gas_ppm is not None:
        parts.append(f"gas {gas_ppm:.0f} ppm")
    if water_pct is not None:
        parts.append(f"water {water_pct:.0f}%")
    return " | ".join(parts)


def build_message(kind: str, status: str) -> tuple[str, bool]:
    """(text, wants_photo) for an alert kind: 'gas_sms', 'fire_call', 'tank_empty', 'mission_done'."""
    head = {"gas_sms": ("GAS DETECTED: the robot sent its SMS", False),
            "fire_call": ("FIRE ALERT: the robot is calling you", True),
            "tank_empty": ("TANK EMPTY: no water left", True),
            "mission_done": ("MISSION DONE: all fires are out", True)}[kind]
    return f"{head[0]}\n{status}", head[1]


class RateLimiter:
    """At most one message per kind every `per_kind_s`, and at most `max_total` messages per `window_s` overall.
    Suppressed alerts are counted and reported with the next message of that kind."""

    def __init__(self, per_kind_s: float = 60.0, max_total: int = 10, window_s: float = 600.0):
        self.per_kind_s, self.max_total, self.window_s = per_kind_s, max_total, window_s
        self.last: dict[str, float] = {}
        self.sent: list[float] = []
        self.suppressed: dict[str, int] = {}

    def allow(self, kind: str, now: float) -> bool:
        self.sent = [t for t in self.sent if now - t < self.window_s]
        if (kind in self.last and now - self.last[kind] < self.per_kind_s) or len(self.sent) >= self.max_total:
            self.suppressed[kind] = self.suppressed.get(kind, 0) + 1
            return False
        self.last[kind] = now
        self.sent.append(now)
        return True

    def take_suppressed(self, kind: str) -> int:
        return self.suppressed.pop(kind, 0)

    def rearm(self, *kinds: str):
        for k in kinds:
            self.last.pop(k, None)


class TelegramError(Exception):
    pass


class TelegramClient:
    def __init__(self, token: str, chat_id: str, timeout: float = 15.0, opener=urllib.request.urlopen):
        self._token, self._chat, self.timeout, self._open = token, str(chat_id), timeout, opener

    def _clean(self, text) -> str:
        return str(text).replace(self._token, "<token>")

    def _call(self, method: str, data: bytes, content_type: str):
        req = urllib.request.Request(f"{API}/bot{self._token}/{method}", data=data, headers={"Content-Type": content_type})
        try:
            with self._open(req, timeout=self.timeout) as resp:
                body = json.loads(resp.read().decode())
        except urllib.error.HTTPError as e:           # the URL (with the token) is in e.url / str(e): report the status only
            raise TelegramError(f"Telegram answered HTTP {e.code} ({'token or chat id rejected' if e.code in (401, 403, 404) else 'error'})") from None
        except Exception as e:
            raise TelegramError(f"could not reach Telegram: {type(e).__name__}") from None
        if not body.get("ok"):
            raise TelegramError(f"Telegram refused the message: {self._clean(body.get('description', '?'))}")

    def send_message(self, text: str):
        self._call("sendMessage", json.dumps({"chat_id": self._chat, "text": text}).encode(), "application/json")

    def send_photo(self, jpeg: bytes, caption: str):
        boundary = uuid.uuid4().hex
        parts = []
        for name, value in (("chat_id", self._chat), ("caption", caption)):
            parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n{value}\r\n'.encode())
        parts.append(f'--{boundary}\r\nContent-Disposition: form-data; name="photo"; filename="frame.jpg"\r\n'
                     f"Content-Type: image/jpeg\r\n\r\n".encode() + jpeg + b"\r\n")
        parts.append(f"--{boundary}--\r\n".encode())
        self._call("sendPhoto", b"".join(parts), f"multipart/form-data; boundary={boundary}")
