"""Minimal SIM800L emulator: parses the AT strings the robot writes and reports SMS / call events."""
from __future__ import annotations

import re

CTRL_Z = "\x1a"


class Sim800l:
    def __init__(self):
        self.text_mode = False
        self._buf = ""
        self._sms_to = None        # set between AT+CMGS and Ctrl-Z
        self._sms_body = ""
        self._call_to = None
        self._call_start = 0.0

    def feed(self, data: str, now: float) -> list[dict]:
        """Feed a chunk written to the UART; returns finished events: {'type': 'sms'|'call_start'|'call_end', ...}."""
        events = []
        for ch in data:
            if self._sms_to is not None:               # collecting the SMS body until Ctrl-Z
                if ch == CTRL_Z:
                    events.append({"type": "sms", "to": self._sms_to, "text": self._sms_body, "t": now})
                    self._sms_to, self._sms_body = None, ""
                else:
                    self._sms_body += ch
                continue
            self._buf += ch
            if ch in "\r\n" and self._buf.strip():
                events += self._command(self._buf.strip(), now)
                self._buf = ""
            elif ch in "\r\n":
                self._buf = ""
        return events

    def _command(self, cmd: str, now: float) -> list[dict]:
        if cmd == "AT+CMGF=1":
            self.text_mode = True
        elif m := re.fullmatch(r'AT\+CMGS="(.*)"', cmd):
            if self.text_mode:
                self._sms_to = m.group(1)
        elif m := re.fullmatch(r"ATD(.*);", cmd):
            self._call_to, self._call_start = m.group(1), now
            return [{"type": "call_start", "to": self._call_to, "t": now}]
        elif cmd == "ATH" and self._call_to is not None:
            ev = {"type": "call_end", "to": self._call_to, "t": now, "duration_s": round(now - self._call_start, 2)}
            self._call_to = None
            return [ev]
        return []
