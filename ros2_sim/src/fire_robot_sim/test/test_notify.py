"""Telegram alerts: texts, rate limiting, client behaviour and that the token never leaks."""
import io
import json
import urllib.error

import pytest

from fire_robot_sim.core.notify import RateLimiter, TelegramClient, TelegramError, build_message, status_line

TOKEN = "123456:SECRET-token"


class Resp(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *a):
        pass


def test_status_line():
    assert status_line(1.0, ["L", "F"], 1500.0, 78.2) == "CNN 100% | flame: L F | gas 1500 ppm | water 78%"
    assert status_line(None, [], None, None) == "flame: none"


def test_messages_and_photos():
    assert build_message("fire_call", "s")[1] is True and "calling" in build_message("fire_call", "s")[0]
    assert build_message("gas_sms", "s")[1] is False
    assert build_message("tank_empty", "x")[0].startswith("TANK EMPTY")
    assert build_message("mission_done", "x")[0].startswith("MISSION DONE")


def test_rate_limit_per_kind_and_total():
    r = RateLimiter(per_kind_s=60, max_total=3, window_s=600)
    assert r.allow("fire_call", 0) and not r.allow("fire_call", 23) and not r.allow("fire_call", 46)
    assert r.take_suppressed("fire_call") == 2 and r.take_suppressed("fire_call") == 0
    assert r.allow("fire_call", 61)
    assert r.allow("gas_sms", 62)
    assert not r.allow("tank_empty", 63)              # 3 messages in the window already
    assert r.allow("tank_empty", 700)                 # window passed
    r.rearm("tank_empty")
    assert r.allow("tank_empty", 701)


def test_send_message_and_photo_requests():
    seen = []

    def opener(req, timeout):
        seen.append(req)
        return Resp(b'{"ok": true}')
    c = TelegramClient(TOKEN, 42, opener=opener)
    c.send_message("hello")
    c.send_photo(b"\xff\xd8jpegbytes", "cap")
    assert seen[0].full_url.endswith("/sendMessage") and json.loads(seen[0].data) == {"chat_id": "42", "text": "hello"}
    assert seen[1].full_url.endswith("/sendPhoto") and b"jpegbytes" in seen[1].data and b'name="caption"' in seen[1].data
    assert seen[1].get_header("Content-type").startswith("multipart/form-data; boundary=")


@pytest.mark.parametrize("exc", [urllib.error.HTTPError(f"https://api.telegram.org/bot{TOKEN}/sendMessage", 401, "x", {}, None),
                                 urllib.error.URLError(f"cannot reach https://api.telegram.org/bot{TOKEN}/")])
def test_errors_never_contain_the_token(exc):
    def opener(req, timeout):
        raise exc
    with pytest.raises(TelegramError) as e:
        TelegramClient(TOKEN, 1, opener=opener).send_message("x")
    assert "SECRET" not in str(e.value) and TOKEN not in str(e.value)


def test_telegram_refusal_text_is_cleaned():
    c = TelegramClient(TOKEN, 1, opener=lambda r, timeout: Resp(json.dumps({"ok": False, "description": f"bad {TOKEN}"}).encode()))
    with pytest.raises(TelegramError) as e:
        c.send_message("x")
    assert TOKEN not in str(e.value)
