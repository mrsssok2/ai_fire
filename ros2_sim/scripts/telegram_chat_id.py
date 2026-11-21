#!/usr/bin/env python3
"""Print your Telegram chat id. Needs only Python 3 (no packages) and the bot token in the environment:

    export TELEGRAM_BOT_TOKEN='123456:ABC...'      # from @BotFather (type it in your terminal, do not put it in a file)
    # open your bot in Telegram and press Start (or send it any message), then:
    python3 ros2_sim/scripts/telegram_chat_id.py

The token is never printed or stored. Only the chat id, name and type of the chats that wrote to the bot are shown."""
import json
import os
import sys
import urllib.error
import urllib.request

token = os.environ.get("TELEGRAM_BOT_TOKEN", "").strip()
if not token:
    sys.exit("TELEGRAM_BOT_TOKEN is not set in the environment (export it first).")
try:
    with urllib.request.urlopen(f"https://api.telegram.org/bot{token}/getUpdates", timeout=15) as r:
        data = json.loads(r.read().decode())
except urllib.error.HTTPError as e:
    sys.exit(f"Telegram answered HTTP {e.code}" + (": the token was rejected (check it)." if e.code in (401, 404) else "."))
except Exception as e:
    sys.exit(f"Could not reach Telegram ({type(e).__name__}).")
chats = {}
for u in data.get("result", []):
    m = u.get("message") or u.get("channel_post") or u.get("my_chat_member") or {}
    c = m.get("chat")
    if c:
        chats[c["id"]] = c
if not chats:
    sys.exit("No messages yet: open your bot in Telegram, press Start (or send 'hi'), then run this again.")
for cid, c in chats.items():
    name = c.get("title") or " ".join(filter(None, [c.get("first_name"), c.get("last_name")])) or c.get("username", "")
    print(f"chat id: {cid}   ({c.get('type')}, {name})")
print("\nexport TELEGRAM_CHAT_ID=<the number above>")
