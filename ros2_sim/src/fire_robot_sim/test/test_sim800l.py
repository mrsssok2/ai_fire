from fire_robot_sim.core.sim800l import Sim800l


def test_sms_needs_text_mode_and_ctrl_z():
    m = Sim800l()
    assert m.feed('AT+CMGS="+91111"\r', 0) == []        # not in text mode: ignored
    m.feed("AT+CMGF=1\r", 1)
    assert m.feed('AT+CMGS="+91111"\r', 2) == []
    assert m.feed("Fire Alert", 3) == []
    (ev,) = m.feed("\x1a", 4)
    assert ev == {"type": "sms", "to": "+91111", "text": "Fire Alert", "t": 4}


def test_call_and_hangup():
    m = Sim800l()
    assert m.feed("ATD+91222;\r\n", 10)[0]["type"] == "call_start"
    (end,) = m.feed("ATH\r\n", 31)
    assert end["duration_s"] == 21 and end["to"] == "+91222"
    assert m.feed("ATH\r\n", 40) == []                  # hang-up without a call


def test_plain_at_is_harmless():
    assert Sim800l().feed("AT\r\n", 0) == []
