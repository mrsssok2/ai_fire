import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from fire_robot_sim.core import overlay
from fire_robot_sim.core.vision import cnn_input, fire_regions


def frame_with_flame():
    img = np.full((240, 320, 3), (190, 205, 225), np.uint8)             # pale sky/floor
    img[80:160, 100:140] = (255, 120, 20)                                # orange-yellow blob
    return img


def test_hsv_proposals_find_the_flame_and_ignore_the_background():
    boxes = fire_regions(frame_with_flame())
    assert len(boxes) == 1
    x, y, w, h = boxes[0]
    assert x <= 100 and y <= 80 and x + w >= 140 and y + h >= 160
    assert fire_regions(np.full((240, 320, 3), (190, 205, 225), np.uint8)) == []


def test_cnn_input_is_nhwc_float_0_1_for_both_resize_modes():
    for mode in ("squash", "fit"):
        a = cnn_input(frame_with_flame(), mode)
        assert a.shape == (1, 224, 224, 3) and a.dtype == np.float32 and 0.0 <= a.min() and a.max() <= 1.0


STATUS = {"flame": {"left": False, "front": True, "right": False}, "analog": {"left": 0, "front": 1.5, "right": 0}, "gas": False,
          "ppm": 12.0, "water": 48.0, "ml": 242.0, "pump": True, "servo": 100.0, "alert": 'SMS to +91: "Gas Detected"'}


def test_window_layout_and_banner_colours():
    fire = overlay.compose(frame_with_flame(), 0.93, [{"x": 100, "y": 80, "w": 40, "h": 80, "conf": 0.9}], STATUS)
    calm = overlay.compose(frame_with_flame(), 0.12, [], STATUS)
    assert fire.shape == (536, 960, 3) and calm.shape == fire.shape
    assert tuple(fire[5, 600]) == overlay.RED and tuple(calm[5, 600]) == overlay.GREEN        # banner bar, right of the text
    assert not np.array_equal(fire, calm)


def test_banner_threshold():
    img = np.zeros((100, 400, 3), np.uint8)
    assert overlay.banner(img, 0.5) is True and overlay.banner(img, 0.49) is False


def test_mission_banner_and_buttons():
    waiting = overlay.compose(frame_with_flame(), 0.1, [], STATUS, {"state": "WAITING", "countdown": 12.0, "message": "MISSION STARTS IN 12 s"})
    done = overlay.compose(frame_with_flame(), 0.1, [], STATUS, {"state": "DONE", "countdown": 0, "message": "MISSION DONE: all fires out"})
    empty = overlay.compose(frame_with_flame(), 0.1, [], STATUS, {"state": "TANK_EMPTY", "countdown": 0, "message": "TANK EMPTY: no water left"})
    running = overlay.compose(frame_with_flame(), 0.1, [], STATUS, {"state": "RUNNING", "countdown": 0, "message": ""})
    assert tuple(waiting[60, 600]) == overlay.AMBER and tuple(done[60, 600]) == overlay.GREEN and tuple(empty[60, 600]) == overlay.RED
    assert tuple(running[60, 600]) != overlay.AMBER                                 # no mission banner while running
    assert tuple(waiting[480 + 20, 100]) == overlay.GREEN                           # START button lit while waiting
    assert tuple(running[480 + 20, 100]) != overlay.GREEN


def test_button_hit_test():
    assert overlay.button_at(100, 480 + 20, 480) == "start"
    assert overlay.button_at(500, 480 + 20, 480) == "reset"
    assert overlay.button_at(100, 100, 480) is None and overlay.button_at(320, 480 + 20, 480) is None
