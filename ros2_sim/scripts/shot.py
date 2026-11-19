#!/usr/bin/env python3
"""Move the fixed docs camera (needs `shot_camera:=true`) and save a PNG.

    shot.py out.png X Y Z ROLL PITCH YAW [topic=/shot/image]
"""
import sys
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from sensor_msgs.msg import Image

sys.path.insert(0, __import__("os").path.expanduser("~/ros2_sim/src/fire_robot_sim"))
from fire_robot_sim.gzio import quat_from_rpy  # noqa: E402

out, (x, y, z, r, p, yaw) = sys.argv[1], [float(v) for v in sys.argv[2:8]]
topic = sys.argv[8] if len(sys.argv) > 8 else "/shot/image"
req = Pose_V()
m = req.pose.add()
m.name = "shot_camera"
m.position.x, m.position.y, m.position.z = x, y, z
m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = quat_from_rpy(r, p, yaw)
gz = GzNode()
for _ in range(5):
    ok, rep = gz.request("/world/fire_world/set_pose_vector", req, Pose_V, Boolean, 1000)
    if ok and rep.data:
        break
rclpy.init()
node = Node("shot")
got = []
node.create_subscription(Image, topic, got.append, 1)
t0 = time.time()
need = int(__import__('os').environ.get('SHOT_FRAMES', '6'))
while len(got) < need and time.time() - t0 < 30:        # skip frames until the new pose + the particles have settled
    rclpy.spin_once(node, timeout_sec=0.3)
cv2.imwrite(out, cv2.cvtColor(CvBridge().imgmsg_to_cv2(got[-1], "rgb8"), cv2.COLOR_RGB2BGR))
