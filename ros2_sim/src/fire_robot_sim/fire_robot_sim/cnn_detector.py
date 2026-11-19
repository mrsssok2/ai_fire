"""CNN fire detector node: the user's trained Keras CNN (converted to ONNX), run with onnxruntime.

The model has ONE sigmoid output. With the 0..1 input scaling real fire photos give ~0.00, so the output is the
probability of the NON-fire class and fire confidence = 1 - output (parameter `invert`; see docs/cnn_model.md).
/camera/image_raw -> /fire/cnn_conf (Float32, whole frame) and /fire/detections (JSON boxes: HSV-proposed regions that
the CNN classifies as fire, each with its own confidence).
"""
import json
import os
import time

import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String

from fire_robot_sim.core.vision import cnn_input, fire_regions


def image_to_rgb(msg: Image) -> np.ndarray:
    return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)


class CnnDetector(Node):
    def __init__(self):
        super().__init__("cnn_detector")
        self.declare_parameter("model_path", "")
        self.declare_parameter("invert", True)
        self.declare_parameter("preprocess", "squash")
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("max_rate", 5.0)
        path = self.get_parameter("model_path").value or os.path.expanduser("~/ros2_sim/models/fire_cnn.onnx")
        opts = ort.SessionOptions()
        opts.intra_op_num_threads = 2                      # keep the CPU free for Gazebo
        self.sess = ort.InferenceSession(path, opts, providers=["CPUExecutionProvider"])
        self.conf_pub = self.create_publisher(Float32, "/fire/cnn_conf", 10)
        self.det_pub = self.create_publisher(String, "/fire/detections", 10)
        self.create_subscription(Image, "/camera/image_raw", self.on_image, 1)
        self.last = 0.0
        self.get_logger().info(f"CNN loaded: {path}")

    def fire_conf(self, rgb):
        out = float(self.sess.run(None, {"input": cnn_input(rgb, self.get_parameter("preprocess").value)})[0][0, 0])
        return 1.0 - out if self.get_parameter("invert").value else out

    def on_image(self, msg):
        if time.time() - self.last < 1.0 / self.get_parameter("max_rate").value:
            return
        self.last = time.time()
        rgb = image_to_rgb(msg)
        conf = self.fire_conf(rgb)
        boxes = []
        for x, y, w, h in fire_regions(rgb):
            c = self.fire_conf(rgb[y:y + h, x:x + w])
            if c >= self.get_parameter("threshold").value:
                boxes.append({"x": x, "y": y, "w": w, "h": h, "conf": round(c, 3)})
        self.conf_pub.publish(Float32(data=conf))
        self.det_pub.publish(String(data=json.dumps({"stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                                                     "conf": round(conf, 4), "boxes": boxes})))


def main():
    rclpy.init()
    node = CnnDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
