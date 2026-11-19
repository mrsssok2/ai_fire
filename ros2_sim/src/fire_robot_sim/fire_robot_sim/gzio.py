"""Thin wrapper over gz.transport13 for the few world-editing calls the fire/water nodes need."""
import math

WORLD = "fire_world"


def quat_from_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class GzIO:
    def __init__(self, world=WORLD, timeout_ms=400):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.particle_emitter_pb2 import ParticleEmitter
        from gz.msgs10.pose_v_pb2 import Pose_V
        from gz.transport13 import Node
        self._types = dict(Boolean=Boolean, ParticleEmitter=ParticleEmitter, Pose_V=Pose_V)
        self.node, self.world, self.timeout = Node(), world, timeout_ms
        self._pubs = {}

    def set_poses(self, items):
        """items: [(model_name, x, y, z, roll, pitch, yaw)] in one set_pose_vector call."""
        req = self._types["Pose_V"]()
        for name, x, y, z, r, p, yaw in items:
            m = req.pose.add()
            m.name = name
            m.position.x, m.position.y, m.position.z = x, y, z
            m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = quat_from_rpy(r, p, yaw)
        ok, rep = self.node.request(f"/world/{self.world}/set_pose_vector", req, self._types["Pose_V"], self._types["Boolean"], self.timeout)
        return ok and rep.data

    def emitter(self, topic, emitting=None, rate=None):
        pub = self._pubs.get(topic)
        if pub is None:
            pub = self._pubs[topic] = self.node.advertise(topic, self._types["ParticleEmitter"])
        m = self._types["ParticleEmitter"]()
        if emitting is not None:
            m.emitting.data = bool(emitting)
        if rate is not None:
            m.rate.data = float(rate)
        return pub.publish(m)
