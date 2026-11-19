"""Pose helpers shared by the fire manager and the water system (pure Python)."""
import math


def nozzle_pose(robot_pose, nozzle_xy, joint_angle, length):
    """World pose of the nozzle pivot and tip.

    robot_pose: (x, y, yaw) of base_link; nozzle_xy: servo axis in the robot frame; joint_angle: nozzle_servo_joint (rad,
    positive = left). Returns ((px, py), (tx, ty), jet_yaw).
    """
    rx, ry, ryaw = robot_pose
    c, s = math.cos(ryaw), math.sin(ryaw)
    px, py = rx + c * nozzle_xy[0] - s * nozzle_xy[1], ry + s * nozzle_xy[0] + c * nozzle_xy[1]
    yaw = ryaw + joint_angle
    return (px, py), (px + length * math.cos(yaw), py + length * math.sin(yaw)), yaw
