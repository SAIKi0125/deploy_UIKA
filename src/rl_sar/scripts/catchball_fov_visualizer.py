#!/usr/bin/env python3
#
# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import math
from dataclasses import dataclass
from threading import Lock

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from rclpy.node import Node


@dataclass(frozen=True)
class CylinderPose:
    midpoint: tuple[float, float, float]
    rpy: tuple[float, float, float]
    length: float


def is_robot_base_link_name(name: str, robot_model_name: str) -> bool:
    return (
        (robot_model_name and name == f"{robot_model_name}::base")
        or name == "robot_model::base"
        or name == "go2w_gazebo::base"
    )


def fov_corners_base(
    cam_offset_base=(0.0, 0.0, 0.20),
    tan_hfov_half=1.2109424,
    tan_vfov_half=0.6808758,
    dist_min=0.5,
    dist_max=4.0,
):
    corners = []
    ray_norm = math.sqrt(tan_hfov_half * tan_hfov_half + tan_vfov_half * tan_vfov_half + 1.0)
    for distance in (dist_min, dist_max):
        z = distance / ray_norm
        x = tan_hfov_half * z
        y = tan_vfov_half * z
        corners.extend([
            (cam_offset_base[0] - x, cam_offset_base[1] - y, cam_offset_base[2] + z),
            (cam_offset_base[0] + x, cam_offset_base[1] - y, cam_offset_base[2] + z),
            (cam_offset_base[0] + x, cam_offset_base[1] + y, cam_offset_base[2] + z),
            (cam_offset_base[0] - x, cam_offset_base[1] + y, cam_offset_base[2] + z),
        ])
    return corners


def quaternion_from_z_axis(direction):
    dx, dy, dz = direction
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1.0e-9:
        return (1.0, 0.0, 0.0, 0.0)

    ux, uy, uz = dx / length, dy / length, dz / length
    dot = uz
    if dot < -0.999999:
        return (0.0, 1.0, 0.0, 0.0)

    qw = 1.0 + dot
    qx = -uy
    qy = ux
    qz = 0.0
    norm = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz)
    return (qw / norm, qx / norm, qy / norm, qz / norm)


def quaternion_to_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_arg = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_arg)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


def cylinder_pose_between(p0, p1):
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    midpoint = (
        0.5 * (p0[0] + p1[0]),
        0.5 * (p0[1] + p1[1]),
        0.5 * (p0[2] + p1[2]),
    )
    return CylinderPose(midpoint=midpoint, rpy=quaternion_to_rpy(quaternion_from_z_axis((dx, dy, dz))), length=length)


def make_fov_sdf(model_name="catchball_fov"):
    corners = fov_corners_base()
    edges = [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]
    visuals = []
    for edge_idx, (a, b) in enumerate(edges):
        pose = cylinder_pose_between(corners[a], corners[b])
        visuals.append(f"""
      <visual name="edge_{edge_idx}">
        <pose>{pose.midpoint[0]:.6f} {pose.midpoint[1]:.6f} {pose.midpoint[2]:.6f} {pose.rpy[0]:.6f} {pose.rpy[1]:.6f} {pose.rpy[2]:.6f}</pose>
        <geometry>
          <cylinder>
            <radius>0.015</radius>
            <length>{pose.length:.6f}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>0.0 1.0 0.35 0.65</ambient>
          <diffuse>0.0 1.0 0.35 0.65</diffuse>
          <emissive>0.0 0.4 0.12 1.0</emissive>
        </material>
        <transparency>0.25</transparency>
      </visual>""")

    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{model_name}">
    <static>false</static>
    <link name="fov_link">
      <gravity>false</gravity>
      <self_collide>false</self_collide>
      <kinematic>true</kinematic>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>0.000001</ixx>
          <iyy>0.000001</iyy>
          <izz>0.000001</izz>
        </inertia>
      </inertial>
{''.join(visuals)}
    </link>
  </model>
</sdf>
"""


class CatchballFovVisualizer(Node):
    def __init__(self):
        super().__init__("catchball_fov_visualizer")
        self.declare_parameter("robot_model_name", "go2w_gazebo")
        self.declare_parameter("enabled", True)
        self.robot_model_name = self.get_parameter("robot_model_name").get_parameter_value().string_value
        self.enabled = self.get_parameter("enabled").get_parameter_value().bool_value

        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.state_client = self.create_client(SetEntityState, "/set_entity_state")
        self.link_states_sub = self.create_subscription(LinkStates, "/link_states", self.on_link_states, 10)
        self.timer = self.create_timer(0.05, self.on_timer)
        self.state_lock = Lock()
        self.latest_base_pose = None
        self.spawn_requested = False
        self.spawned = False
        self.model_name = "catchball_fov"

    def on_link_states(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            if is_robot_base_link_name(name, self.robot_model_name):
                with self.state_lock:
                    self.latest_base_pose = pose
                return

    def on_timer(self):
        if not self.enabled:
            return
        if not self.spawned:
            self.spawn_model_once()
            return
        with self.state_lock:
            pose = self.latest_base_pose
        if pose is None or not self.state_client.service_is_ready():
            return
        self.send_state(pose)

    def spawn_model_once(self):
        if self.spawn_requested or not self.spawn_client.service_is_ready():
            self.spawn_client.wait_for_service(timeout_sec=0.0)
            return

        request = SpawnEntity.Request()
        request.name = self.model_name
        request.xml = make_fov_sdf(self.model_name)
        request.robot_namespace = ""
        request.reference_frame = "world"
        request.initial_pose.orientation.w = 1.0

        self.spawn_requested = True
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.on_spawn_done)

    def on_spawn_done(self, future):
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Failed to spawn catchball FOV visualizer: {exc}")
            self.spawn_requested = False
            return

        self.spawned = result.success or "already" in result.status_message.lower()
        if not self.spawned:
            self.get_logger().warning(result.status_message)
            self.spawn_requested = False

    def send_state(self, pose):
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = self.model_name
        request.state.reference_frame = "world"
        request.state.pose = pose
        request.state.twist = Twist()
        self.state_client.call_async(request)


def main():
    rclpy.init()
    node = CatchballFovVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
