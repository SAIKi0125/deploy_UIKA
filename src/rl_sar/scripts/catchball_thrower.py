#!/usr/bin/env python3
#
# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import math
import random
from dataclasses import dataclass
from threading import Lock

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.msg import LinkStates
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node


@dataclass(frozen=True)
class TrainingThrow:
    tc: float
    p0_world: tuple[float, float, float]
    v0_world: tuple[float, float, float]
    pc_world: tuple[float, float, float]
    pc_local: tuple[float, float]


def is_robot_base_link_name(name: str, robot_model_name: str) -> bool:
    return (
        (robot_model_name and name == f"{robot_model_name}::base")
        or name == "robot_model::base"
        or name == "go2w_gazebo::base"
    )


def find_robot_base_pose(names, poses, robot_model_name: str):
    for name, pose in zip(names, poses):
        if is_robot_base_link_name(name, robot_model_name):
            return pose
    return None


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class TrainingThrowSampler:
    def __init__(self, rng: random.Random):
        self.rng = rng
        self.g = 9.81
        self.zc_world = 0.6
        self.vx_max = 2.5
        self.speed_safety_factor = 0.7
        self.reaction_margin = 0.35
        self.t_min = 1.20
        self.t_max = 1.55
        self.x_min = 0.5
        self.x_cap = 2.0
        self.vh_min = 0.5
        self.vh_max = 3.5
        self.theta_min = -math.pi
        self.theta_max = math.pi
        self.z0_world_min = 0.4
        self.z0_world_max = 2.2
        self.vz_min = 1.5
        self.vz_max = 8.5
        self.x0_min = -4.0
        self.x0_max = 5.0
        self.y0_min = -3.5
        self.y0_max = 3.5
        self.speed_min = 2.0
        self.speed_max = 9.5
        self.z_apex_max = 5.0
        self.max_resample = 100

    def sample(self, robot_x: float, robot_y: float, robot_yaw: float) -> TrainingThrow:
        vx_eff = self.speed_safety_factor * self.vx_max
        for _ in range(self.max_resample):
            tc = self.rng.uniform(self.t_min, self.t_max)
            t_eff = tc - self.reaction_margin
            if t_eff <= 0.0:
                continue

            r_max = min(vx_eff * t_eff, self.x_cap)
            if r_max <= self.x_min:
                continue

            rc = self.rng.uniform(self.x_min, r_max)
            phi = self.rng.uniform(-math.pi, math.pi)
            xc = rc * math.cos(phi)
            yc = rc * math.sin(phi)

            vh = self.rng.uniform(self.vh_min, self.vh_max)
            theta = self.rng.uniform(self.theta_min, self.theta_max)
            x0 = xc - vh * tc * math.cos(theta)
            y0 = yc - vh * tc * math.sin(theta)

            z_low = max(
                self.z0_world_min,
                self.zc_world + 0.5 * self.g * tc * tc - self.vz_max * tc,
            )
            z_high = min(
                self.z0_world_max,
                self.zc_world + 0.5 * self.g * tc * tc - self.vz_min * tc,
            )
            if z_low >= z_high:
                continue

            z0 = self.rng.uniform(z_low, z_high)
            v0x = (xc - x0) / tc
            v0y = (yc - y0) / tc
            v0z = (self.zc_world - z0 + 0.5 * self.g * tc * tc) / tc

            if not (self.x0_min <= x0 <= self.x0_max):
                continue
            if not (self.y0_min <= y0 <= self.y0_max):
                continue
            if not (self.z0_world_min <= z0 <= self.z0_world_max):
                continue

            speed = math.sqrt(v0x * v0x + v0y * v0y + v0z * v0z)
            if not (self.speed_min <= speed <= self.speed_max):
                continue
            if not (self.vz_min <= v0z <= self.vz_max):
                continue

            z_apex = z0 + max(v0z, 0.0) ** 2 / (2.0 * self.g)
            if z_apex > self.z_apex_max:
                continue

            return self._to_world(robot_x, robot_y, robot_yaw, tc, x0, y0, z0, v0x, v0y, v0z, xc, yc)

        return self._fallback(robot_x, robot_y, robot_yaw)

    def _fallback(self, robot_x: float, robot_y: float, robot_yaw: float) -> TrainingThrow:
        tc = 1.35
        z0 = 1.2
        rc = 1.2
        phi = self.rng.uniform(-math.pi, math.pi)
        xc = rc * math.cos(phi)
        yc = rc * math.sin(phi)
        x0 = -1.0
        y0 = 0.0
        v0x = (xc - x0) / tc
        v0y = (yc - y0) / tc
        v0z = (self.zc_world - z0 + 0.5 * self.g * tc * tc) / tc
        return self._to_world(robot_x, robot_y, robot_yaw, tc, x0, y0, z0, v0x, v0y, v0z, xc, yc)

    def _to_world(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        tc: float,
        x0: float,
        y0: float,
        z0: float,
        v0x: float,
        v0y: float,
        v0z: float,
        xc: float,
        yc: float,
    ) -> TrainingThrow:
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        def rotate_xy(x_local, y_local):
            return (
                cos_yaw * x_local - sin_yaw * y_local,
                sin_yaw * x_local + cos_yaw * y_local,
            )

        p0_dx, p0_dy = rotate_xy(x0, y0)
        pc_dx, pc_dy = rotate_xy(xc, yc)
        v0_dx, v0_dy = rotate_xy(v0x, v0y)
        return TrainingThrow(
            tc=tc,
            p0_world=(robot_x + p0_dx, robot_y + p0_dy, z0),
            v0_world=(v0_dx, v0_dy, v0z),
            pc_world=(robot_x + pc_dx, robot_y + pc_dy, self.zc_world),
            pc_local=(xc, yc),
        )


class CatchballThrower(Node):
    def __init__(self):
        super().__init__("catchball_thrower")
        self.declare_parameter("robot_model_name", "go2w_gazebo")
        self.declare_parameter("random_seed", 0)
        self.declare_parameter("enabled", False)
        self.robot_model_name = self.get_parameter("robot_model_name").get_parameter_value().string_value
        seed = self.get_parameter("random_seed").get_parameter_value().integer_value
        self.enabled = self.get_parameter("enabled").get_parameter_value().bool_value
        self.sampler = TrainingThrowSampler(random.Random(seed) if seed >= 0 else random.Random())

        self.client = self.create_client(SetEntityState, "/set_entity_state")
        self.model_states_sub = self.create_subscription(ModelStates, "/model_states", self.on_model_states, 10)
        self.link_states_sub = self.create_subscription(LinkStates, "/link_states", self.on_link_states, 10)
        self.timer_period = 0.02
        self.wait_time = 0.8
        self.task_timeout_margin = 0.7
        self.ball_ground_z_thresh = 0.05
        self.gravity = 9.81
        self.state = "wait"
        self.state_start_time = self.get_clock().now()
        self.current_throw: TrainingThrow | None = None
        self.state_lock = Lock()
        self.has_ball = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.timer = self.create_timer(self.timer_period, self.on_timer)

    def on_model_states(self, msg):
        ball_seen = False
        for name in msg.name:
            if name == "catchball":
                ball_seen = True

        with self.state_lock:
            self.has_ball = ball_seen

    def on_link_states(self, msg):
        pose = find_robot_base_pose(msg.name, msg.pose, self.robot_model_name)
        if pose is None:
            return

        with self.state_lock:
            self.robot_x = pose.position.x
            self.robot_y = pose.position.y
            self.robot_yaw = yaw_from_quaternion(pose.orientation)

    def on_timer(self):
        if not self.client.service_is_ready():
            self.client.wait_for_service(timeout_sec=0.0)
            return

        with self.state_lock:
            if not self.has_ball:
                return
            robot_x = self.robot_x
            robot_y = self.robot_y
            robot_yaw = self.robot_yaw

        if not self.enabled:
            pose = self.make_pose(robot_x, robot_y, -10.0)
            twist = self.make_twist(0.0, 0.0, 0.0)
            self.send_state(pose, twist)
            return

        now = self.get_clock().now()
        elapsed = (now - self.state_start_time).nanoseconds * 1.0e-9

        if self.state == "wait":
            if elapsed < self.wait_time:
                pose = self.make_pose(robot_x, robot_y, -10.0)
                twist = self.make_twist(0.0, 0.0, 0.0)
                self.send_state(pose, twist)
                return
            self.current_throw = self.sampler.sample(robot_x, robot_y, robot_yaw)
            self.state = "flight"
            self.state_start_time = now
            elapsed = 0.0

        if self.current_throw is None:
            pose = self.make_pose(robot_x, robot_y, -10.0)
            twist = self.make_twist(0.0, 0.0, 0.0)
            self.send_state(pose, twist)
            return

        t = elapsed
        p0 = self.current_throw.p0_world
        v0 = self.current_throw.v0_world
        z = p0[2] + v0[2] * t - 0.5 * self.gravity * t * t

        if t > self.current_throw.tc + self.task_timeout_margin or z < self.ball_ground_z_thresh:
            self.state = "wait"
            self.state_start_time = now
            self.current_throw = None
            pose = self.make_pose(robot_x, robot_y, -10.0)
            twist = self.make_twist(0.0, 0.0, 0.0)
        else:
            pose = self.make_pose(
                p0[0] + v0[0] * t,
                p0[1] + v0[1] * t,
                z,
            )
            twist = self.make_twist(
                v0[0],
                v0[1],
                v0[2] - self.gravity * t,
            )

        self.send_state(pose, twist)

    def send_state(self, pose, twist):
        request = SetEntityState.Request()
        request.state = EntityState()
        request.state.name = "catchball"
        request.state.reference_frame = "world"
        request.state.pose = pose
        request.state.twist = twist
        self.client.call_async(request)

    @staticmethod
    def make_pose(x, y, z):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        return pose

    @staticmethod
    def make_twist(x, y, z):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        return twist


def main():
    rclpy.init()
    node = CatchballThrower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
