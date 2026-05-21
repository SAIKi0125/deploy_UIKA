#!/usr/bin/env python3
"""Publish a fixed UIKA sensor frame and verify rl_real_uika emits commands."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from interfaces.msg import MotorCommand12, MotorFeedback12
from rclpy.node import Node
from sensor_msgs.msg import Imu


FIELD_NAMES = (
    "fl_hip", "fl_thigh", "fl_calf",
    "fr_hip", "fr_thigh", "fr_calf",
    "rl_hip", "rl_thigh", "rl_calf",
    "rr_hip", "rr_thigh", "rr_calf",
)

DEFAULT_DOF_POS = (
    -0.80, 0.05, 0.70,
     0.80, -0.05, -0.70,
     0.75, 0.05, 0.70,
    -0.75, -0.05, -0.70,
)

CALF_INDICES = {2, 5, 8, 11}


class UikaGoldenFrame(Node):
    def __init__(self, args):
        super().__init__("fake_uika_golden_frame")
        self.args = args
        self.command_count = 0
        self.last_command = None
        self.start_time = time.monotonic()
        self.last_report = self.start_time

        self.feedback_pub = self.create_publisher(
            MotorFeedback12, args.motor_feedback_topic, 10)
        self.imu_pub = self.create_publisher(Imu, args.imu_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.xbox_vel_pub = self.create_publisher(Twist, args.xbox_vel_topic, 10)
        self.command_sub = self.create_subscription(
            MotorCommand12, args.motor_command_topic, self.command_callback, 10)

        period = 1.0 / args.rate_hz
        self.timer = self.create_timer(period, self.publish_frame)
        self.get_logger().info(
            f"publishing fake frame: feedback={args.motor_feedback_topic} "
            f"imu={args.imu_topic}, listening command={args.motor_command_topic}")

    def command_callback(self, msg):
        self.command_count += 1
        self.last_command = msg
        if self.command_count == 1:
            values = self.command_values(msg)
            self.get_logger().info(
                f"first motor command received: q0={values['position'][0]:.4f} "
                f"kp0={values['kp'][0]:.2f} kd0={values['kd'][0]:.2f}")

    def publish_frame(self):
        now = self.get_clock().now().to_msg()

        feedback = MotorFeedback12()
        feedback.header.stamp = now
        feedback.header.frame_id = "fake_uika_golden_frame"
        for i, name in enumerate(FIELD_NAMES):
            motor_feedback = getattr(feedback, name)
            position = DEFAULT_DOF_POS[i]
            if i in CALF_INDICES:
                position *= self.args.calf_gear_ratio
            motor_feedback.position = float(position)
            motor_feedback.velocity = 0.0
            motor_feedback.torque = 0.0
            motor_feedback.temperature = 25.0
        self.feedback_pub.publish(feedback)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = "imu_link"
        imu.orientation.w = 1.0
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = 0.0
        imu.angular_velocity.x = 0.0
        imu.angular_velocity.y = 0.0
        imu.angular_velocity.z = 0.0
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 9.81
        self.imu_pub.publish(imu)

        zero_cmd = Twist()
        self.cmd_vel_pub.publish(zero_cmd)
        self.xbox_vel_pub.publish(zero_cmd)

        now_mono = time.monotonic()
        if now_mono - self.last_report >= self.args.report_period:
            self.last_report = now_mono
            self.get_logger().info(f"motor commands received: {self.command_count}")

    @staticmethod
    def command_values(msg):
        values = {"position": [], "velocity": [], "torque": [], "kp": [], "kd": []}
        for name in FIELD_NAMES:
            cmd = getattr(msg, name)
            values["position"].append(float(cmd.position))
            values["velocity"].append(float(cmd.velocity))
            values["torque"].append(float(cmd.torque))
            values["kp"].append(float(cmd.kp))
            values["kd"].append(float(cmd.kd))
        return values

    def command_is_valid(self):
        if self.last_command is None:
            return False
        values = self.command_values(self.last_command)
        for series in values.values():
            if any(not math.isfinite(v) for v in series):
                return False
        return True


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--rate-hz", type=float, default=200.0)
    parser.add_argument("--min-commands", type=int, default=5)
    parser.add_argument("--report-period", type=float, default=1.0)
    parser.add_argument("--calf-gear-ratio", type=float, default=1.0)
    parser.add_argument("--motor-command-topic", default="/motor_command")
    parser.add_argument("--motor-feedback-topic", default="/motor_feedback")
    parser.add_argument("--imu-topic", default="/imu/data")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--xbox-vel-topic", default="/xbox_vel")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = UikaGoldenFrame(args)
    deadline = time.monotonic() + args.duration
    exit_code = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.command_count >= args.min_commands and node.command_is_valid():
                exit_code = 0
                break
        if exit_code == 0:
            node.get_logger().info(
                f"PASS: received {node.command_count} valid motor command frames")
        else:
            node.get_logger().error(
                f"FAIL: received {node.command_count}/{args.min_commands} motor command frames")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
