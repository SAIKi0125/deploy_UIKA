#!/usr/bin/env python3
#
# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import math
import pathlib
import random
import sys
import unittest


SCRIPTS_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from catchball_thrower import find_robot_base_pose, TrainingThrowSampler  # noqa: E402


class DummyPose:
    pass


class TestTrainingThrowSampler(unittest.TestCase):
    def test_find_robot_base_pose_prefers_named_base_link(self):
        expected = DummyPose()
        fallback = DummyPose()

        pose = find_robot_base_pose(
            ["go2w_gazebo::trunk", "go2w_gazebo::base", "robot_model::base"],
            [DummyPose(), expected, fallback],
            "go2w_gazebo",
        )

        self.assertIs(pose, expected)

    def test_samples_vary_and_remain_within_training_bounds(self):
        sampler = TrainingThrowSampler(random.Random(7))

        first = sampler.sample(robot_x=0.0, robot_y=0.0, robot_yaw=0.0)
        second = sampler.sample(robot_x=0.0, robot_y=0.0, robot_yaw=0.0)

        self.assertNotEqual(first.p0_world, second.p0_world)
        for sample in (first, second):
            self.assertGreaterEqual(sample.tc, 1.20)
            self.assertLessEqual(sample.tc, 1.55)
            self.assertGreaterEqual(sample.p0_world[2], 0.4)
            self.assertLessEqual(sample.p0_world[2], 2.2)

            speed = math.sqrt(sum(v * v for v in sample.v0_world))
            self.assertGreaterEqual(speed, 2.0)
            self.assertLessEqual(speed, 9.5)

            z_apex = sample.p0_world[2] + max(sample.v0_world[2], 0.0) ** 2 / (2.0 * 9.81)
            self.assertLessEqual(z_apex, 5.0)

    def test_seeded_sampling_is_reproducible(self):
        a = TrainingThrowSampler(random.Random(123)).sample(robot_x=1.0, robot_y=2.0, robot_yaw=0.25)
        b = TrainingThrowSampler(random.Random(123)).sample(robot_x=1.0, robot_y=2.0, robot_yaw=0.25)

        self.assertEqual(a.tc, b.tc)
        self.assertEqual(a.p0_world, b.p0_world)
        self.assertEqual(a.v0_world, b.v0_world)


if __name__ == "__main__":
    unittest.main()
