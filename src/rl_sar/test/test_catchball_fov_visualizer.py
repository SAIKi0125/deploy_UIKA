#!/usr/bin/env python3
#
# Copyright (c) 2024-2025 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import math
import pathlib
import sys
import unittest


SCRIPTS_DIR = pathlib.Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from catchball_fov_visualizer import cylinder_pose_between, fov_corners_base  # noqa: E402


class TestCatchballFovVisualizer(unittest.TestCase):
    def test_fov_corners_use_training_camera_range_and_offset(self):
        corners = fov_corners_base(
            cam_offset_base=(0.0, 0.0, 0.20),
            tan_hfov_half=1.0,
            tan_vfov_half=0.5,
            dist_min=1.5,
            dist_max=3.0,
        )

        self.assertEqual(len(corners), 8)
        self.assertEqual(corners[0], (-1.0, -0.5, 1.2))
        self.assertEqual(corners[7], (-2.0, 1.0, 2.2))

    def test_cylinder_pose_connects_two_local_points(self):
        pose = cylinder_pose_between((0.0, 0.0, 0.2), (0.0, 0.0, 1.2))

        self.assertEqual(pose.midpoint, (0.0, 0.0, 0.7))
        self.assertAlmostEqual(pose.length, 1.0)
        self.assertTrue(all(math.isfinite(value) for value in pose.rpy))


if __name__ == "__main__":
    unittest.main()
