#!/usr/bin/env python3
"""Unit tests for sim/real contract helpers (no rclpy)."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from parity_lib import (  # noqa: E402
    clamp,
    extract_named_positions,
    imu_from_odometry,
    integrate_dps,
    joint_value_to_rad,
    pose_xy_yaw_error,
    world_name_from_sdf,
    wrap_pi,
)


class ParityLibTest(unittest.TestCase):
    def test_clamp(self):
        self.assertEqual(clamp(2.0, -1.0, 1.0), 1.0)
        self.assertEqual(clamp(-2.0, -1.0, 1.0), -1.0)

    def test_joint_value_to_rad(self):
        self.assertAlmostEqual(joint_value_to_rad(0.5), 0.5)
        self.assertAlmostEqual(joint_value_to_rad(90.0), math.radians(90.0))

    def test_integrate_dps_hits_limit(self):
        pos = integrate_dps(0.0, 180.0, 1.0, -math.pi, math.pi)
        self.assertAlmostEqual(pos, math.pi)

    def test_extract_named(self):
        got = extract_named_positions(
            ["FL_joint", "pan_joint", "tilt_joint"],
            [0.1, 0.2, -0.3],
            ("pan_joint", "tilt_joint"),
        )
        self.assertEqual(got["pan_joint"], 0.2)
        self.assertEqual(got["tilt_joint"], -0.3)

    def test_pose_error(self):
        dx, dy, dyaw, dist = pose_xy_yaw_error((0.0, 0.0, 0.0), (3.0, 4.0, math.pi))
        self.assertAlmostEqual(dx, 3.0)
        self.assertAlmostEqual(dy, 4.0)
        self.assertAlmostEqual(dist, 5.0)
        self.assertAlmostEqual(abs(dyaw), math.pi, places=6)

    def test_wrap_pi(self):
        self.assertAlmostEqual(wrap_pi(3 * math.pi), math.pi, places=6)

    def test_imu_from_odom(self):
        packed = imu_from_odometry(
            qw=1.0,
            qx=0.0,
            qy=0.0,
            qz=0.0,
            wz=0.2,
            vx=1.0,
            vy=0.0,
            prev_vx=0.0,
            prev_vy=0.0,
            dt=0.5,
        )
        self.assertAlmostEqual(packed["angular_velocity"][2], 0.2)
        self.assertAlmostEqual(packed["linear_acceleration"][0], 2.0)
        self.assertAlmostEqual(packed["linear_acceleration"][2], 9.80665)

    def test_world_name_from_sdf(self):
        sdf = "<sdf><world name='small_house'></world></sdf>"
        self.assertEqual(world_name_from_sdf(sdf, "fallback"), "small_house")
        self.assertEqual(world_name_from_sdf("<sdf/>", "fallback"), "fallback")
        world = Path(__file__).resolve().parents[1] / "worlds" / "small_house.sdf"
        if world.is_file():
            self.assertEqual(
                world_name_from_sdf(world.read_text(encoding="utf-8"), "x"),
                "small_house",
            )

    def test_bridge_placeholders(self):
        yaml = Path(__file__).resolve().parents[1] / "config" / "bridge.yaml"
        text = yaml.read_text(encoding="utf-8")
        self.assertIn("__WORLD__", text)
        self.assertIn("__MODEL__", text)
        self.assertNotIn("/world/small_house/model/tank_sim/joint_state", text)


if __name__ == "__main__":
    unittest.main()
