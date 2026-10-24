import math
import unittest
from unittest.mock import Mock

import robot_arm as ra
from robot_arm_gui import RobotArmApp


class RobotArmFacadeTests(unittest.TestCase):
    def test_default_geometry_matches_robot_dimensions(self):
        config = ra.DEFAULT_CONFIG
        self.assertEqual(
            (config.l0, config.l1, config.l2, config.l3, config.finger_length),
            (100.0, 500.0, 400.0, 80.0, 150.0),
        )
        self.assertAlmostEqual(ra.GRIP_DEPTH, 124.0490014699)

    def test_facade_forwards_four_joint_kinematics(self):
        config = ra.RobotConfig(l0=55.0)
        arm = ra.RobotArm(config)
        pose = (30.0, 45.0, -40.0, -60.0)

        self.assertEqual(
            arm.forward_kinematics(*pose, opening=0.5),
            ra.forward_kinematics(*pose, config=config, opening=0.5),
        )
        self.assertEqual(
            arm.gripper_pose(*pose, opening=0.5),
            ra.gripper_pose(*pose, config=config, opening=0.5),
        )

        target = arm.forward_kinematics(*pose)
        self.assertEqual(
            arm.inverse_kinematics(
                target.x, target.y, target.z, joint3=-60.0
            ),
            ra.inverse_kinematics(
                target.x, target.y, target.z, config=config, joint3=-60.0
            ),
        )
        self.assertEqual(
            arm.is_reachable(target.x, target.y, target.z, joint3=-60.0),
            ra.is_reachable(
                target.x, target.y, target.z, config, math.radians(-60.0)
            ),
        )
        self.assertTrue(arm.check_joint_limits(*pose))

    def test_auto_joint3_preserves_requested_elbow_branch(self):
        target = ra.forward_kinematics(0.0, 30.0, 30.0, -60.0)
        expected = ra.inverse_kinematics(
            target.x, target.y, target.z, joint2_up=False, joint3=-60.0
        )

        result = ra.inverse_kinematics(
            target.x, target.y, target.z, joint2_up=False, joint3="auto"
        )

        self.assertEqual(result, expected)

    def test_auto_joint3_uses_direct_signed_base_yaw_for_target(self):
        target = ra.CartesianPoint(-300.0, -120.0, 150.0)

        result = ra.inverse_kinematics(target.x, target.y, target.z, joint3="auto")
        expected = ra.inverse_kinematics(target.x, target.y, target.z, joint3=-60.0)
        rebuilt = ra.forward_kinematics(
            result.theta_base, result.joint1, result.joint2, result.joint3
        )

        self.assertEqual(result, expected)
        self.assertTrue(
            ra.check_joint_limits(
                result.theta_base, result.joint1, result.joint2, result.joint3
            )
        )
        self.assertLess(math.dist(target.coords(), rebuilt.coords()), 0.1)

    def test_negative_y_targets_use_signed_base_yaw(self):
        target = ra.CartesianPoint(-300.0, -120.0, 150.0)
        for joint3 in (-60.0, "auto"):
            angles = ra.inverse_kinematics(
                target.x, target.y, target.z, joint3=joint3
            )
            rebuilt = ra.forward_kinematics(
                angles.theta_base, angles.joint1, angles.joint2,
                angles.joint3,
            )
            expected_yaw = math.degrees(math.atan2(target.y, target.x))
            self.assertAlmostEqual(angles.theta_base, expected_yaw)
            self.assertLess(
                math.dist(target.coords(), rebuilt.coords()), 0.1
            )

    def test_base_limits_include_signed_full_rotation(self):
        for base in (-180.0, -90.0, 0.0, 90.0, 180.0):
            self.assertTrue(
                ra.check_joint_limits(base, 90.0, 0.0, -60.0)
            )
        for base in (-180.001, 180.001):
            self.assertFalse(
                ra.check_joint_limits(base, 90.0, 0.0, -60.0)
            )

    def test_joint_limit_checks_reject_non_finite_angles(self):
        for joint_index in range(4):
            for value in (math.nan, math.inf, -math.inf):
                angles = [0.0] * 4
                angles[joint_index] = value
                with self.subTest(joint=joint_index, value=value):
                    self.assertFalse(ra.check_joint_limits(*angles))
                    with self.assertRaises(ValueError):
                        ra.requires_joint_limits(*angles)


class RobotArmGuiTests(unittest.TestCase):
    def test_visualized_pose_updates_all_joint_angle_fields(self):
        app = RobotArmApp.__new__(RobotArmApp)
        for name in ("v_base", "v_j1", "v_j2", "v_j3_ang", "v_open_a"):
            setattr(app, name, Mock())
            getattr(app, name).var = Mock()
        app.viz_mode = Mock()
        app._toggle_viz = Mock()
        app.notebook = Mock()
        app.viz_tab_index = 4
        app._draw = Mock()

        angles = ra.JointAngles(15.0, 25.0, 35.0, -45.0)
        target = ra.CartesianPoint(1.0, 2.0, 3.0)
        app._goto_viz_angles(angles, 0.5, target)

        app.v_j3_ang.var.set.assert_called_once_with("-45.000")
        app._draw.assert_called_once_with(angles, target, 0.5)


if __name__ == "__main__":
    unittest.main()
