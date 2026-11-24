import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

import robot_arm as ra
import robot_arm_gui as ragui
from robot_arm_gui import RobotArmApp, _same_pose


class RobotArmFacadeTests(unittest.TestCase):
    def test_joint4_is_the_canonical_wrist_roll_field(self):
        angles = ra.JointAngles(0.0, 0.0, 0.0, 0.0, 135.0)

        self.assertEqual(angles.joint4, 135.0)
        self.assertEqual(angles.gripper_roll, 135.0)

    def test_default_geometry_matches_robot_dimensions(self):
        config = ra.DEFAULT_CONFIG
        self.assertEqual(
            (config.l0, config.l1, config.l2, config.l3, config.finger_length),
            (100.0, 400.0, 300.0, 250.0, 150.0),
        )
        self.assertAlmostEqual(ra.GRIP_DEPTH, 124.0490014699)
        self.assertEqual(
            ra.DEFAULT_CONFIG.joint_limits,
            {
                "base": (-180.0, 180.0),
                "joint1": (-90.0, 90.0),
                "joint2": (-150.0, 150.0),
                "joint3": (-120.0, 120.0),
                "joint4": (-180.0, 180.0),
                "gripper_roll": (-180.0, 180.0),
            },
        )

    def test_two_finger_gripper_is_symmetric_and_keeps_arm_dimensions(self):
        config = ra.RobotConfig(gripper_fingers=2)
        self.assertEqual(
            (config.l0, config.l1, config.l2, config.l3, config.finger_length),
            (100.0, 400.0, 300.0, 250.0, 150.0),
        )
        angles = ra.JointAngles(35.0, 60.0, -25.0, -60.0)
        pose = (angles.theta_base, angles.joint1, angles.joint2, angles.joint3)
        two_finger_pose = ra.gripper_pose(*pose, config=config)
        three_finger_point = ra.forward_kinematics(
            angles.theta_base, angles.joint1, angles.joint2, angles.joint3
        )
        self.assertEqual(len(two_finger_pose.finger_paths), 2)
        self.assertEqual(len(two_finger_pose.fingertips), 2)
        self.assertEqual(two_finger_pose.grip_point, three_finger_point)

        closed = ra.gripper_pose(*pose, config=config, opening=0.0)
        for tip in closed.fingertips:
            self.assertLess(math.dist(tip.coords(), closed.grip_point.coords()), 1e-9)

        path_a, path_b = two_finger_pose.finger_paths
        for point_a, point_b in zip(path_a, path_b):
            midpoint_offset = tuple(
                point_a.coords()[i] + point_b.coords()[i]
                - 2.0 * two_finger_pose.gripper_mount.coords()[i]
                for i in range(3)
            )
            cross = (
                two_finger_pose.axis[1] * midpoint_offset[2]
                - two_finger_pose.axis[2] * midpoint_offset[1],
                two_finger_pose.axis[2] * midpoint_offset[0]
                - two_finger_pose.axis[0] * midpoint_offset[2],
                two_finger_pose.axis[0] * midpoint_offset[1]
                - two_finger_pose.axis[1] * midpoint_offset[0],
            )
            self.assertLess(math.sqrt(sum(value * value for value in cross)), 1e-9)

    def test_base_platform_circle_is_visual_only(self):
        angles = ra.JointAngles(0.0, 45.0, -20.0, -60.0)
        grip_before = ra.forward_kinematics(
            angles.theta_base, angles.joint1, angles.joint2, angles.joint3
        )
        axis = Mock()

        points = ra.draw_robot(axis, angles)

        grip_after = ra.forward_kinematics(
            angles.theta_base, angles.joint1, angles.joint2, angles.joint3
        )
        self.assertEqual(grip_after, grip_before)
        disk = next(
            call for call in axis.plot_surface.call_args_list
            if call.kwargs.get("label") == "base platform (R=50 mm)"
        )
        xs, ys, zs = disk.args
        self.assertEqual(xs.shape, (64, 12))
        self.assertEqual(ys.shape, (64, 12))
        self.assertEqual(zs.shape, (64, 12))
        radii = [math.hypot(float(x), float(y)) for x, y in zip(xs.flat, ys.flat)]
        self.assertTrue(math.isclose(max(radii), 50.0))
        self.assertEqual(min(radii), 0.0)
        self.assertTrue((zs == 0.0).all())
        self.assertEqual(disk.kwargs["alpha"], 0.45)
        self.assertTrue(
            any(
                math.isclose(point.x, -50.0, abs_tol=1e-12)
                and math.isclose(point.y, 0.0, abs_tol=1e-12)
                and math.isclose(point.z, 0.0, abs_tol=1e-12)
                for point in points
            )
        )

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
        target = ra.CartesianPoint(
            -812.2248514304025, -324.8667629845331, -95.55566192449189
        )

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

    def test_shoulder_and_wrist_limits_are_inclusive_and_enforced(self):
        for shoulder in (-90.0, 90.0):
            self.assertTrue(ra.check_joint_limits(0.0, shoulder, 0.0, 0.0))
        for wrist in (-120.0, 120.0):
            self.assertTrue(ra.check_joint_limits(0.0, 0.0, 0.0, wrist))
        self.assertFalse(ra.check_joint_limits(0.0, -90.001, 0.0, 0.0))
        self.assertFalse(ra.check_joint_limits(0.0, 90.001, 0.0, 0.0))
        self.assertFalse(ra.check_joint_limits(0.0, 0.0, 0.0, -120.001))
        self.assertFalse(ra.check_joint_limits(0.0, 0.0, 0.0, 120.001))

    def test_joint_limit_checks_reject_non_finite_angles(self):
        for joint_index in range(5):
            for value in (math.nan, math.inf, -math.inf):
                angles = [0.0] * 5
                angles[joint_index] = value
                with self.subTest(joint=joint_index, value=value):
                    self.assertFalse(ra.check_joint_limits(*angles))
                    with self.assertRaises(ValueError):
                        ra.requires_joint_limits(*angles)

    def test_roll_rotates_fingers_without_moving_the_grasp_point(self):
        angles = (35.0, 45.0, -20.0, -60.0)
        unrolled = ra.gripper_pose(*angles, gripper_roll=0.0)
        rolled = ra.gripper_pose(*angles, gripper_roll=90.0)

        self.assertEqual(unrolled.grip_point, rolled.grip_point)
        self.assertNotEqual(unrolled.fingertips, rolled.fingertips)
        self.assertEqual(rolled.gripper_roll, 90.0)

    def test_roll_limit_ik_preservation_and_shortest_path_interpolation(self):
        self.assertTrue(ra.check_joint_limits(0.0, 0.0, 0.0, 0.0, gripper_roll=180.0))
        self.assertFalse(ra.check_joint_limits(0.0, 0.0, 0.0, 0.0, gripper_roll=180.001))

        target = ra.forward_kinematics(45.0, 30.0, -30.0, -30.0)
        solved = ra.inverse_kinematics(
            target.x, target.y, target.z, joint3=-30.0, gripper_roll=135.0
        )
        self.assertEqual(solved.gripper_roll, 135.0)
        self.assertTrue(ra.check_joint_limits(
            solved.theta_base, solved.joint1, solved.joint2, solved.joint3,
            gripper_roll=solved.gripper_roll,
        ))

        frames = ra.interpolate_joints(
            ra.JointAngles(0.0, 0.0, 0.0, 0.0, 170.0),
            ra.JointAngles(0.0, 0.0, 0.0, 0.0, -170.0),
            frames=3,
        )
        self.assertEqual((frames[0].gripper_roll, frames[-1].gripper_roll), (170.0, -170.0))
        self.assertEqual(abs(frames[1].gripper_roll), 180.0)

    def test_base_interpolation_takes_the_shortest_way_round(self):
        frames = ra.interpolate_joints(
            ra.JointAngles(170.0, 0.0, 0.0, 0.0),
            ra.JointAngles(-170.0, 0.0, 0.0, 0.0),
            frames=5,
        )
        self.assertEqual(frames[0].theta_base, 170.0)
        self.assertEqual(frames[-1].theta_base, -170.0)
        self.assertTrue(all(abs(f.theta_base) >= 170.0 for f in frames))

    def test_animation_limits_include_the_base_without_wasting_space(self):
        angles = ra.JointAngles(45.0, 30.0, -20.0, -60.0)
        limits = ra.animation_limits((angles,), opening=1.0)

        self.assertEqual(len(limits), 3)
        self.assertTrue(all(lo <= 0.0 <= hi for lo, hi in limits))
        spans = [hi - lo for lo, hi in limits]
        self.assertTrue(all(math.isclose(s, spans[0]) for s in spans))
        self.assertTrue(any(not math.isclose(lo, -hi) for lo, hi in limits))

    def test_robot_view_draws_a_reusable_grid(self):
        captured = {}
        view = ra.RobotView.__new__(ra.RobotView)
        view._GRID_LINES = 3
        view._grid = SimpleNamespace(
            set_data_3d=lambda *arrays: captured.setdefault("arrays", arrays)
        )

        ra.RobotView._draw_grid(view, ((-100.0, 100.0),) * 3)

        xs, ys, zs = captured["arrays"]
        self.assertGreater(len(xs), 12)
        self.assertTrue(any(math.isnan(value) for value in xs))
        self.assertIn(0.0, xs)
        self.assertIn(0.0, ys)
        self.assertIn(0.0, zs)
        self.assertTrue(all(-100.0 <= value <= 100.0 or math.isnan(value)
                            for value in (*xs, *ys, *zs)))


class RobotArmGuiTests(unittest.TestCase):
    def test_roll_only_change_is_a_distinct_pose(self):
        start = ra.JointAngles(0.0, 0.0, 0.0, 0.0, 0.0)
        rolled = ra.JointAngles(0.0, 0.0, 0.0, 0.0, 90.0)

        self.assertFalse(_same_pose(start, rolled))
        self.assertTrue(_same_pose(start, start))

    def test_visualized_pose_updates_all_joint_angle_fields(self):
        app = RobotArmApp.__new__(RobotArmApp)
        for name in ("v_base", "v_j1", "v_j2", "v_j3_ang", "v_roll_ang", "v_open_a"):
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
        app.v_roll_ang.var.set.assert_called_once_with("0.000")
        app._draw.assert_called_once_with(angles, target, 0.5)

    def test_anim_step_jumps_to_the_frame_due_at_elapsed_time(self):
        frames = tuple(range(6))
        delays = []
        drawn = []
        app = SimpleNamespace(
            _anim_frames=frames,
            _anim_openings=tuple(float(i) for i in frames),
            _anim_stop_requested=False,
            _anim_start=0.0,
            _anim_tick=1,
            _anim_idx=0,
            _anim_job="old",
            _anim_target=None,
            _anim_limits=None,
            _draw=lambda *args: drawn.append(args),
            root=SimpleNamespace(after=lambda delay, callback: delays.append(delay) or "new"),
            viz_anim_btn=SimpleNamespace(configure=lambda **kwargs: None),
            viz_plot_btn=SimpleNamespace(configure=lambda **kwargs: None),
            _anim_finish=lambda: setattr(app, "_anim_job", None),
        )
        app._anim_step = lambda: RobotArmApp._anim_step(app)
        with patch.object(ragui.time, "perf_counter", side_effect=[0.05, 0.05]):
            RobotArmApp._anim_step(app)

        self.assertEqual([args[0] for args in drawn], [3])
        self.assertEqual(app._anim_idx, 4)
        self.assertEqual(app._anim_job, "new")
        self.assertEqual(len(delays), 1)
        self.assertAlmostEqual(delays[0], 14, delta=1)

    def test_slow_anim_step_finishes_instead_of_stretching_the_move(self):
        frames = tuple(range(6))
        drawn = []
        app = SimpleNamespace(
            _anim_frames=frames,
            _anim_openings=tuple(float(i) for i in frames),
            _anim_stop_requested=False,
            _anim_start=0.0,
            _anim_tick=1,
            _anim_idx=0,
            _anim_job="old",
            _anim_target=None,
            _anim_limits=None,
            _draw=lambda *args: drawn.append(args),
            root=SimpleNamespace(after=lambda delay, callback: self.fail("no more frames")),
            viz_anim_btn=SimpleNamespace(configure=lambda **kwargs: None),
            viz_plot_btn=SimpleNamespace(configure=lambda **kwargs: None),
            _anim_finish=lambda: setattr(app, "_anim_job", None),
        )
        app._anim_step = lambda: RobotArmApp._anim_step(app)
        with patch.object(ragui.time, "perf_counter", return_value=0.2):
            RobotArmApp._anim_step(app)

        self.assertEqual([args[0] for args in drawn], [5])
        self.assertEqual(app._anim_idx, 6)
        self.assertIsNone(app._anim_job)


if __name__ == "__main__":
    unittest.main()
