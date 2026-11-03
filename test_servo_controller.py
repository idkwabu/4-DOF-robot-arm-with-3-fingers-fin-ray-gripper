import json
import tempfile
import unittest
from pathlib import Path

import servo_controller


class ServoControllerTests(unittest.TestCase):
    def test_roll_is_validated_and_sent_before_gripper_opening(self):
        state = {
            "base": 0,
            "joint1": -90,
            "joint2": 0,
            "joint3": 120,
            "gripper_roll": -180,
            "gripper": 100,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot_state.json"
            path.write_text(json.dumps(state), encoding="utf-8")

            self.assertEqual(
                servo_controller.load_servo_positions(path),
                (90, 0, 90, 180, 0, 180),
            )

    def test_out_of_range_roll_is_rejected(self):
        state = {
            "base": 0,
            "joint1": 0,
            "joint2": 0,
            "joint3": 0,
            "gripper_roll": 180.1,
            "gripper": 50,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot_state.json"
            path.write_text(json.dumps(state), encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "gripper_roll"):
                servo_controller.load_servo_positions(path)

    def test_legacy_state_defaults_missing_roll_to_zero(self):
        state = {
            "base": 0,
            "joint1": 0,
            "joint2": 0,
            "joint3": 0,
            "gripper": 50,
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "robot_state.json"
            path.write_text(json.dumps(state), encoding="utf-8")

            self.assertEqual(
                servo_controller.load_servo_positions(path),
                (90, 90, 90, 90, 90, 90),
            )


if __name__ == "__main__":
    unittest.main()
