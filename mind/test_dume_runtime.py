import importlib.util
import math
import json
import queue
import sys
import tempfile
import threading
import unittest
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import patch


APP_PATH = Path(__file__).resolve().parents[1] / "Dum-E.py"
sys.path.insert(0, str(APP_PATH.parent / "mind"))
sys.path.insert(0, str(APP_PATH.parent / "movements"))
import mind as brain
import robot_arm as movement_arm

SPEC = importlib.util.spec_from_file_location("dume_app", APP_PATH)
if SPEC is None or SPEC.loader is None:
    raise RuntimeError(f"Could not load application module from {APP_PATH}")
dume = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = dume
SPEC.loader.exec_module(dume)


class DumERuntimeTests(unittest.TestCase):
    def test_all_interfaces_use_the_shared_joint_limits(self):
        shared = dict(dume.JOINT_LIMITS)
        self.assertEqual(shared["base"], (-180.0, 180.0))
        self.assertEqual(brain.JOINT_RANGES, {**shared, "gripper": (0, 100)})
        self.assertEqual(dume.JOINT_RANGES, {**shared, "gripper": (0, 100)})
        self.assertEqual(movement_arm.JOINT_LIMITS, shared)

    def test_kinematics_reject_invalid_opening_and_empty_verification(self):
        angles = dume.HOME_ANGLES
        self.assertEqual(
            dume.forward_kinematics(angles, opening=None),
            dume.forward_kinematics(angles),
        )
        for call in (
            lambda: dume.forward_kinematics(angles, opening=2.0),
            lambda: dume.inverse_kinematics(300.0, 0.0, 200.0, opening=2.0),
        ):
            with self.assertRaises(ValueError):
                call()
        with self.assertRaises(ValueError):
            dume.verify_fk_ik(samples=0)

    def test_visual_pose_state_writes_all_validated_joint_values(self):
        pose = {
            "base": -135.5,
            "joint1": 120,
            "joint2": -60.25,
            "joint3": -30,
            "gripper": 75.5,
        }
        with tempfile.TemporaryDirectory() as directory:
            mind_state = Path(directory) / "mind" / "robot_state.json"
            root_state = Path(directory) / "robot_state.json"
            mind_state.parent.mkdir()
            with patch.object(dume, "STATE_FILE", mind_state), patch.object(
                dume, "ROOT_STATE_FILE", root_state
            ):
                dume.save_pose_state(pose)
            self.assertEqual(json.loads(mind_state.read_text(encoding="utf-8")), pose)
            self.assertEqual(json.loads(root_state.read_text(encoding="utf-8")), pose)

    def test_visual_pose_state_rejects_invalid_values_without_writing(self):
        pose = dume.DEFAULT_STATE.copy()
        pose["joint2"] = float("nan")
        with tempfile.TemporaryDirectory() as directory:
            mind_state = Path(directory) / "mind" / "robot_state.json"
            root_state = Path(directory) / "robot_state.json"
            with patch.object(dume, "STATE_FILE", mind_state), patch.object(
                dume, "ROOT_STATE_FILE", root_state
            ):
                with self.assertRaisesRegex(ValueError, "joint2"):
                    dume.save_pose_state(pose)
            self.assertFalse(mind_state.exists())
            self.assertFalse(root_state.exists())

    def test_ensure_state_file_creates_root_mirror_from_shared_state(self):
        pose = {
            "base": -45,
            "joint1": 100,
            "joint2": -80,
            "joint3": -35,
            "gripper": 25,
        }
        with tempfile.TemporaryDirectory() as directory:
            mind_state = Path(directory) / "mind" / "robot_state.json"
            root_state = Path(directory) / "robot_state.json"
            mind_state.parent.mkdir()
            mind_state.write_text(json.dumps(pose), encoding="utf-8")
            with patch.object(dume, "STATE_FILE", mind_state), patch.object(
                dume, "ROOT_STATE_FILE", root_state
            ):
                loaded = dume.ensure_state_file()
            self.assertEqual(loaded, pose)
            self.assertEqual(json.loads(root_state.read_text(encoding="utf-8")), pose)

    def test_plot_and_animation_modes_persist_the_visualized_pose(self):
        calls = []
        angles = dume.HOME_ANGLES
        app = SimpleNamespace(
            _resolve_viz_pose=lambda: (angles, None, 0.25),
            _draw=lambda *args, **kwargs: calls.append("draw"),
            _persist_visual_pose=lambda: calls.append("persist"),
            _anim_job="job",
            viz_anim_btn=SimpleNamespace(configure=lambda **kwargs: None),
            viz_plot_btn=SimpleNamespace(configure=lambda **kwargs: None),
        )
        dume.DumEApp._do_viz_plot(app)
        dume.DumEApp._anim_finish(app)
        self.assertEqual(calls, ["draw", "persist", "persist"])

    def test_auto_ik_keeps_the_requested_elbow_branch(self):
        target = dume.forward_kinematics(dume.HOME_ANGLES)
        for branch, expected_sign in ((True, -1), (False, 1)):
            result = dume.inverse_kinematics(
                target.x, target.y, target.z, joint2_up=branch, joint3="auto"
            )
            _, delta = dume._forearm(
                math.radians(result.joint3), 1.0, dume.CONFIG
            )
            effective_elbow = math.radians(result.joint2) + delta
            self.assertEqual(math.copysign(1, effective_elbow), expected_sign)

    def test_auto_ik_uses_signed_base_yaw_for_reported_target(self):
        target = dume.CartesianPoint(-300.0, -120.0, 150.0)
        result = dume.inverse_kinematics(
            target.x, target.y, target.z, joint3="auto"
        )
        expected = dume.inverse_kinematics(
            target.x, target.y, target.z, joint3=-60.0
        )
        self.assertEqual(result, expected)
        self.assertAlmostEqual(
            result.theta_base, math.degrees(math.atan2(target.y, target.x))
        )

    def test_model_worker_serializes_and_replaces_stale_queued_requests(self):
        app = dume.DumEApp.__new__(dume.DumEApp)
        app.model_requests = queue.Queue(maxsize=dume.MODEL_REQUEST_QUEUE_SIZE)
        app.results = queue.Queue()
        started = threading.Event()
        release = threading.Event()
        active = 0
        maximum_active = 0
        seen = []
        lock = threading.Lock()

        def fake_model(prompt):
            nonlocal active, maximum_active
            with lock:
                active += 1
                maximum_active = max(maximum_active, active)
                seen.append(prompt)
            if prompt == "first":
                started.set()
                if not release.wait(5):
                    raise TimeoutError("test did not release the first request")
            with lock:
                active -= 1
            return {"prompt": prompt}

        with patch.object(dume, "ask_model", side_effect=fake_model):
            worker = threading.Thread(target=app._model_worker, daemon=True)
            worker.start()
            try:
                app._queue_model_request(1, "first")
                self.assertTrue(started.wait(2))
                app._queue_model_request(2, "stale")
                app._queue_model_request(3, "latest")
            finally:
                release.set()
                app.model_requests.put(None)
                worker.join(5)

        self.assertFalse(worker.is_alive())
        self.assertEqual(seen, ["first", "latest"])
        self.assertEqual(maximum_active, 1)
        self.assertEqual(
            [app.results.get_nowait(), app.results.get_nowait()],
            [(1, {"prompt": "first"}, None), (3, {"prompt": "latest"}, None)],
        )


if __name__ == "__main__":
    unittest.main()
