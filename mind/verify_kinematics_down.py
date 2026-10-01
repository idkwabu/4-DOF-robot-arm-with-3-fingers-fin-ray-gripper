"""Check that ``joint3="down"`` really pins the L3 stem straight down.

The mode claims a WORLD-frame orientation: the L3 stem, plus the gripper grip
depth, always points along -Z no matter how the arm is folded.  Asserting that
through the solver's own angle arithmetic would only prove it agrees with
itself, so the load-bearing check here is geometric and independent -- solve,
then forward-kinematics the pose and measure the actual link vector.

The repo carries two independent copies of the solver with different call
signatures and different joint limits, so both are exercised through a small
adapter.  They are expected to disagree about *which* targets fit inside their
joint limits; they must agree about the geometry.
"""

import importlib.util
import math
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "movements"))

import robot_arm

_spec = importlib.util.spec_from_file_location("dume_main", ROOT / "Dum-E.py")
dume = importlib.util.module_from_spec(_spec)
sys.modules["dume_main"] = dume
_spec.loader.exec_module(dume)

STEM_DOWN_DEG = -90.0
TOL_MM = 1e-6
TOL_DEG = 1e-6

# A spread that covers all four quadrants, the folded-back region behind the
# base, and both the reach envelope and its edges.
TARGETS = (
    (300.0, 120.0, 150.0),
    (0.0, 400.0, 100.0),
    (200.0, 200.0, -50.0),
    (-300.0, -120.0, 150.0),
    (-400.0, 0.0, 200.0),
    (350.0, -250.0, 100.0),
    (150.0, 0.0, 400.0),
)

# Far outside any pose; used to prove the mode refuses rather than inventing one.
UNREACHABLE = (5000.0, 5000.0, 5000.0)

BAD_MODES = ("sideways", "down-ish", "vertical", "")

DOWN = (0.0, 0.0, -1.0)


class Copy:
    """One solver copy behind a uniform interface.

    ``solve``, ``pose`` and ``forward`` keep each copy's own keyword arguments,
    so every check below stays identical across both.
    """

    def __init__(self, name, l3, solve, pose, forward, limits, declared_axis=None):
        self.name = name
        self.l3 = l3
        self._solve = solve
        self._pose = pose
        self._forward = forward
        self._limits = limits
        self._declared_axis = declared_axis

    def solve(self, target, joint2_up, opening, joint3):
        return self._solve(target, joint2_up, opening, joint3)

    def pose(self, angles, opening):
        return self._pose(angles, opening)

    def grip_point(self, angles, opening):
        return self._forward(angles, opening)

    def in_limits(self, angles):
        return self._limits(angles)

    def stem_outward(self, pose):
        """L3 link vector, pointing from the wrist toward the gripper mount."""
        mount, wrist = pose.gripper_mount, pose.joint3
        return (mount.x - wrist.x, mount.y - wrist.y, mount.z - wrist.z)


COPIES = (
    Copy(
        name="movements/robot_arm.py",
        l3=robot_arm.DEFAULT_CONFIG.l3,
        solve=lambda t, up, opening, j3: robot_arm.inverse_kinematics(
            x=t[0], y=t[1], z=t[2], joint2_up=up, opening=opening, joint3=j3
        ),
        pose=lambda a, opening: robot_arm.gripper_pose(
            a.theta_base, a.joint1, a.joint2, a.joint3, opening=opening
        ),
        forward=lambda a, opening: robot_arm.forward_kinematics(
            a.theta_base, a.joint1, a.joint2, a.joint3, opening=opening
        ),
        limits=lambda a: robot_arm.check_joint_limits(
            a.theta_base, a.joint1, a.joint2, a.joint3
        ),
        declared_axis="axis",
    ),
    Copy(
        name="Dum-E.py",
        l3=dume.CONFIG.l3,
        solve=lambda t, up, opening, j3: dume.inverse_kinematics(
            x=t[0], y=t[1], z=t[2], joint2_up=up, opening=opening, joint3=j3
        ),
        pose=lambda a, opening: dume.gripper_pose(a, opening),
        forward=lambda a, opening: dume.forward_kinematics(a, opening),
        limits=lambda a: dume.check_joint_limits(a),
    ),
)


def point_error(point, target):
    return math.dist((point.x, point.y, point.z), target)


def angle_residue(total):
    """Fold to [0, 180] degrees, since each joint is wrapped independently."""
    residue = total % 360.0
    return min(residue, 360.0 - residue)


def check_target(copy, target, opening):
    """Every invariant that must hold for one solved target, across both branches.

    Raises AssertionError on the first violation.
    """
    for up in (True, False):
        angles = copy.solve(target, up, opening, "down")
        pose = copy.pose(angles, opening)

        # The L3 link itself must be exactly L3 long and point straight down.
        stem = copy.stem_outward(pose)
        length = math.dist(stem, (0.0, 0.0, 0.0))
        if abs(length - copy.l3) > TOL_MM:
            raise AssertionError(f"L3 length {length:.6f} mm != {copy.l3} mm")
        tilt = math.dist(tuple(component / length for component in stem), DOWN)
        if tilt > TOL_MM:
            raise AssertionError(f"stem tilts {tilt:.3e} off straight down")

        if copy._declared_axis is not None:
            declared = tuple(getattr(pose, copy._declared_axis))
            if math.dist(declared, DOWN) > TOL_MM:
                raise AssertionError(f"reported axis {declared} is not straight down")

        # The angle identity the mode is named for.
        residue = angle_residue(angles.joint1 + angles.joint2 + angles.joint3 - STEM_DOWN_DEG)
        if residue > TOL_DEG:
            raise AssertionError(
                f"joint1+joint2+joint3 is {residue:.3e} deg off {STEM_DOWN_DEG}"
            )

        # And the pose really does put the grip point on the target.
        error = point_error(copy.grip_point(angles, opening), target)
        if error > TOL_MM:
            raise AssertionError(f"round-trip error {error:.3e} mm")


def check_copy(copy):
    """Run every check against one copy; return (failures, stats)."""
    failures = []

    # 1. The defining property, geometrically, at every opening.
    checked = 0
    for opening in (0.0, 0.5, 1.0):
        for target in TARGETS:
            try:
                check_target(copy, target, opening)
                checked += 1
            except (AssertionError, ValueError) as exc:
                failures.append(f"opening={opening} target={target}: {exc}")

    # 2. Folded-back targets behind the base stay reachable.
    folded = [t for t in TARGETS if t[1] < 0.0]
    folded_ok = 0
    for target in folded:
        try:
            copy.solve(target, True, 1.0, "down")
            folded_ok += 1
        except ValueError:
            pass
    if not folded_ok:
        failures.append(f"no y<0 target reachable with a down wrist ({len(folded)} tried)")

    # 3. Refuses an impossible target instead of inventing a pose.
    try:
        copy.solve(UNREACHABLE, True, 1.0, "down")
        failures.append("accepted a target 5 m away")
    except ValueError:
        pass

    # 4. Mode vocabulary is case-insensitive and tolerant of stray spaces.
    target = TARGETS[0]
    for spelling in ("down", "DOWN", "Down", "  down  "):
        try:
            copy.solve(target, True, 1.0, spelling)
        except ValueError as exc:
            failures.append(f"rejected valid mode {spelling!r}: {exc}")

    # 5. Typos are rejected rather than silently read as a wrist angle.
    for mode in BAD_MODES:
        try:
            copy.solve(target, True, 1.0, mode)
            failures.append(f"accepted bogus mode {mode!r}")
        except ValueError:
            pass

    # 6. Regression: the pre-existing numeric and auto paths still round-trip.
    for mode in (-60.0, "auto"):
        try:
            angles = copy.solve(target, True, 1.0, mode)
            error = point_error(copy.grip_point(angles, 1.0), target)
            if error > TOL_MM:
                failures.append(f"joint3={mode!r} round-trip error {error:.3e} mm")
        except ValueError as exc:
            failures.append(f"joint3={mode!r} regressed: {exc}")

    # 7. Coverage of the default workspace. Reported, not gated: the two copies
    # use different joint limits, so they are expected to differ here.
    rng = random.Random(11)
    pool = []
    for _ in range(500):
        candidate = (
            rng.uniform(-550, 550),
            rng.uniform(-550, 550),
            rng.uniform(-150, 600),
        )
        try:
            copy.solve(candidate, True, 1.0, "auto")
        except ValueError:
            continue
        pool.append(candidate)
    covered = 0
    for candidate in pool:
        try:
            copy.solve(candidate, True, 1.0, "down")
            covered += 1
        except ValueError:
            pass
    share = covered / len(pool) if pool else 0.0
    if share <= 0.0:
        failures.append("down mode reaches nothing")

    stats = {
        "checked": checked,
        "folded": (folded_ok, len(folded)),
        "pool": len(pool),
        "covered": covered,
        "share": share,
    }
    return failures, stats


def main() -> int:
    all_failures = []

    for copy in COPIES:
        print(f"=== {copy.name} ===")
        failures, stats = check_copy(copy)
        print(f"  stem straight down at every opening, every branch,"
              f" {stats['checked']}/{len(TARGETS) * 3} target/opening pairs")
        print(f"  folded-back targets solved: {stats['folded'][0]}/{stats['folded'][1]}")
        print(f"  down covers {stats['covered']}/{stats['pool']} = {stats['share']:.1%}"
              " of default-reachable targets")
        for line in failures:
            print(f"  FAIL {line}")
        all_failures.extend(f"{copy.name}: {line}" for line in failures)

    print()
    if all_failures:
        print(f"FAIL: {len(all_failures)} problem(s)")
        for line in all_failures:
            print(f"  {line}")
        return 1
    print("PASS: joint3='down' points the stem straight down in both copies")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())