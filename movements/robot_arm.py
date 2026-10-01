"""
DUM-E: 4-DOF Robot Arm — Kinematics Core and 3D Visualization.

Implements the mathematical core of a 4-DOF arm plus a matplotlib 3D
visualisation and a text CLI. A tkinter desktop UI lives in
``robot_arm_gui.py``. Deliberately not implemented here: Arduino, ESP32,
serial, servos, camera, vision, speech, LLM/AI, web, databases, or advanced
motion planning.

All angles are internally radians; public functions take and report degrees.
The physical chain is::

    base -> fixed vertical L0 -> joint1 -> L1 -> joint2 -> L2 -> joint3
         -> L3 -> gripper_mount -> fingers

L0 is a fixed 50 mm vertical link. The base yaw rotates the radial-Z arm
around +Z but cannot tilt L0. joint1 tilts L1, joint2 tilts L2 relative
to L1, and joint3 tilts L3 relative to L2.
The grip point is the on-axis centroid of the three fingertips, so forward
and inverse kinematics target the object centre.

The radial projection is signed. A folded arm can project behind the base,
in which case an IK solution may return a base azimuth shifted by 180 degrees.
The two-link radial-Z solve uses the effective forearm formed from L2, L3,
and the fingertip grip depth. Reach bounds are measured from joint1, not the
world origin. Joint limits are checked separately from geometric reachability.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, Sequence, Tuple

# =====================================================================
# 2. ROBOT CONFIGURATION  (the single place robot dimensions live)
# =====================================================================

# Link lengths in millimetres.
L0: float = 50.0
L1: float = 300.0
L2: float = 250.0
L3: float = 70.0

# Fixed orientation of L3 relative to L2.
WRIST_STEM_ANGLE: float = -60.0

# Absolute stem orientation used by the ``joint3="down"`` IK mode: the L3 stem
# plus the gripper grip depth points straight down in the WORLD frame, so
# ``joint1 + joint2 + joint3 == STEM_DOWN_ANGLE_DEG`` for any reachable pose.
# This is a different thing from WRIST_STEM_ANGLE, which is a wrist angle
# RELATIVE to L2.
STEM_DOWN_ANGLE_DEG: float = -90.0

# Simulated 3-finger gripper (kinematic end-effector, visual + reachable).
GRIPPER_FINGERS: int = 3        # number of fingers (3 = tripod, 120 deg apart)
FINGER_LENGTH: float = 50.0     # length of each finger segment [mm]
FINGER_OPENING_DEG: float = 30.0  # max spread of a finger off the tool axis [deg]
GRIPPER_OPENING: float = 1.0    # 0 = closed (tips meet on the tool axis), 1 = open

# Joint limits measured in DEGREES.  These are configuration parameters,
# independent of the kinematics math, and may change when the physical
# arm is built.  Ranges are inclusive on both ends.
#
#   base:     (-180, 180)    full revolution, per the atan2 convention.
#   joint1:   (0, 180)       link-1 angle in the radial-Z plane, measured
#                            from horizontal outward (0 = horizontal,
#                            positive = rising above horizon).
#   joint2:   (-150, 150)    relative angle of link 2 w.r.t. link 1.
#                            NOTE: the up/down IK branch is selected by the
#                            sign of (joint2 + delta_g), not by the raw sign
#                            of joint2 (see the module docstring).  Targets
#                            close to the base still push |joint2| toward
#                            ~136 deg, so the elbow limit is the binding
#                            constraint on the inner part of the workspace.
#   joint3:   (-150, 150)    relative angle of link 3 (wrist stem L3) w.r.t. link 2.
#                            This is the wrist angle between the forearm and the
#                            fixed stem L3.  The WRIST_STEM_ANGLE constant (-60 deg)
#                            is the default/home offset baked into the effective
#                            forearm model.  The physical joint3 rotates L3 around
#                            the same axis as joint2 (the radial-Z plane).
JOINT_LIMITS: Dict[str, Tuple[float, float]] = {
    "base": (-180.0, 180.0),
    "joint1": (-90.0, 110.0),
    "joint2": (-150.0, 150.0),
    "joint3": (-150.0, 150.0),
}

# Numerical tolerances.
POSITION_TOLERANCE_MM: float = 0.1    # FK -> IK -> FK Cartesian tolerance
ANGLE_TOLERANCE_RAD: float = 1e-6     # angle-reconstruction tolerance
REACH_EPSILON_MM: float = 1e-9        # boundary slack for d in [min,max]
MIN_D_SHARE_LIMIT: float = 1e-9       # guard: d^2 division, near-base axis

# =====================================================================
# 3. DATA STRUCTURES
# =====================================================================


@dataclass(frozen=True)
class RobotConfig:
    """All physical/kinematic parameters of the arm.

    Configurable parameters are centralised here; functions take a
    ``RobotConfig`` rather than hard-coding the dimensions.
    """

    l0: float = L0
    l1: float = L1
    l2: float = L2
    l3: float = L3
    wrist_stem_angle: float = WRIST_STEM_ANGLE
    gripper_fingers: int = GRIPPER_FINGERS
    finger_length: float = FINGER_LENGTH
    finger_opening_deg: float = FINGER_OPENING_DEG
    gripper_opening: float = GRIPPER_OPENING
    joint_limits: Dict[str, Tuple[float, float]] = field(
        default_factory=lambda: {k: v for k, v in JOINT_LIMITS.items()}
    )


DEFAULT_CONFIG: RobotConfig = RobotConfig()


@dataclass(frozen=True)
class CartesianPoint:
    """A cartesian position in the base frame, millimetres."""

    x: float
    y: float
    z: float

    def coords(self) -> Tuple[float, float, float]:
        """Return (x, y, z) as a plain tuple."""
        return (self.x, self.y, self.z)


@dataclass(frozen=True)
class JointAngles:
    """Joint angles.  Public API reports DEGREES (see conversions)."""

    theta_base: float       # rotation about +Z                       [deg]
    joint1: float           # link 1 in the radial-Z plane            [deg]
    joint2: float           # link 2 relative to link 1               [deg]
    joint3: float           # link 3 relative to link 2               [deg]


# Animation start pose: link 1 and link 2 straight up.  Note the gripper stem
# is still kinked by phi, so the grip point is NOT directly overhead.
HOME_ANGLES = JointAngles(0.0, 90.0, 0.0, -60.0)


@dataclass(frozen=True)
class FingerGripper:
    """Full gripper geometry for one pose (base-frame mm; axis is unit).

    ``grip_point`` is the on-axis centroid of the fingertip triangle and is
    the point IK aims at. The ``wrist`` property remains an alias for the
    gripper mount for compatibility with older callers.
    """

    joint1: CartesianPoint
    joint2: CartesianPoint
    joint3: CartesianPoint
    gripper_mount: CartesianPoint
    axis: Tuple[float, float, float]
    spread: float
    fingertips: Tuple[CartesianPoint, ...]
    grip_point: CartesianPoint
    opening: float

    @property
    def wrist(self) -> CartesianPoint:
        return self.joint3

    @property
    def shoulder(self) -> CartesianPoint:
        return self.joint1

    @property
    def elbow(self) -> CartesianPoint:
        return self.joint2


# =====================================================================
# 4. MATHEMATICAL UTILITY FUNCTIONS
# =====================================================================


def _to_rad(deg: float) -> float:
    return math.radians(deg)


def _to_deg(rad: float) -> float:
    return math.degrees(rad)


def _clamp(value: float, lo: float, hi: float) -> float:
    """Clamp *value* into [lo, hi] to absorb floating-point noise."""
    return max(lo, min(hi, value))


def _wrap_angle(rad: float) -> float:
    """Wrap an angle to (-math.pi, math.pi]."""
    return math.atan2(math.sin(rad), math.cos(rad))


def _check_finite(value: float, name: str) -> None:
    if not math.isfinite(value):
        raise ValueError(f"Numerically invalid input: {name}={value!r} is not finite.")


def _reach_bounds(config: RobotConfig, opening: float | None = None) -> Tuple[float, float]:
    """Geometric reach bounds for the GRIP POINT (|L1 - m_g|, L1 + m_g).

    Kept in one place so ``inverse_kinematics()`` and ``is_reachable()``
    agree by construction.  ``m_g`` comes from ``_grip_parameters``.
    """
    m, _ = _grip_parameters(config, opening)
    return abs(config.l1 - m), config.l1 + m


def _forearm_parameters(config: RobotConfig, axis_len: float) -> Tuple[float, float]:
    """Effective forearm (m, delta) for a rigid tool-axis extension.

    Any extension along the tool axis (stem L3, or L3 + finger grip depth)
    collapses to ``m*e^(i*(ts+te+delta))`` with ``phi`` the fixed
    ``wrist_stem_angle``, letting the IK solve a plain 2-link arm
    ``(L1, m)`` at angles ``(joint1, te + delta)``.
    """
    phi = math.radians(config.wrist_stem_angle)
    real = config.l2 + axis_len * math.cos(phi)
    imag = axis_len * math.sin(phi)
    m = math.hypot(real, imag)
    delta = math.atan2(imag, real)
    return m, delta


def _stem_parameters(config: RobotConfig) -> Tuple[float, float]:
    """Effective forearm (m, delta) for the fixed wrist-angle stem L3."""
    return _forearm_parameters(config, config.l3)


def _grip_parameters(config: RobotConfig, opening: float | None = None) -> Tuple[float, float]:
    """Effective forearm (m_g, delta_g) for the GRIP POINT = the l_g = 
    ``finger_length*cos(spread)`` on-axis grip depth plus the stem:
    ``L2 + (L3 + l_g)*e^(i*phi) = m_g*e^(i*delta_g)``.
    """
    _, _, l_g, _ = _gripper_pose_params(config, opening)
    return _forearm_parameters(config, config.l3 + l_g)


def _gripper_pose_params(
    config: RobotConfig, opening: float | None = None
) -> Tuple[float, float, float, int]:
    """(spread_rad, cos(spread), l_g, finger_count) for the gripper opening.

    opening 0 (closed) -> spread 0, fingertips converge ON the tool axis at
    depth finger_length; opening 1 (open) -> spread = finger_opening_deg.
    """
    openings = config.gripper_opening if opening is None else opening
    if not 0.0 <= openings <= 1.0:
        raise ValueError(f"gripper opening must be in [0, 1]; got {openings!r}.")
    spread = math.radians(config.finger_opening_deg) * openings
    return spread, math.cos(spread), config.finger_length * math.cos(spread), config.gripper_fingers


# =====================================================================
# 5. FORWARD KINEMATICS
# =====================================================================


def _fk_parts(
    theta_base: float, joint1: float, joint2: float, joint3: float, config: RobotConfig
) -> Tuple[CartesianPoint, CartesianPoint, CartesianPoint, CartesianPoint, CartesianPoint]:
    """Return base, joint1, joint2, joint3, and gripper_mount (radians in)."""
    _check_finite(theta_base, "theta_base (rad)")
    _check_finite(joint1, "joint1 (rad)")
    _check_finite(joint2, "joint2 (rad)")
    _check_finite(joint3, "joint3 (rad)")

    l1n = config.l1
    l2n = config.l2
    l3n = config.l3
    cb = math.cos(theta_base)
    sb = math.sin(theta_base)

    base = CartesianPoint(0.0, 0.0, 0.0)
    joint1_pos = CartesianPoint(0.0, 0.0, config.l0)

    link2_angle = joint1 + joint2
    joint2_radial = l1n * math.cos(joint1)
    joint2_height = config.l0 + l1n * math.sin(joint1)
    joint2_pos = CartesianPoint(
        joint2_radial * cb, joint2_radial * sb, joint2_height
    )

    link3_angle = link2_angle + joint3
    joint3_radial = joint2_radial + l2n * math.cos(link2_angle)
    joint3_height = joint2_height + l2n * math.sin(link2_angle)
    joint3_pos = CartesianPoint(
        joint3_radial * cb, joint3_radial * sb, joint3_height
    )

    gripper_angle = link3_angle
    gripper_radial = joint3_radial + l3n * math.cos(gripper_angle)
    gripper_height = joint3_height + l3n * math.sin(gripper_angle)
    gripper_mount = CartesianPoint(
        gripper_radial * cb, gripper_radial * sb, gripper_height
    )
    return base, joint1_pos, joint2_pos, joint3_pos, gripper_mount


def _gripper_pose(
    theta_base: float, joint1: float, joint2: float, joint3_angle: float, config: RobotConfig, opening: float | None = None
) -> FingerGripper:
    """Finger-gripper geometry for angles in RADIANS."""
    _, joint1_pos, joint2_pos, joint3_pos, gripper_mount = _fk_parts(
        theta_base, joint1, joint2, joint3_angle, config
    )

    spread_rad, _, l_g, n_fingers = _gripper_pose_params(config, opening)
    link2_angle = joint1 + joint2
    link3_angle = link2_angle + joint3_angle
    gripper_angle = link3_angle
    cb = math.cos(theta_base)
    sb = math.sin(theta_base)
    ax = (
        math.cos(gripper_angle) * cb,
        math.cos(gripper_angle) * sb,
        math.sin(gripper_angle),
    )

    if abs(ax[2]) < 0.9999:
        ref = (0.0, 0.0, 1.0)
    else:
        ref = (1.0, 0.0, 0.0)
    u0 = _cross3(ax, ref)
    u0 = tuple(c / math.sqrt(sum(c * c for c in u0)) for c in u0)
    v0 = _cross3(ax, u0)

    grip_point = CartesianPoint(
        gripper_mount.x + l_g * ax[0],
        gripper_mount.y + l_g * ax[1],
        gripper_mount.z + l_g * ax[2],
    )

    tips = []
    for k in range(n_fingers):
        azimuth = 2.0 * math.pi * k / n_fingers
        radial = (
            math.cos(azimuth) * u0[0] + math.sin(azimuth) * v0[0],
            math.cos(azimuth) * u0[1] + math.sin(azimuth) * v0[1],
            math.cos(azimuth) * u0[2] + math.sin(azimuth) * v0[2],
        )
        direction = (
            math.cos(spread_rad) * ax[0] + math.sin(spread_rad) * radial[0],
            math.cos(spread_rad) * ax[1] + math.sin(spread_rad) * radial[1],
            math.cos(spread_rad) * ax[2] + math.sin(spread_rad) * radial[2],
        )
        tips.append(
            CartesianPoint(
                gripper_mount.x + config.finger_length * direction[0],
                gripper_mount.y + config.finger_length * direction[1],
                gripper_mount.z + config.finger_length * direction[2],
            )
        )

    opening_eff = config.gripper_opening if opening is None else opening
    return FingerGripper(
        joint1=joint1_pos,
        joint2=joint2_pos,
        joint3=joint3_pos,
        gripper_mount=gripper_mount,
        axis=ax,
        spread=math.degrees(spread_rad),
        fingertips=tuple(tips),
        grip_point=grip_point,
        opening=opening_eff,
    )


def _cross3(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Vector (cross) product of two 3-vectors."""
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _fk_rad(
    theta_base: float, joint1: float, joint2: float, joint3: float, config: RobotConfig, opening: float | None = None
) -> CartesianPoint:
    """Forward kinematics (all angles in RADIANS): returns the GRIP POINT."""
    return _gripper_pose(theta_base, joint1, joint2, joint3, config, opening).grip_point


def forward_kinematics(
    theta_base: float,
    joint1: float,
    joint2: float,
    joint3: float,
    config: RobotConfig = DEFAULT_CONFIG,
    opening: float | None = None,
) -> CartesianPoint:
    """Forward kinematics: returns the GRIP POINT (fingertip centroid).

    ``theta_base``/``joint1``/``joint2``/``joint3`` in degrees; ``opening`` 0..1
    defaults to ``config.gripper_opening``.  See the module docstring for
    the model (fixed stem angle, effective forearm).
    """
    return _fk_rad(
        _to_rad(theta_base), _to_rad(joint1), _to_rad(joint2), _to_rad(joint3), config, opening
    )


def gripper_pose(
    theta_base: float,
    joint1: float,
    joint2: float,
    joint3: float,
    config: RobotConfig = DEFAULT_CONFIG,
    opening: float | None = None,
) -> FingerGripper:
    """Full simulated gripper geometry for a pose (all angles in degrees)."""
    return _gripper_pose(
        _to_rad(theta_base), _to_rad(joint1), _to_rad(joint2), _to_rad(joint3), config, opening
    )


def finger_tips(
    theta_base: float,
    joint1: float,
    joint2: float,
    joint3: float,
    config: RobotConfig = DEFAULT_CONFIG,
    opening: float | None = None,
) -> Tuple[CartesianPoint, ...]:
    """The ``gripper_fingers`` fingertip positions (degrees in, mm out)."""
    return _gripper_pose(
        _to_rad(theta_base), _to_rad(joint1), _to_rad(joint2), _to_rad(joint3), config, opening
    ).fingertips


def _reach_bounds_with_joint3(config: RobotConfig, joint3: float, opening: float | None = None) -> Tuple[float, float]:
    """Geometric reach bounds for the GRIP POINT with a given joint3 angle."""
    spread, _, l_g, _ = _gripper_pose_params(config, opening)
    # Effective forearm: L2 + (L3 + l_g) * e^(i*joint3)
    phi3 = joint3
    real = config.l2 + (config.l3 + l_g) * math.cos(phi3)
    imag = (config.l3 + l_g) * math.sin(phi3)
    m = math.hypot(real, imag)
    return abs(config.l1 - m), config.l1 + m


def _ik_rad(
    x: float, y: float, z: float, joint2_up: bool, config: RobotConfig, 
    joint3: float, opening: float | None = None
) -> JointAngles:
    """Inverse kinematics core; angles in RADIANS.  Raises ``ValueError``
    for targets outside the geometric workspace or numerically invalid.
    Cartesian targets are world-frame coordinates, with joint1 at z=L0.
    joint3 is the L3 angle relative to L2 (radians).
    """
    for name, v in (("x", x), ("y", y), ("z", z)):
        _check_finite(v, name)

    l1n = config.l1
    spread, _, l_g, _ = _gripper_pose_params(config, opening)
    phi3 = joint3
    # Effective forearm: L2 + (L3 + l_g) * e^(i*joint3)
    real = config.l2 + (config.l3 + l_g) * math.cos(phi3)
    imag = (config.l3 + l_g) * math.sin(phi3)
    m = math.hypot(real, imag)
    delta = math.atan2(imag, real)
    min_reach = abs(config.l1 - m)
    max_reach = config.l1 + m

    # Step 1: base angle isolates the azimuth.
    theta_base = math.atan2(y, x)

    # Step 2: collapse to the radial-Z 2-link problem from joint1.
    r = math.hypot(x, y)
    height = z - config.l0
    d2 = r * r + height * height
    d = math.sqrt(d2)

    # Geometric workspace check.
    if d < min_reach - REACH_EPSILON_MM or d > max_reach + REACH_EPSILON_MM:
        raise ValueError(
            f"Target is outside the robot workspace: distance to joint1 pivot "
            f"{d:.3f} mm not within [{min_reach:.3f}, {max_reach:.3f}] mm."
        )

    # Guard against targets sitting essentially on the joint1 axis.
    if d <= MIN_D_SHARE_LIMIT:
        raise ValueError(
            "Target is too close to the joint1 reference point "
            "(degenerate law-of-cosines limit); target is unreachable."
        )

    # Step 3: effective elbow angle te' = joint2 + delta by the law of
    # cosines, using the FK identity
    #   d^2 = L1^2 + m^2 + 2*L1*m*cos(te'),   te' = joint2 + delta
    # so  cos(te') = (d^2 - L1^2 - m^2) / (2*L1*m).
    cos_tep = (d2 - l1n * l1n - m * m) / (2.0 * l1n * m)
    cos_tep = _clamp(cos_tep, -1.0, 1.0)
    te_abs = math.acos(cos_tep)  # in [0, pi]

    # Step 4: joint-1 angle.  Two triangle angles:
    #   psi   = bearing of the target in the radial-Z plane
    #   gamma = interior angle at joint 1 between the line to the
    #           target and the direction of link 1.
    psi = math.atan2(height, r)
    cos_gamma = (l1n * l1n + d2 - m * m) / (2.0 * l1n * d)
    cos_gamma = _clamp(cos_gamma, -1.0, 1.0)
    gamma = math.acos(cos_gamma)

    # The two sign pairings are forced:
    #   joint2-up   => joint1 = psi + gamma, te' = -te_abs
    #   joint2-down => joint1 = psi - gamma, te' = +te_abs
    # then recover the PHYSICAL elbow angle:  joint2 = te' - delta.
    if joint2_up:
        joint1 = psi + gamma
        joint2 = -te_abs - delta
    else:
        joint1 = psi - gamma
        joint2 = +te_abs - delta

    joint_angles = JointAngles(
        theta_base=_wrap_angle(theta_base),
        joint1=_wrap_angle(joint1),
        joint2=_wrap_angle(joint2),
        joint3=joint3,  # pass through the given joint3
    )
    # Final propagation guard: never emit NaN/Inf silently.
    for angle in (joint_angles.theta_base, joint_angles.joint1, joint_angles.joint2):
        if not math.isfinite(angle):
            raise ValueError("IK produced a non-finite angle; target is invalid.")
    return joint_angles


def _ik_stem_world(
    x: float,
    y: float,
    z: float,
    joint2_up: bool,
    config: RobotConfig,
    opening: float | None = None,
    stem_deg: float = STEM_DOWN_ANGLE_DEG,
) -> JointAngles:
    """Inverse kinematics for a FIXED ABSOLUTE stem orientation.

    Used by the ``joint3="down"`` mode.  Unlike the general solver, which is
    handed a wrist angle and folds L3 + grip depth into an effective forearm,
    this one constrains where the stem points in the world: the returned pose
    always satisfies

        joint1 + joint2 + joint3 == stem_deg

    so the gripper approaches the target along a vertical line no matter how
    the arm is folded.

    Because the stem is rigid, fixing its absolute direction makes its
    contribution to the grip point a known constant vector.  Subtracting that
    vector from the target leaves a plain 2-link ``(L1, L2)`` problem, which is
    why this is closed-form like the general solver rather than iterative.

    Returns degrees.  Raises ``ValueError`` when the target is unreachable for
    this wrist orientation.  Joint limits are intentionally NOT applied.
    """
    for name, v in (("x", x), ("y", y), ("z", z)):
        _check_finite(v, name)

    stem = math.radians(stem_deg)
    spread, _, l_g, _ = _gripper_pose_params(config, opening)
    tool = config.l3 + l_g  # rigid stem + grip depth, length of the known vector

    # Peel the known stem vector off the target: the remaining point is what
    # links 1 and 2 alone would have to reach.
    r = math.hypot(x, y) - tool * math.cos(stem)
    height = (z - config.l0) - tool * math.sin(stem)

    d2 = r * r + height * height
    d = math.sqrt(d2)
    min_reach = abs(config.l1 - config.l2)
    max_reach = config.l1 + config.l2

    if d <= MIN_D_SHARE_LIMIT:
        raise ValueError(
            "Target is too close to the joint1 reference point for a "
            "straight-down wrist; target is unreachable."
        )

    if d < min_reach - REACH_EPSILON_MM or d > max_reach + REACH_EPSILON_MM:
        raise ValueError(
            f"Target is unreachable with a straight-down wrist: after removing the "
            f"stem offset the links must span {d:.3f} mm, outside "
            f"[{min_reach:.3f}, {max_reach:.3f}] mm. Use another joint3 value "
            f"(or \"auto\") for this target."
        )

    # Standard 2-link solve on the peeled-off point, same branch pairing and
    # clamping conventions as _ik_rad.
    psi = math.atan2(height, r)
    cos_gamma = (config.l1 * config.l1 + d2 - config.l2 * config.l2) / (2.0 * config.l1 * d)
    gamma = math.acos(_clamp(cos_gamma, -1.0, 1.0))
    cos_te = (d2 - config.l1 * config.l1 - config.l2 * config.l2) / (2.0 * config.l1 * config.l2)
    te = math.acos(_clamp(cos_te, -1.0, 1.0))

    if joint2_up:
        joint1 = psi + gamma
        joint2 = -te
    else:
        joint1 = psi - gamma
        joint2 = +te

    # joint3 is a consequence of the constraint, not an input.
    joint3 = stem - joint1 - joint2

    theta_base = math.atan2(y, x)
    joint_angles = JointAngles(
        theta_base=_to_deg(_wrap_angle(theta_base)),
        joint1=_to_deg(_wrap_angle(joint1)),
        joint2=_to_deg(_wrap_angle(joint2)),
        joint3=_to_deg(_wrap_angle(joint3)),
    )
    for angle in (
        joint_angles.theta_base,
        joint_angles.joint1,
        joint_angles.joint2,
        joint_angles.joint3,
    ):
        if not math.isfinite(angle):
            raise ValueError("IK produced a non-finite angle; target is invalid.")
    return joint_angles


def inverse_kinematics(
    x: float,
    y: float,
    z: float,
    joint2_up: bool = True,
    config: RobotConfig = DEFAULT_CONFIG,
    opening: float | None = None,
    joint3: float | str = -60.0,
) -> JointAngles:
    """Inverse kinematics for the GRIP POINT (degrees in, `JointAngles` out).

    ``theta_base = atan2(y, x)``; the radial-Z 2-link problem is solved by
    the law of cosines with effective forearm computed from L2, L3, l_g, and joint3.
    ``joint2_up`` chooses the sign of ``te' = joint2 + delta`` (up: negative, down: positive);
    physical ``joint2 = te' - delta``.  Raises ``ValueError`` for targets outside
    the geometric workspace, degenerate targets near the joint1 pivot, or
    non-finite input.  Joint limits are intentionally NOT applied here - use
    ``check_joint_limits`` separately.

    ``joint3`` is the L3 wrist angle in degrees RELATIVE to L2, or one of two
    strings:

    ``"auto"``
        search for a joint3 that reaches the target within the joint limits.
    ``"down"``
        instead solve for joint3 so the stem points straight DOWN in the world
        frame, i.e. ``joint1 + joint2 + joint3 == -90``.  This constrains the
        absolute tool orientation rather than the wrist angle relative to the
        forearm, so the gripper approaches along a vertical line regardless of
        posture.  The reachable span is narrower than a free wrist: the links
        must span ``|L1 - L2| <= d <= L1 + L2`` after the stem offset is
        removed from the target.
    """
    if isinstance(joint3, str):
        mode = joint3.strip().lower()
        if mode == "auto":
            return _inverse_kinematics_auto_joint3(x, y, z, joint2_up, config, opening)
        if mode == "down":
            return _ik_stem_world(x, y, z, joint2_up, config, opening)
        raise ValueError(
            f"Unknown joint3 mode {joint3!r}; expected a number in degrees, "
            f'"auto", or "down".'
        )
    angles_rad = _ik_rad(x, y, z, joint2_up, config, math.radians(joint3), opening)
    return JointAngles(
        theta_base=_to_deg(angles_rad.theta_base),
        joint1=_to_deg(angles_rad.joint1),
        joint2=_to_deg(angles_rad.joint2),
        joint3=joint3,
    )


# =====================================================================
# 7. WORKSPACE VALIDATION
# =====================================================================


def _inverse_kinematics_auto_joint3(
    x: float,
    y: float,
    z: float,
    joint2_up: bool,
    config: RobotConfig,
    opening: float | None,
) -> JointAngles:
    """Search for a joint3 value that makes the target reachable with all joints within limits.
    
    Strategy:
    1. Try default joint3 (-60°) first
    2. Search through joint3 range in steps, preferring values closer to -60° (default)
    3. Return first valid solution found (tries both joint2_up and joint2_down for each joint3)
    """
    # Define search range for joint3
    j3_min, j3_max = config.joint_limits.get("joint3", (-150.0, 150.0))
    
    # Order of joint3 values to try: prefer default (-60), then sweep outward
    candidates = [-60.0]
    step = 5.0
    max_offset = max(abs(j3_min + 60), abs(j3_max + 60))
    for offset in range(int(step), int(max_offset) + 1, int(step)):
        candidates.append(-60.0 + offset)
        candidates.append(-60.0 - offset)
    
    # Clamp candidates to limits and remove duplicates
    seen = set()
    unique_candidates = []
    for c in candidates:
        if j3_min <= c <= j3_max and c not in seen:
            seen.add(c)
            unique_candidates.append(c)
    
    last_error = None
    for j3 in unique_candidates:
        try:
            # Check geometric reachability first (fast check)
            if not is_reachable(x, y, z, config, math.radians(j3), opening):
                continue
            
            # Try joint2_up
            angles_rad = _ik_rad(x, y, z, True, config, math.radians(j3), opening)
            candidate = JointAngles(
                theta_base=_to_deg(angles_rad.theta_base),
                joint1=_to_deg(angles_rad.joint1),
                joint2=_to_deg(angles_rad.joint2),
                joint3=j3,
            )
            if check_joint_limits(candidate.theta_base, candidate.joint1, candidate.joint2, candidate.joint3, config):
                return candidate
                
            # Try joint2_down
            angles_rad = _ik_rad(x, y, z, False, config, math.radians(j3), opening)
            candidate = JointAngles(
                theta_base=_to_deg(angles_rad.theta_base),
                joint1=_to_deg(angles_rad.joint1),
                joint2=_to_deg(angles_rad.joint2),
                joint3=j3,
            )
            if check_joint_limits(candidate.theta_base, candidate.joint1, candidate.joint2, candidate.joint3, config):
                return candidate
                
        except ValueError as e:
            continue
    
    # If we got here, no valid joint3 found
    raise ValueError(f"No valid joint3 found in range [{j3_min:.1f}, {j3_max:.1f}] for target ({x:.1f}, {y:.1f}, {z:.1f})")


def is_reachable(
    x: float, y: float, z: float, config: RobotConfig = DEFAULT_CONFIG, 
    joint3: float = math.radians(-60.0), opening: float | None = None
) -> bool:
    """Geometric reachability of the grip-point target, independent of joint
    limits: reachable iff ``|L1 - m| - eps <= d <= L1 + m + eps`` with
    ``d = hypot(hypot(x, y), z - L0)`` measured from the joint1 pivot.
    joint3 is the L3 angle relative to L2 (radians).
    """
    for name, v in (("x", x), ("y", y), ("z", z)):
        if not math.isfinite(v):
            return False
    r = math.hypot(x, y)
    d = math.hypot(r, z - config.l0)
    min_reach, max_reach = _reach_bounds_with_joint3(config, joint3, opening)
    return (min_reach - REACH_EPSILON_MM) <= d <= (max_reach + REACH_EPSILON_MM)


# =====================================================================
# 8. JOINT-LIMIT VALIDATION
# =====================================================================


def check_joint_limits(
    theta_base: float,
    joint1: float,
    joint2: float,
    joint3: float,
    config: RobotConfig = DEFAULT_CONFIG,
) -> bool:
    """Check joint limits (all angles in DEGREES): True iff every joint is
    within its configured range (inclusive).  Independent of the IK math -
    validates whether the PHYSICAL arm could achieve a geometric solution.
    """
    angles = {
        "base": theta_base,
        "joint1": joint1,
        "joint2": joint2,
        "joint3": joint3,
    }
    for joint, value in angles.items():
        lo, hi = config.joint_limits[joint]
        if value < lo or value > hi:
            return False
    return True


def requires_joint_limits(
    theta_base: float,
    joint1: float,
    joint2: float,
    joint3: float,
    config: RobotConfig = DEFAULT_CONFIG,
) -> JointAngles:
    """Like check_joint_limits but raises ValueError when out of range."""
    if not check_joint_limits(theta_base, joint1, joint2, joint3, config):
        detail = {j: f"[{lo}, {hi}]" for j, (lo, hi) in config.joint_limits.items()}
        raise ValueError(
            f"Joint angles out of configured limits "
            f"({theta_base},{joint1},{joint2},{joint3}) deg; limits = {detail}."
        )
    return JointAngles(theta_base, joint1, joint2, joint3)


# =====================================================================
# 9. FK -> IK -> FK VERIFICATION
# =====================================================================


def verify_fk_ik(
    n_samples: int = 20,
    config: RobotConfig = DEFAULT_CONFIG,
    opening: float | None = None,
) -> Tuple[float, int, int]:
    """Round-trip harness: random angles -> FK -> XYZ -> IK -> FK (BOTH
    elbow configs).  Exact ANGLE recovery is asserted only for front-facing
    poses (signed radial projection r >= 0); backward-folded poses (r < 0)
    legitimately swap the gripper azimuth by 180 deg, so only the POSITION
    round trip is checked there.

    Returns (max_position_error_mm, samples_passed, samples_total) and
    prints a PASS/FAIL summary (acceptance bar POSITION_TOLERANCE_MM).
    """
    import random

    rng = random.Random(42)
    max_pos_err = 0.0
    max_angle_err_rad = 0.0
    passed = 0
    total = 0

    for _ in range(n_samples):
        base = rng.uniform(*config.joint_limits["base"])
        joint1 = rng.uniform(*config.joint_limits["joint1"])
        joint2 = rng.uniform(*config.joint_limits["joint2"])
        joint3 = rng.uniform(*config.joint_limits.get("joint3", (-150.0, 150.0)))

        target = forward_kinematics(base, joint1, joint2, joint3, config, opening)

        for flag, label in ((True, "joint2-up"), (False, "joint2-down")):
            total += 1
            try:
                recovered = inverse_kinematics(
                    target.x, target.y, target.z, joint2_up=flag, config=config, opening=opening, joint3=joint3
                )
            except ValueError as exc:
                print(f"  FAIL [{label}] angles ({base:.1f},{joint1:.1f},{joint2:.1f},{joint3:.1f}): {exc}")
                continue

            # FK(IK(x,y,z)) must reproduce the original Cartesian target.
            rebuilt = forward_kinematics(
                recovered.theta_base, recovered.joint1, recovered.joint2, recovered.joint3, config, opening
            )
            pos_err = math.hypot(
                rebuilt.x - target.x, math.hypot(rebuilt.y - target.y, rebuilt.z - target.z)
            )
            max_pos_err = max(max_pos_err, pos_err)

            # Assert exact ANGLE round-trip only when the IK flag matches
            # the ORIGINAL geometric elbow configuration (sign of
            # te' = joint2 + delta, NOT the physical joint2 sign) AND the
            # pose is front-facing (r >= 0); backward-folded poses swap
            # azimuth, so only the position check above applies to those.
            spread, _, l_g, _ = _gripper_pose_params(config, opening)
            phi3 = math.radians(joint3)
            real = config.l2 + (config.l3 + l_g) * math.cos(phi3)
            imag = (config.l3 + l_g) * math.sin(phi3)
            delta_rad = math.atan2(imag, real)
            orig_is_joint2_up = (_to_rad(joint2) + delta_rad) <= 0.0
            if flag == orig_is_joint2_up:
                _, _, l_g, _ = _gripper_pose_params(config, opening)
                stem_abs = _to_rad(joint1) + _to_rad(joint2) + math.radians(joint3)
                r_signed = (
                    config.l1 * math.cos(_to_rad(joint1))
                    + config.l2 * math.cos(_to_rad(joint1) + _to_rad(joint2))
                    + (config.l3 + l_g) * math.cos(stem_abs)
                )
                if r_signed >= 0.0:
                    a_err = max(
                        abs(_to_rad(recovered.theta_base) - _to_rad(base)),
                        abs(_to_rad(recovered.joint1) - _to_rad(joint1)),
                        abs(_to_rad(recovered.joint2) - _to_rad(joint2)),
                        abs(_to_rad(recovered.joint3) - _to_rad(joint3)),
                    )
                    max_angle_err_rad = max(max_angle_err_rad, a_err)

            if pos_err <= POSITION_TOLERANCE_MM:
                passed += 1
            else:
                print(
                    f"  FAIL [{label}] pos error {pos_err:9.6f} mm "
                    f"for angles ({base:.2f},{joint1:.2f},{joint2:.2f}) deg"
                )

    ok = passed == total and max_pos_err <= POSITION_TOLERANCE_MM
    status = "PASS" if ok else "FAIL"
    print("=" * 56)
    print(f"FK -> IK -> FK verification: {status}")
    print(f"Position error: {max_pos_err:.6f} mm  (tolerance {POSITION_TOLERANCE_MM} mm)")
    print(f"Max angle round-trip error: {math.degrees(max_angle_err_rad):.6f} deg")
    print(f"Samples: {passed}/{total} passed")
    print("=" * 56)
    return max_pos_err, passed, total


# =====================================================================
# 10. 3D VISUALIZATION (matplotlib)
# =====================================================================


def _draw_grip_octahedron(ax, center, axis, size, color="tab:green", label="grip point") -> None:
    ax.scatter(center.x, center.y, center.z, color="tab:red", s=25)


def _set_equal_aspect(ax, points) -> None:
    """Force equal visual scale on all three axes, with symmetric limits."""
    import numpy as np

    xs = np.array([p.x for p in points])
    ys = np.array([p.y for p in points])
    zs = np.array([p.z for p in points])
    span = max(
        float(xs.max() - xs.min()),
        float(ys.max() - ys.min()),
        float(zs.max() - zs.min()),
        1.0,
    )
    mid_x, mid_y, mid_z = (xs.max() + xs.min()) / 2, (ys.max() + ys.min()) / 2, (zs.max() + zs.min()) / 2
    ax.set_xlim(mid_x - span, mid_x + span)
    ax.set_ylim(mid_y - span, mid_y + span)
    ax.set_zlim(mid_z - span, mid_z + span)


def _apply_limits(ax, limits) -> None:
    """Set all three axis limits from a ``(x, y, z)`` tuple of (min, max) pairs."""
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_zlim(*limits[2])


def _plot_points_for_pose(
    angles: JointAngles,
    config: RobotConfig,
    opening: float | None,
    target: CartesianPoint | None,
) -> list:
    """The key plotted points of one pose (links, wrist, grip, fingertips, target)."""
    a_rad = JointAngles(_to_rad(angles.theta_base), _to_rad(angles.joint1), _to_rad(angles.joint2), _to_rad(angles.joint3))
    base, joint1_pos, joint2_pos, joint3, gripper_mount = _fk_parts(
        a_rad.theta_base, a_rad.joint1, a_rad.joint2, a_rad.joint3, config
    )
    gripper = _gripper_pose(a_rad.theta_base, a_rad.joint1, a_rad.joint2, a_rad.joint3, config, opening)
    points = [base, joint1_pos, joint2_pos, joint3, gripper_mount, gripper.grip_point]
    points += list(gripper.fingertips)
    if target is not None:
        points.append(target)
    return points


def animation_limits(
    poses: Sequence[JointAngles],
    config: RobotConfig = DEFAULT_CONFIG,
    opening: float | None = None,
    target: CartesianPoint | None = None,
):
    """Equal-aspect axis limits that enclose EVERY pose in a trajectory.

    Used by the animation so the axes stay rock-steady for the whole motion
    instead of re-scaling on every frame (which would zoom/pan the view).
    Falls back to a small unit box for an empty trajectory.
    """
    import numpy as np

    xs, ys, zs = [], [], []
    for angles in poses:
        points = _plot_points_for_pose(angles, config, opening, target)
        xs.extend(p.x for p in points)
        ys.extend(p.y for p in points)
        zs.extend(p.z for p in points)
    if not xs:
        return ((0.0, 1.0), (0.0, 1.0), (0.0, 1.0))
    span = max(
        max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), 1.0
    )
    mid_x, mid_y, mid_z = (
        (max(xs) + min(xs)) / 2,
        (max(ys) + min(ys)) / 2,
        (max(zs) + min(zs)) / 2,
    )
    return (
        (mid_x - span, mid_x + span),
        (mid_y - span, mid_y + span),
        (mid_z - span, mid_z + span),
    )


def draw_robot(
    ax,
    angles: JointAngles,
    config: RobotConfig = DEFAULT_CONFIG,
    target: CartesianPoint | None = None,
    opening: float | None = None,
    fixed_limits=None,
) -> None:
    """Draw the arm, fingers, grip point and optional target onto an existing
    3D ``Axes`` (degrees in).  Shared by the CLI and the tkinter GUI so both
    render identically; the axes are cleared first, the canvas is re-used
    for consecutive poses.  By default the view centres on both the pose and
    ``target``; pass ``fixed_limits`` (from :func:`animation_limits`) to
    keep the axes identical across a whole animation.  Returns
    ``plot_points``.
    """
    ax.clear()
    a_rad = JointAngles(_to_rad(angles.theta_base), _to_rad(angles.joint1), _to_rad(angles.joint2), _to_rad(angles.joint3))
    base, joint1_pos, joint2_pos, joint3, gripper_mount = _fk_parts(
        a_rad.theta_base, a_rad.joint1, a_rad.joint2, a_rad.joint3, config
    )
    gripper = _gripper_pose(a_rad.theta_base, a_rad.joint1, a_rad.joint2, a_rad.joint3, config, opening)

    # World axes from the base origin (RGB = X, Y, Z).
    m, _ = _grip_parameters(config, opening)
    axis_len = (config.l1 + m) * 0.5
    for origin, heading, colour, label in (
        ((0, 0, 0), (axis_len, 0, 0), "r", "X"),
        ((0, 0, 0), (0, axis_len, 0), "g", "Y"),
        ((0, 0, 0), (0, 0, axis_len), "b", "Z"),
    ):
        ax.quiver(*origin, *heading, length=axis_len, normalize=True, color=colour)
        ax.text(*tuple(h + axis_len * 0.55 for h in heading), label, color=colour, fontsize=10)

    # Base post (visual only, not part of the FK model).
    ax.plot([0, 0], [0, 0], [0, -axis_len * 0.2], color="k", linewidth=3)

    xs = [base.x, joint1_pos.x, joint2_pos.x, joint3.x, gripper_mount.x]
    ys = [base.y, joint1_pos.y, joint2_pos.y, joint3.y, gripper_mount.y]
    zs = [base.z, joint1_pos.z, joint2_pos.z, joint3.z, gripper_mount.z]
    ax.plot(xs[:2], ys[:2], zs[:2], color="k", linewidth=5, label="fixed L0")
    ax.plot(xs[1:], ys[1:], zs[1:], color="k", linewidth=4, label="tilt links")
    ax.scatter(*base.coords(), color="k", s=90, marker="o", label="base")
    ax.scatter(joint1_pos.x, joint1_pos.y, joint1_pos.z, color="tab:purple", s=90, marker="s", label="joint 1")
    ax.scatter(joint2_pos.x, joint2_pos.y, joint2_pos.z, color="tab:orange", s=90, marker="o", label="joint 2")
    ax.scatter(joint3.x, joint3.y, joint3.z, color="tab:blue", s=130, marker="*", label="joint 3")

    for k, tip in enumerate(gripper.fingertips):
        ax.plot([gripper_mount.x, tip.x], [gripper_mount.y, tip.y], [gripper_mount.z, tip.z], color="tab:cyan", linewidth=0.8)
        ax.scatter(tip.x, tip.y, tip.z, color="tab:cyan", s=15, marker="o", label="fingertip" if k == 0 else None)

    # Optional IK target marker.
    if target is not None:
        ax.scatter(target.x, target.y, target.z, color="tab:red", s=160, marker="+", label="target")
        ax.plot(
            [gripper_mount.x, target.x], [gripper_mount.y, target.y], [gripper_mount.z, target.z],
            color="tab:red", linestyle=":", linewidth=1,
        )

    ax.set_xlabel("X [mm]")
    ax.set_ylabel("Y [mm]")
    ax.set_zlabel("Z [mm]")
    ax.set_title(
        f"L0={config.l0:g}  L1={config.l1:g}  L2={config.l2:g}  L3={config.l3:g} mm\n"
        f"base yaw={angles.theta_base:7.2f}  joint1={angles.joint1:7.2f}  "
        f"joint2={angles.joint2:7.2f}  (deg)   opening={gripper.opening:.2f}"
    )
    ax.legend(loc="upper left", fontsize=8)
    plot_points = [base, joint1_pos, joint2_pos, joint3, gripper.grip_point]
    plot_points += list(gripper.fingertips)
    if target is not None:
        plot_points.append(target)
    if fixed_limits is not None:
        _apply_limits(ax, fixed_limits)
    else:
        _set_equal_aspect(ax, plot_points)
    return plot_points


def _new_3d_axes(figsize=(8, 7)):
    """Fresh matplotlib figure + empty 3D axes (lazy-imports matplotlib)."""
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (registers 3d projection)

    fig = plt.figure(figsize=figsize)
    return fig, fig.add_subplot(111, projection="3d")


def plot_robot(
    angles: JointAngles,
    config: RobotConfig = DEFAULT_CONFIG,
    target: CartesianPoint | None = None,
    opening: float | None = None,
) -> None:
    """3D plot of the arm and simulated gripper in its OWN window (blocks)."""
    import matplotlib.pyplot as plt

    fig, ax = _new_3d_axes()
    draw_robot(ax, angles, config, target, opening)
    plt.show()


def interpolate_joints(
    from_angles: JointAngles,
    to_angles: JointAngles,
    frames: int = 60,
) -> Tuple[JointAngles, ...]:
    """Joint-space interpolation between two poses (degrees in/out).

    Linear joint movement on a single shared cubic ease-in-out progress
    parameter (arm accelerates out of ``from``, cruises, eases into ``to``).
    ``theta_base`` takes the SHORTEST angular path (170 -> -170 sweeps
    through 180).  Returns exactly ``frames`` poses with
    ``poses[0] == from_angles`` and ``poses[-1] == to_angles``.
    """
    if frames < 2:
        raise ValueError(f"frames must be >= 2, got {frames}.")
    base_delta = math.degrees(_wrap_angle(math.radians(to_angles.theta_base - from_angles.theta_base)))
    j1_delta = to_angles.joint1 - from_angles.joint1
    j2_delta = to_angles.joint2 - from_angles.joint2
    j3_delta = to_angles.joint3 - from_angles.joint3

    poses = []
    for k in range(frames):
        t = k / (frames - 1)
        ease = t * t * (3.0 - 2.0 * t)  # smoothstep ease-in-out
        base = from_angles.theta_base + base_delta * ease
        if 0 < k < frames - 1:
            # Keep intermediate poses in (-180, 180] so the label/limits stay
            # canonical; the ±180 seam is visually continuous (arm is periodic).
            base = math.degrees(_wrap_angle(math.radians(base)))
        elif k == frames - 1:
            base = to_angles.theta_base  # exact endpoint (unwrapped equals it mod 360)
        poses.append(
            JointAngles(
                base,
                from_angles.joint1 + j1_delta * ease,
                from_angles.joint2 + j2_delta * ease,
                from_angles.joint3 + j3_delta * ease,
            )
        )
    return tuple(poses)


def plot_animation(
    from_angles: JointAngles,
    to_angles: JointAngles,
    config: RobotConfig = DEFAULT_CONFIG,
    opening: float | None = None,
    target: CartesianPoint | None = None,
    frames: int = 60,
    interval: int = 30,
) -> None:
    """Animate a smooth joint-space transition in its OWN window (blocks).

    Steps ``interpolate_joints(...)`` through the shared ``draw_robot()``
    primitive, keeping the optional red ``+`` target visible.  The 3D axes
    are pinned to ``animation_limits(...)`` so the view never re-scales
    while the arm moves.
    """
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    poses = interpolate_joints(from_angles, to_angles, frames)

    fig, ax = _new_3d_axes()
    # Whole-trajectory limits so the axes never re-scale while the arm moves.
    limits = animation_limits(poses, config, opening, target)

    def draw_frame(k: int) -> Tuple[list, ...]:
        draw_robot(ax, poses[k], config, target, opening, fixed_limits=limits)
        return ax.collections + ax.lines

    anim = FuncAnimation(fig, draw_frame, frames=frames, interval=interval,
                         blit=False, cache_frame_data=False, repeat=False)
    plt.show()
    # Keep a reference so the animation is not garbage-collected mid-play.
    fig._dume_anim = anim


# =====================================================================
# 11. ROBOT ARM FACADE (class-based usage for later integration)
# =====================================================================


class RobotArm:
    """Object-oriented facade binding a specific ``RobotConfig`` so callers
    (soon the Arduino/AI layers) never pass ``config`` to every call."""

    def __init__(self, config: RobotConfig = DEFAULT_CONFIG) -> None:
        self.config = config

    l0 = property(lambda s: s.config.l0)
    l1 = property(lambda s: s.config.l1)
    l2 = property(lambda s: s.config.l2)
    l3 = property(lambda s: s.config.l3)
    stem_angle = property(lambda s: s.config.wrist_stem_angle)

    def forward_kinematics(self, theta_base, joint1, joint2, opening=None) -> CartesianPoint:
        return forward_kinematics(theta_base, joint1, joint2, self.config, opening)

    def inverse_kinematics(self, x, y, z, joint2_up: bool = True, opening=None) -> JointAngles:
        return inverse_kinematics(x, y, z, joint2_up=joint2_up, config=self.config, opening=opening)

    def is_reachable(self, x, y, z, opening=None) -> bool:
        return is_reachable(x, y, z, self.config, opening)

    def check_joint_limits(self, theta_base, joint1, joint2) -> bool:
        return check_joint_limits(theta_base, joint1, joint2, self.config)

    def gripper_pose(self, theta_base, joint1, joint2, opening=None) -> FingerGripper:
        return gripper_pose(theta_base, joint1, joint2, self.config, opening)

    def plot(self, angles: JointAngles, target: CartesianPoint | None = None, opening=None) -> None:
        plot_robot(angles, self.config, target, opening)

    def verify(self, n_samples: int = 20, opening=None) -> Tuple[float, int, int]:
        return verify_fk_ik(n_samples, self.config, opening)


# =====================================================================
# 12-13. CLI AND main()
# =====================================================================


def _prompt_float(prompt: str) -> float:
    """Read a float from stdin, re-prompting until a valid number is given."""
    while True:
        raw = input(prompt).strip()
        try:
            return float(raw)
        except ValueError:
            print(f"  Invalid number: {raw!r}. Please enter a number (e.g. 45.0).")


def _prompt_angles() -> JointAngles:
    tb = _prompt_float("  theta_base     (deg): ")
    ts = _prompt_float("  joint1         (deg): ")
    te = _prompt_float("  joint2         (deg): ")
    tj = _prompt_float("  joint3         (deg): ")
    return JointAngles(tb, ts, te, tj)


def _prompt_xyz() -> CartesianPoint:
    x = _prompt_float("  x (mm): ")
    y = _prompt_float("  y (mm): ")
    z = _prompt_float("  z (mm): ")
    return CartesianPoint(x, y, z)


def _prompt_opening(config: RobotConfig) -> float | None:
    """Ask for a gripper opening; blank/enter keeps the config default."""
    raw = input(
        f"  gripper opening 0..1 (default {config.gripper_opening:g}, Enter = default): "
    ).strip()
    if raw == "":
        return None
    return float(raw)


def _prompt_frames(default: int = 30) -> int:
    """Ask for the animation frame count; blank/enter keeps the default."""
    raw = input(f"  animation frames (default {default}, Enter = default): ").strip()
    if raw == "":
        return default
    try:
        n = int(raw)
    except ValueError:
        print(f"  Invalid integer: {raw!r}. Using {default}.")
        return default
    return max(2, n)


JOINT3_MODES: Tuple[str, ...] = ("auto", "down")


def _parse_joint3_mode(raw: str) -> float | str:
    """Parse a joint3 entry: blank -> the -60 deg default, a number, or a mode
    string from :data:`JOINT3_MODES` (``auto`` / ``down``).  Shared by the CLI
    prompt and the GUI so both accept the same vocabulary."""
    raw = raw.strip()
    if raw == "":
        return -60.0
    lowered = raw.lower()
    if lowered in JOINT3_MODES:
        return lowered
    try:
        return float(raw)
    except ValueError:
        raise ValueError(
            f"Invalid joint3 {raw!r}: enter degrees, or one of {', '.join(JOINT3_MODES)}."
        )


def _prompt_ik_target(config: RobotConfig):
    """Shared options 2/5/7 block: target XYZ + elbow mode + opening + joint3 -> IK.

    Returns ``(p, joint2_up, opening, angles)`` or ``None`` when the input
    is rejected (an ERROR/note was already printed).
    """
    p = _prompt_xyz()
    joint2_up = input("  joint2 pose [up/down] (up): ").strip().lower() != "down"
    joint3_str = input("  joint3 (deg, 'auto', or 'down' for a straight-down wrist) (-60): ").strip()
    try:
        joint3 = _parse_joint3_mode(joint3_str)
    except ValueError as exc:
        print(f"\n  ERROR: {exc}")
        return None
    opening = _prompt_opening(config)

    # Reachability can only be pre-checked once joint3 is a concrete angle.
    if not isinstance(joint3, str) and not is_reachable(
        p.x, p.y, p.z, config, math.radians(joint3), opening
    ):
        print("\n  ERROR: target is outside the geometric workspace.")
        return None
    try:
        a = inverse_kinematics(
            p.x, p.y, p.z, joint2_up=joint2_up, config=config, opening=opening, joint3=joint3
        )
    except ValueError as exc:
        print(f"\n  ERROR: {exc}")
        return None
    return p, joint2_up, opening, a


MENU = """
================================
       DUM-E 4-DOF ROBOT
================================

1. Forward Kinematics
2. Inverse Kinematics
3. Check Reachability
4. Visualize From Joint Angles
5. Visualize From XYZ Target
6. Run FK -> IK Verification
7. Animate To Target (smooth)
8. Exit
================================
"""


def cli(config: RobotConfig = DEFAULT_CONFIG) -> None:
    """Text menu driving the kinematics + visualization stages."""
    last_angles = HOME_ANGLES  # pose the animation transitions FROM
    while True:
        print(MENU)
        choice = input("Select option: ").strip()

        if choice == "1":  # Forward Kinematics
            a = _prompt_angles()
            opening = _prompt_opening(config)
            p = forward_kinematics(a.theta_base, a.joint1, a.joint2, config, opening)
            last_angles = a
            print(
                f"\n  Grip point: x={p.x:.3f} mm, y={p.y:.3f} mm, z={p.z:.3f} mm  "
                f"(opening {config.gripper_opening if opening is None else opening:.3f})"
            )
            gp = gripper_pose(a.theta_base, a.joint1, a.joint2, config, opening)
            for k, tip in enumerate(gp.fingertips):
                print(
                    f"  Finger {k + 1} tip: x={tip.x:.3f} mm, y={tip.y:.3f} mm, "
                    f"z={tip.z:.3f} mm"
                )

        elif choice == "2":  # Inverse Kinematics
            got = _prompt_ik_target(config)
            if got is None:
                continue
            _, joint2_up, _, a = got
            hits = check_joint_limits(a.theta_base, a.joint1, a.joint2, a.joint3, config)
            print(
                f"\n  theta_base={a.theta_base:8.3f} deg\n"
                f"  joint1={a.joint1:8.3f} deg\n"
                f"  joint2={a.joint2:8.3f} deg\n"
                f"  joint3={a.joint3:8.3f} deg"
            )
            print(f"  within joint limits: {'yes' if hits else 'NO — out of range'}")

        elif choice == "3":  # Check Reachability
            p = _prompt_xyz()
            joint3 = _prompt_float("  joint3         (deg): ")
            opening = _prompt_opening(config)
            if is_reachable(p.x, p.y, p.z, config, math.radians(joint3), opening):
                print("\n  REACHABLE (geometrically), manipulator limits may still apply.")
            else:
                print("\n  UNREACHABLE for this arm.")

        elif choice == "4":  # Visualize From Joint Angles
            a = _prompt_angles()
            opening = _prompt_opening(config)
            p = forward_kinematics(a.theta_base, a.joint1, a.joint2, a.joint3, config, opening)
            print(f"\n  Grip point: x={p.x:.3f} mm, y={p.y:.3f} mm, z={p.z:.3f} mm")
            if not check_joint_limits(a.theta_base, a.joint1, a.joint2, a.joint3, config):
                print("  NOTE: angles exceed configured joint limits.")
            last_angles = a
            plot_robot(a, config=config, opening=opening)

        elif choice == "5":  # Visualize From XYZ Target
            got = _prompt_ik_target(config)
            if got is None:
                continue
            p, joint2_up, opening, a = got
            print(
                f"\n  IK result: base={a.theta_base:.3f}, joint1={a.joint1:.3f}, "
                f"joint2={a.joint2:.3f}, joint3={a.joint3:.3f} (deg)"
            )
            rebuilt = forward_kinematics(
                a.theta_base, a.joint1, a.joint2, a.joint3, config, opening
            )
            print(
                f"  FK(recomputed): x={rebuilt.x:.3f}, y={rebuilt.y:.3f}, z={rebuilt.z:.3f} mm"
            )
            if not check_joint_limits(a.theta_base, a.joint1, a.joint2, a.joint3, config):
                print("  NOTE: solution exceeds configured joint limits.")
            last_angles = a
            plot_robot(a, config=config, target=p, opening=opening)

        elif choice == "6":  # Verification
            opening = _prompt_opening(config)
            verify_fk_ik(config=config, opening=opening)

        elif choice == "7":  # Animate To Target
            got = _prompt_ik_target(config)
            if got is None:
                continue
            p, _, opening, a = got
            frames = _prompt_frames()
            if all(
                abs(getattr(last_angles, f) - getattr(a, f)) < 1e-3
                for f in ("theta_base", "joint1", "joint2", "joint3")
            ):
                print("\n  NOTE: start and target poses are identical - nothing to animate.")
                continue
            print(
                f"\n  Animating joint-space transition:"
                f"\n    start  base={last_angles.theta_base:8.3f}  "
                f"joint1={last_angles.joint1:8.3f}  joint2={last_angles.joint2:8.3f}  joint3={last_angles.joint3:8.3f} deg"
                f"\n    target base={a.theta_base:8.3f}  "
                f"joint1={a.joint1:8.3f}  joint2={a.joint2:8.3f}  joint3={a.joint3:8.3f} deg"
            )
            if not check_joint_limits(a.theta_base, a.joint1, a.joint2, a.joint3, config):
                print("  NOTE: target pose exceeds configured joint limits.")
            plot_animation(
                last_angles, a, config=config, opening=opening, target=p, frames=frames
            )
            last_angles = a

        elif choice == "8":  # Exit
            print("Goodbye.")
            break

        else:
            print("Unknown option. Try 1-8.")


def main() -> None:
    cli()


if __name__ == "__main__":
    main()