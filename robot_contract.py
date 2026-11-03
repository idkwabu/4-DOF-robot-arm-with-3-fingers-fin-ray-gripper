"""Shared physical joint ranges used by the brain, GUIs, kinematics, and servo bridge."""

from types import MappingProxyType
from typing import Mapping

JOINT_LIMITS: Mapping[str, tuple[float, float]] = MappingProxyType(
    {
        "base": (-180.0, 180.0),
        "joint1": (-90.0, 90.0),
        "joint2": (-150.0, 150.0),
        "joint3": (-120.0, 120.0),
        "gripper_roll": (-180.0, 180.0),
    }
)
