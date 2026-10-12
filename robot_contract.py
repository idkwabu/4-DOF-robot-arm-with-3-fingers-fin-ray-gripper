"""Shared physical joint ranges used by the brain, GUI, and kinematics APIs."""

from types import MappingProxyType
from typing import Mapping

JOINT_LIMITS: Mapping[str, tuple[float, float]] = MappingProxyType(
    {
        "base": (-180.0, 180.0),
        "joint1": (0.0, 180.0),
        "joint2": (-150.0, 150.0),
        "joint3": (-150.0, 150.0),
    }
)
