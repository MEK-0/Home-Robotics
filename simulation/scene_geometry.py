"""Configuration-driven scene transform and primitive helpers."""
from __future__ import annotations

import math
from typing import Any, Mapping

from .config_loader import ConfigBundle, ConfigError


def quaternion_multiply(first: tuple[float, ...], second: tuple[float, ...]) -> tuple[float, float, float, float]:
    w1, x1, y1, z1 = first
    w2, x2, y2, z2 = second
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def rotate_vector(quaternion: tuple[float, ...], vector: tuple[float, ...]) -> tuple[float, float, float]:
    _, x, y, z = quaternion_multiply(
        quaternion_multiply(quaternion, (0.0, *vector)),
        (quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]),
    )
    return x, y, z


def resolve_object_pose(config: ConfigBundle, obj: Mapping[str, Any]) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    initial = obj["initial"]
    frame = str(initial["reference_frame"])
    local_position = tuple(map(float, initial["position"]))
    local_quaternion = tuple(map(float, initial["quaternion_wxyz"]))
    if frame == "world":
        return local_position, local_quaternion
    try:
        surface_pose = config.scene["surfaces"][frame]["pose"]
    except KeyError as exc:
        raise ConfigError(f"Object '{obj['id']}' uses unknown initial reference frame '{frame}'") from exc
    parent_position = tuple(map(float, surface_pose["position"]))
    parent_quaternion = tuple(map(float, surface_pose["quaternion_wxyz"]))
    rotated = rotate_vector(parent_quaternion, local_position)
    return tuple(parent_position[index] + rotated[index] for index in range(3)), quaternion_multiply(parent_quaternion, local_quaternion)


def yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    return math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)
