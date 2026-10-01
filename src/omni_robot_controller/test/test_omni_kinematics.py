import math

import pytest

from omni_robot_controller.omni_kinematics_node import (
    compute_wheel_angular_velocities,
    limit_wheel_velocities,
)


ANGLES = tuple(math.radians(a) for a in (60.0, -60.0, 180.0))
RADIUS = 0.032
BASE_RADIUS = 0.115


def test_forward_kinematics():
    wheels = compute_wheel_angular_velocities(
        0.1, 0.0, 0.0, ANGLES, RADIUS, BASE_RADIUS
    )
    assert wheels[0] == pytest.approx(-2.706329, rel=1e-5)
    assert wheels[1] == pytest.approx(2.706329, rel=1e-5)
    assert wheels[2] == pytest.approx(0.0, abs=1e-9)


def test_lateral_kinematics():
    wheels = compute_wheel_angular_velocities(
        0.0, 0.1, 0.0, ANGLES, RADIUS, BASE_RADIUS
    )
    assert wheels[0] == pytest.approx(1.5625, rel=1e-6)
    assert wheels[1] == pytest.approx(1.5625, rel=1e-6)
    assert wheels[2] == pytest.approx(-3.125, rel=1e-6)


def test_rotation_kinematics():
    wheels = compute_wheel_angular_velocities(
        0.0, 0.0, 0.5, ANGLES, RADIUS, BASE_RADIUS
    )
    expected = BASE_RADIUS * 0.5 / RADIUS
    assert wheels == pytest.approx([expected] * 3, rel=1e-6)


def test_velocity_limit_preserves_ratio():
    wheels = [30.0, -15.0, 0.0]
    limited = limit_wheel_velocities(wheels, 20.0)
    assert limited == pytest.approx([20.0, -10.0, 0.0], rel=1e-6)


def test_zero_limit_stops_all_wheels():
    assert limit_wheel_velocities([1.0, -2.0, 3.0], 0.0) == [0.0, 0.0, 0.0]
