from karaburan_simulation.simcontrolnode import mix_thruster_forces
import pytest


def test_straight_thrust_uses_equal_force_commands():
    left, right = mix_thruster_forces(0.25, 0.0, 5.0)

    assert left == 0.25
    assert right == 0.25


def test_steering_changes_symmetric_differential_thrust():
    left, right = mix_thruster_forces(0.25, 0.08, 5.0)

    assert left == pytest.approx(-0.15)
    assert right == pytest.approx(0.65)
