"""`robot.name` in the config picks the robot12idb class, not just the ini file.

The models are not labels: the UR5 carries a tool changer, so its TCP sits
45 mm further out than the UR3's. Building the wrong class drives the arm with
the other one's offsets and puts every taught descent 45 mm out.
"""
from pathlib import Path
import sys
import types

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from palmixer.server import _robot_model


def fake_robot12idb():
    """The shape of the real module: two model classes off a shared base, plus
    unrelated names that must not be mistaken for models."""
    mod = types.SimpleNamespace()

    class UR_cam_grip:
        pass

    class UR3(UR_cam_grip):
        tcp = [0.0, 0.0, 0.15, 0.0, 0.0, 0.0]

    class UR5(UR_cam_grip):
        tcp = [0.0, 0.0, 0.195, 0.0, 0.0, 0.0]

    class ToolChangerException(Exception):
        pass

    mod.UR_cam_grip = UR_cam_grip
    mod.UR3 = UR3
    mod.UR5 = UR5
    mod.ToolChangerException = ToolChangerException
    mod.april_tag_size = {"heater": 0.012}
    return mod


@pytest.mark.parametrize("name,expected_z", [("UR3", 0.15), ("UR5", 0.195)])
def test_the_name_selects_the_matching_class(name, expected_z):
    cls = _robot_model(fake_robot12idb(), name)
    assert cls.__name__ == name
    assert cls.tcp[2] == expected_z


@pytest.mark.parametrize("name", ["ur5", "Ur5", "UR5"])
def test_matching_is_case_insensitive(name):
    assert _robot_model(fake_robot12idb(), name).__name__ == "UR5"


def test_an_unknown_model_is_refused_rather_than_defaulting_to_ur3():
    """Falling back would silently drive a UR5 as a UR3 -- the exact failure
    this function exists to prevent."""
    with pytest.raises(ValueError) as exc:
        _robot_model(fake_robot12idb(), "UR10")
    message = str(exc.value)
    assert "UR10" in message
    assert "UR3, UR5" in message, "the error should list what is available"


def test_non_model_classes_are_not_offered_as_models():
    """ToolChangerException is a class in the same module and must not be
    reachable as a robot model."""
    with pytest.raises(ValueError) as exc:
        _robot_model(fake_robot12idb(), "ToolChangerException")
    assert "ToolChangerException" not in str(exc.value).split("Available:")[1]


def test_the_real_module_exposes_both_arms():
    """Guards the base-class discriminator against a UR_12idb refactor."""
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))
    robot12idb = pytest.importorskip("robot12idb",
                                     reason="UR_12idb not importable")
    ur3 = _robot_model(robot12idb, "UR3")
    ur5 = _robot_model(robot12idb, "UR5")
    assert ur3 is robot12idb.UR3 and ur5 is robot12idb.UR5
    # The 45 mm that makes this matter at all.
    assert ur5.tcp[2] - ur3.tcp[2] == pytest.approx(0.045)


def test_both_real_classes_accept_the_arguments_the_server_passes():
    """The server calls cls(name=..., ip=...). UR5's signature differs from
    UR3's (it has no use_rtde), so this is not a given."""
    import inspect
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "UR_12idb"))
    robot12idb = pytest.importorskip("robot12idb",
                                     reason="UR_12idb not importable")
    for name in ("UR3", "UR5"):
        params = inspect.signature(_robot_model(robot12idb, name).__init__).parameters
        assert {"name", "ip"} <= set(params), "%s cannot be built the way the server builds it" % name
