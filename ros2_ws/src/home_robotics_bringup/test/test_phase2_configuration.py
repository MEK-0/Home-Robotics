"""Static Phase 2 description and controller-contract tests."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = ROOT / "home_robotics_description/urdf/home_robotics.urdf.xacro"
CONTROL_XACRO = ROOT / "home_robotics_description/urdf/home_robotics.ros2_control.xacro"
CONTROLLERS = ROOT / "home_robotics_bringup/config/controllers.yaml"


def rendered_robot():
    output = subprocess.check_output(["xacro", str(DESCRIPTION)], text=True)
    return ET.fromstring(output)


def expected_joints():
    names = []
    for robot in ("panda1", "panda2"):
        names.extend([f"{robot}_rail_joint"])
        names.extend(f"{robot}_joint{index}" for index in range(1, 8))
        names.extend([f"{robot}_finger_joint1", f"{robot}_finger_joint2"])
    return set(names)


def test_description_contains_all_phase2_joints_and_unique_links():
    robot = rendered_robot()
    joints = {joint.attrib["name"] for joint in robot.findall("joint")}
    links = [link.attrib["name"] for link in robot.findall("link")]
    assert expected_joints() <= joints
    assert len(links) == len(set(links))


def test_description_contains_required_frames_and_correct_rail_axes():
    robot = rendered_robot()
    links = {link.attrib["name"] for link in robot.findall("link")}
    assert {"world", "shared_rail", "panda1_tcp", "panda2_tcp"} <= links
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    for name in ("panda1_rail_joint", "panda2_rail_joint"):
        assert joints[name].find("axis").attrib["xyz"] == "1 0 0"


def test_ros2_control_declares_twenty_state_joints_and_eighteen_commands():
    robot = rendered_robot()
    control = robot.find("ros2_control")
    assert control is not None
    joints = control.findall("joint")
    assert len(joints) == 20
    assert sum(bool(joint.findall("command_interface")) for joint in joints) == 18
    assert all(len(joint.findall("state_interface")) == 2 for joint in joints)


def test_controller_groups_are_disjoint_and_cover_both_robots():
    config = yaml.safe_load(CONTROLLERS.read_text(encoding="utf-8"))
    panda1 = config["panda1_trajectory_controller"]["ros__parameters"]["joints"]
    panda2 = config["panda2_trajectory_controller"]["ros__parameters"]["joints"]
    assert len(panda1) == len(panda2) == 8
    assert not set(panda1) & set(panda2)
    assert panda1[0] == "panda1_rail_joint"
    assert panda2[0] == "panda2_rail_joint"


def test_minimum_separation_is_declared_at_hardware_boundary():
    text = CONTROL_XACRO.read_text(encoding="utf-8")
    assert '<param name="minimum_rail_separation">0.7</param>' in text
