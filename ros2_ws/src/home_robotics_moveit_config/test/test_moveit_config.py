"""Structural contracts for the initial dual-Panda MoveIt configuration."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory


EXPECTED_GROUPS = {
    "panda1_manipulator",
    "panda2_manipulator",
    "panda1_gripper",
    "panda2_gripper",
}


def _shares():
    return (
        Path(get_package_share_directory("home_robotics_moveit_config")),
        Path(get_package_share_directory("home_robotics_description")),
    )


def test_srdf_references_robot_model_and_preserves_inter_robot_collisions():
    moveit_share, description_share = _shares()
    srdf = ET.parse(moveit_share / "config/home_robotics.srdf").getroot()
    rendered = subprocess.check_output(
        ["xacro", str(description_share / "urdf/home_robotics.urdf.xacro")], text=True
    )
    urdf = ET.fromstring(rendered)
    links = {node.attrib["name"] for node in urdf.findall("link")}
    joints = {node.attrib["name"] for node in urdf.findall("joint")}

    groups = {group.attrib["name"]: group for group in srdf.findall("group")}
    assert set(groups) == EXPECTED_GROUPS
    for group in groups.values():
        for chain in group.findall("chain"):
            assert chain.attrib["base_link"] in links
            assert chain.attrib["tip_link"] in links
        for joint in group.findall("joint"):
            assert joint.attrib["name"] in joints
    for end_effector in srdf.findall("end_effector"):
        assert end_effector.attrib["parent_link"] in links
        assert end_effector.attrib["group"] in groups
        assert end_effector.attrib["parent_group"] in groups

    disabled = [
        {entry.attrib["link1"], entry.attrib["link2"]}
        for entry in srdf.findall("disable_collisions")
    ]
    assert not any(
        any(link.startswith("panda1_") for link in pair)
        and any(link.startswith("panda2_") for link in pair)
        for pair in disabled
    )
    assert not any(
        "shared_rail" in pair
        and any(link.startswith(("panda1_link", "panda2_link")) for link in pair)
        for pair in disabled
    )


def test_controller_mapping_matches_existing_controller_contract():
    moveit_share, _ = _shares()
    with (moveit_share / "config/moveit_controllers.yaml").open() as stream:
        config = yaml.safe_load(stream)
    manager = config["moveit_simple_controller_manager"]
    assert set(manager["controller_names"]) == {
        "panda1_trajectory_controller",
        "panda2_trajectory_controller",
        "panda1_gripper_controller",
        "panda2_gripper_controller",
    }
    for prefix in ("panda1_", "panda2_"):
        arm = manager[prefix + "trajectory_controller"]
        assert arm["type"] == "FollowJointTrajectory"
        assert arm["joints"] == [prefix + "rail_joint"] + [prefix + f"joint{i}" for i in range(1, 8)]
        gripper = manager[prefix + "gripper_controller"]
        assert gripper["type"] == "GripperCommand"
        assert gripper["joints"] == [prefix + "finger_joint1"]


def test_static_environment_contract_uses_authoritative_scene():
    moveit_share, _ = _shares()
    with (moveit_share / "config/scene.yaml").open() as stream:
        scene = yaml.safe_load(stream)["scene"]

    assert scene["frame"] == "world"
    assert list(scene["surfaces"]) == [
        "surface_left_1", "surface_left_2", "surface_left_3",
        "surface_right_1", "surface_right_2", "surface_right_3",
    ]
    assert all(surface["dimensions"] == [0.825, 0.75, 0.08]
               for surface in scene["surfaces"].values())
    assert scene["table_geometry"]["leg_dimensions"] == [0.06, 0.06, 0.60]
    assert scene["floor"]["arena_border"] == {
        "dimensions": [6.0, 5.0], "width": 0.06, "height": 0.005,
        "floor_clearance": 0.005, "material": "arena_border",
    }
    static_ids = {
        scene["floor"]["id"],
        "arena_front", "arena_back", "arena_left", "arena_right",
        *scene["surfaces"],
        *scene["shared_rail"]["supports"]["names"],
    }
    assert len(static_ids) == 14
    assert static_ids.isdisjoint({"cube", "apple", "purple_ball", "bowl", "pan"})
    assert (moveit_share / "launch/planning_scene_environment.launch.py").is_file()
    executable = (Path(get_package_prefix("home_robotics_moveit_config"))
                  / "lib/home_robotics_moveit_config/planning_scene_environment")
    assert executable.is_file()
