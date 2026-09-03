"""Installed Panda visual-mesh contract tests."""
from pathlib import Path
import re
import subprocess

from ament_index_python.packages import get_package_share_directory


def test_all_rendered_package_mesh_uris_resolve():
    share = Path(get_package_share_directory("home_robotics_description"))
    xacro = share / "urdf/home_robotics.urdf.xacro"
    rendered = subprocess.check_output(["xacro", str(xacro)], text=True)
    uris = set(re.findall(r'package://home_robotics_description/([^\"]+)', rendered))
    assert len(uris) == 56
    assert all((share / uri).is_file() for uri in uris)


def test_both_pandas_reference_the_same_validated_mesh_set():
    share = Path(get_package_share_directory("home_robotics_description"))
    rendered = subprocess.check_output(
        ["xacro", str(share / "urdf/home_robotics.urdf.xacro")], text=True
    )
    assert rendered.count("link0_0.obj") == 2
    assert rendered.count("link7_7.obj") == 2
    assert rendered.count("hand_4.obj") == 2
    assert rendered.count("finger_0.obj") == 4
