from __future__ import annotations
import shutil
from pathlib import Path
import pytest
import yaml
from simulation.config_loader import ConfigError, ConfigLoader
ROOT = Path(__file__).resolve().parents[2]
def test_config_files_load_successfully():
    config = ConfigLoader(ROOT / "config").load()
    assert config.scene["frame"] == "world"
    assert set(config.objects) == {"cube", "apple", "purple_ball", "bowl", "pan"}
    assert config.physics["timestep"] > 0
def test_missing_config_file_fails_clearly(tmp_path):
    shutil.copytree(ROOT / "config", tmp_path / "config"); (tmp_path / "config" / "physics.yaml").unlink()
    with pytest.raises(ConfigError, match=r"Required configuration file is missing: .*physics\.yaml"): ConfigLoader(tmp_path / "config").load()
def test_invalid_required_fields_fail_clearly(tmp_path):
    shutil.copytree(ROOT / "config", tmp_path / "config"); path = tmp_path / "config" / "physics.yaml"
    document = yaml.safe_load(path.read_text()); del document["physics"]["timestep"]; path.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match=r"physics missing required field\(s\): timestep"): ConfigLoader(tmp_path / "config").load()
def test_invalid_reference_fails_clearly(tmp_path):
    shutil.copytree(ROOT / "config", tmp_path / "config"); path = tmp_path / "config" / "objects.yaml"
    document = yaml.safe_load(path.read_text()); document["objects"]["cube"]["grasp_profile"] = "missing_profile"; path.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="unknown grasp profile 'missing_profile'"): ConfigLoader(tmp_path / "config").load()
def test_duplicate_canonical_id_is_rejected(tmp_path):
    shutil.copytree(ROOT / "config", tmp_path / "config")
    path = tmp_path / "config" / "objects.yaml"
    path.write_text("objects:\n  cube: {}\n  cube: {}\n")
    with pytest.raises(ConfigError, match="Duplicate canonical ID"):
        ConfigLoader(tmp_path / "config").load()

def test_unsafe_shared_rail_home_separation_is_rejected(tmp_path):
    shutil.copytree(ROOT / "config", tmp_path / "config")
    path = tmp_path / "config" / "robots.yaml"
    document = yaml.safe_load(path.read_text())
    document["robots"]["panda2"]["rail"]["home_position"] = -0.3
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="minimum separation or ordering"):
        ConfigLoader(tmp_path / "config").load()

def test_carriage_geometry_must_remain_on_shared_support(tmp_path):
    shutil.copytree(ROOT / "config", tmp_path / "config")
    path = tmp_path / "config" / "robots.yaml"
    document = yaml.safe_load(path.read_text())
    document["robots"]["panda1"]["rail"]["lower_limit"] = -1.6
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="limits must lie inside shared rail limits|geometry can leave"):
        ConfigLoader(tmp_path / "config").load()
