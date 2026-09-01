"""Reusable import path for the vendored Franka Panda MJCF model."""
from __future__ import annotations
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping
from xml.etree import ElementTree as ET
from .config_loader import ConfigError

class PandaModelSource:
    """Merge shared Panda assets/defaults and create prefixed physical instances."""

    ASSET_NAMESPACE = "franka"
    VIRTUAL_ASSET_ROOT = "franka_emika_panda/assets"

    def __init__(self, model_path: Path) -> None:
        self.model_path = model_path.resolve()
        if not self.model_path.is_file():
            raise ConfigError(f"Configured Panda model does not exist: {self.model_path}")
        self.root = ET.fromstring(self.model_path.read_text(encoding="utf-8"))
        self._asset_names = self._collect_asset_names()
        self._class_names = self._collect_class_names()

    def _collect_asset_names(self) -> dict[str, str]:
        names: dict[str, str] = {}
        asset = self.root.find("asset")
        if asset is None:
            raise ConfigError(f"Panda model has no asset section: {self.model_path}")
        for element in asset:
            original = element.get("name")
            if original is None and element.get("file"):
                original = Path(str(element.get("file"))).stem
            if original:
                names[original] = f"{self.ASSET_NAMESPACE}_{original}"
        return names

    def _collect_class_names(self) -> dict[str, str]:
        return {
            str(element.get("class")): f"{self.ASSET_NAMESPACE}_{element.get('class')}"
            for element in self.root.findall(".//default[@class]")
        }

    def _rewrite_shared_references(self, element: ET.Element) -> None:
        for node in element.iter():
            for attribute in ("class", "childclass"):
                value = node.get(attribute)
                if value in self._class_names:
                    node.set(attribute, self._class_names[value])
            for attribute in ("mesh", "material"):
                value = node.get(attribute)
                if value in self._asset_names:
                    node.set(attribute, self._asset_names[value])

    def add_shared_definitions(self, target_root: ET.Element) -> None:
        target_asset = target_root.find("asset")
        if target_asset is None:
            target_asset = ET.SubElement(target_root, "asset")
        source_asset = self.root.find("asset")
        assert source_asset is not None
        for source in source_asset:
            clone = deepcopy(source)
            original = clone.get("name")
            if original is None and clone.get("file"):
                original = Path(str(clone.get("file"))).stem
            if original:
                clone.set("name", self._asset_names[original])
            if clone.get("file"):
                clone.set("file", f"{self.VIRTUAL_ASSET_ROOT}/{clone.get('file')}")
            self._rewrite_shared_references(clone)
            target_asset.append(clone)

        target_default = target_root.find("default")
        source_default = self.root.find("default")
        if target_default is None or source_default is None:
            raise ConfigError("Both world and Panda model require default sections")
        for source in source_default:
            clone = deepcopy(source)
            self._rewrite_shared_references(clone)
            target_default.append(clone)

    def _prefix_instance(self, element: ET.Element, prefix: str) -> None:
        for node in element.iter():
            if node.get("name"):
                node.set("name", f"{prefix}{node.get('name')}")
            for attribute in ("joint", "joint1", "joint2", "body1", "body2"):
                if node.get(attribute):
                    node.set(attribute, f"{prefix}{node.get(attribute)}")
        self._rewrite_shared_references(element)

    def instantiate(self, target_root: ET.Element, base_body: ET.Element, robot: Mapping[str, Any], *, gravity_compensation: float = 0.0) -> None:
        source_body = self.root.find("worldbody/body")
        if source_body is None:
            raise ConfigError(f"Panda model has no root body: {self.model_path}")
        prefix = str(robot["prefix"])
        body = deepcopy(source_body)
        self._prefix_instance(body, prefix)
        for panda_body in body.iter("body"):
            panda_body.set("gravcomp", str(gravity_compensation))
        hand = body.find(f".//body[@name='{prefix}hand']")
        if hand is None:
            raise ConfigError("Panda model does not contain the expected hand body")
        tcp = robot["gripper"]["tcp_offset"]
        ET.SubElement(
            hand, "site", name=robot["gripper"]["tcp_frame"],
            pos=" ".join(map(str, tcp["position"])),
            quat=" ".join(map(str, tcp["quaternion_wxyz"])),
            size="0.005", rgba="0.0 0.8 0.2 1.0",
        )
        base_body.append(body)

        target_equality = target_root.find("equality")
        if target_equality is None:
            target_equality = ET.SubElement(target_root, "equality")
        source_equality = self.root.find("equality")
        if source_equality is not None:
            for source in source_equality:
                clone = deepcopy(source)
                self._prefix_instance(clone, prefix)
                target_equality.append(clone)

        target_contact = target_root.find("contact")
        if target_contact is None:
            target_contact = ET.SubElement(target_root, "contact")
        source_contact = self.root.find("contact")
        if source_contact is not None:
            for source in source_contact:
                clone = deepcopy(source)
                self._prefix_instance(clone, prefix)
                target_contact.append(clone)

    def virtual_assets(self) -> dict[str, bytes]:
        asset_dir = self.model_path.parent / "assets"
        if not asset_dir.is_dir():
            raise ConfigError(f"Panda asset directory is missing: {asset_dir}")
        return {
            f"{self.VIRTUAL_ASSET_ROOT}/{path.name}": path.read_bytes()
            for path in asset_dir.iterdir() if path.is_file()
        }
