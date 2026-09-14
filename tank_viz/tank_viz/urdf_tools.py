"""URDF helpers for ghost (command preview) robot model."""
from __future__ import annotations

import xml.etree.ElementTree as ET


def make_ghost(urdf_str: str, prefix: str = "cmd_", alpha: float = 0.3) -> str:
    """Prefix all link/joint names and force visual material alpha."""
    root = ET.fromstring(urdf_str)
    if root.tag != "robot":
        raise ValueError("expected <robot> root")

    root.set("name", prefix + (root.get("name") or "tank"))

    # Collect original names then rewrite references
    link_names = {el.get("name") for el in root.findall("link") if el.get("name")}
    joint_names = {el.get("name") for el in root.findall("joint") if el.get("name")}

    for link in root.findall("link"):
        name = link.get("name")
        if name:
            link.set("name", prefix + name)
        for visual in link.findall("visual"):
            mat = visual.find("material")
            if mat is None:
                mat = ET.SubElement(visual, "material", name="")
            color = mat.find("color")
            if color is None:
                color = ET.SubElement(mat, "color")
                color.set("rgba", f"0.2 0.8 1.0 {alpha}")
            else:
                rgba = (color.get("rgba") or "0.5 0.5 0.5 1").split()
                if len(rgba) < 4:
                    rgba = ["0.2", "0.8", "1.0", str(alpha)]
                else:
                    rgba[3] = str(alpha)
                color.set("rgba", " ".join(rgba))

    for joint in root.findall("joint"):
        name = joint.get("name")
        if name:
            joint.set("name", prefix + name)
        for tag in ("parent", "child"):
            el = joint.find(tag)
            if el is not None and el.get("link") in link_names:
                el.set("link", prefix + el.get("link"))

    # Keep unused joint_names referenced for clarity / future mimic tags
    _ = joint_names

    xml_body = ET.tostring(root, encoding="unicode")
    return '<?xml version="1.0" encoding="utf-8"?>\n' + xml_body + "\n"
