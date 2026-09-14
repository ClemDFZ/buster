#!/usr/bin/env python3
"""Post-process SolidWorks URDF export into a Gazebo Harmonic-ready tank_sim.urdf.

Idempotent: reading the generated URDF again is a no-op for structural edits
(gazebo blocks are stripped and re-appended).
"""
from __future__ import annotations

import argparse
import re
import shutil
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

# ---------------------------------------------------------------------------
# Sim constants (header of script, as planned)
# ---------------------------------------------------------------------------
OPTICAL_RPY = "-1.5708 0 -1.5708"
# SolidWorks realsense_link / CSI_link are ALREADY optical-oriented (Z fwd).
# Extra optical children stay identity for naming; Gazebo cameras look along +X so
# need this pose on the sensor (maps optical → Gazebo camera_link).
GZ_CAM_POSE_ON_OPTICAL = "0 0 0 0 -1.57079632679 1.57079632679"

# RealSense D415
RS_WIDTH, RS_HEIGHT, RS_FPS = 848, 480, 30
RS_HFOV = 1.204  # 69 deg
RS_NEAR, RS_FAR = 0.3, 10.0

# CSI IMX219
CSI_WIDTH, CSI_HEIGHT, CSI_FPS = 1280, 720, 30
CSI_HFOV = 1.0856  # 62.2 deg

# Mecanum geometry (mega_drive firmware)
WHEEL_RADIUS = 0.0485
WHEEL_SEP = 0.21  # L = W
WHEEL_BASE = 0.21
OMEGA_WHEEL_MAX = 30.0  # rad/s (firmware)

WHEEL_LINKS = (
    "FL_wheel_link",
    "FR_wheel_link",
    "RL_wheel_link",
    "RR_wheel_link",
)
SENSOR_FIXED_JOINTS = (
    "realsense_joint",
    "CSI_joint",
    "realsense_camera_joint",
    "csi_camera_joint",
    "realsense_color_optical_joint",
    "realsense_depth_optical_joint",
    "csi_optical_joint",
)


def _indent(elem: ET.Element, level: int = 0) -> None:
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        for child in elem:
            _indent(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = i + "  "
        if not elem[-1].tail or not elem[-1].tail.strip():
            elem[-1].tail = i
    if level and (not elem.tail or not elem.tail.strip()):
        elem.tail = i


def _find_link(root: ET.Element, name: str) -> ET.Element | None:
    for link in root.findall("link"):
        if link.get("name") == name:
            return link
    return None


def _find_joint(root: ET.Element, name: str) -> ET.Element | None:
    for joint in root.findall("joint"):
        if joint.get("name") == name:
            return joint
    return None


def _set_inertial(
    link: ET.Element,
    mass: float,
    ixx: float = 1e-6,
    iyy: float = 1e-6,
    izz: float = 1e-6,
) -> None:
    inertial = link.find("inertial")
    if inertial is None:
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(inertial, "mass", value=str(mass))
        ET.SubElement(
            inertial,
            "inertia",
            ixx=str(ixx),
            ixy="0",
            ixz="0",
            iyy=str(iyy),
            iyz="0",
            izz=str(izz),
        )
        return
    mass_el = inertial.find("mass")
    if mass_el is None:
        mass_el = ET.SubElement(inertial, "mass")
    mass_el.set("value", str(mass))
    inertia = inertial.find("inertia")
    if inertia is None:
        inertia = ET.SubElement(inertial, "inertia")
    for k, v in (
        ("ixx", ixx),
        ("ixy", 0),
        ("ixz", 0),
        ("iyy", iyy),
        ("iyz", 0),
        ("izz", izz),
    ):
        inertia.set(k, str(v))


def _ensure_link_joint(
    root: ET.Element,
    parent: str,
    child: str,
    joint_name: str,
    rpy: str,
) -> None:
    if _find_link(root, child) is None:
        link = ET.SubElement(root, "link", name=child)
        _set_inertial(link, mass=1e-4, ixx=1e-8, iyy=1e-8, izz=1e-8)
    j = _find_joint(root, joint_name)
    if j is None:
        j = ET.SubElement(root, "joint", name=joint_name, type="fixed")
        ET.SubElement(j, "origin", xyz="0 0 0", rpy=rpy)
        ET.SubElement(j, "parent", link=parent)
        ET.SubElement(j, "child", link=child)
    else:
        # Keep joint up to date when regenerating from export
        origin = j.find("origin")
        if origin is None:
            origin = ET.SubElement(j, "origin")
        origin.set("xyz", "0 0 0")
        origin.set("rpy", rpy)


def _ensure_optical_frame(
    root: ET.Element,
    parent_link: str,
    frame: str,
    joint_name: str,
    *,
    already_optical: bool,
) -> None:
    """Create ROS optical frame (Z forward) from a camera_link (X forward)."""
    rpy = "0 0 0" if already_optical else OPTICAL_RPY
    _ensure_link_joint(root, parent_link, frame, joint_name, rpy)


def _mesh_prefix(mesh_pkg: str) -> str:
    return f"package://{mesh_pkg}/meshes/"


def _rewrite_mesh_uris(root: ET.Element, mesh_pkg: str) -> None:
    prefix = _mesh_prefix(mesh_pkg)
    for mesh in root.iter("mesh"):
        fn = mesh.get("filename", "")
        if fn.startswith("package://urdf_export/meshes/"):
            mesh.set("filename", prefix + fn.split("/")[-1])
        elif fn.startswith(prefix):
            pass
        elif "/meshes/" in fn:
            mesh.set("filename", prefix + fn.split("/")[-1])


def _strip_gazebo(root: ET.Element) -> None:
    for g in list(root.findall("gazebo")):
        root.remove(g)


def _add_preserve_fixed(root: ET.Element) -> None:
    for jname in SENSOR_FIXED_JOINTS:
        if _find_joint(root, jname) is None:
            continue
        g = ET.SubElement(root, "gazebo", reference=jname)
        p = ET.SubElement(g, "preserveFixedJoint")
        p.text = "true"


def _add_wheel_friction(root: ET.Element, mu: float) -> None:
    for link in WHEEL_LINKS:
        g = ET.SubElement(root, "gazebo", reference=link)
        ET.SubElement(g, "mu1").text = str(mu)
        ET.SubElement(g, "mu2").text = str(mu)
        ET.SubElement(g, "kp").text = "1000000.0"
        ET.SubElement(g, "kd").text = "100.0"
        ET.SubElement(g, "minDepth").text = "0.001"
        ET.SubElement(g, "maxVel").text = "1.0"


def _strip_collisions(root: ET.Element, link_names: tuple[str, ...]) -> None:
    for name in link_names:
        link = _find_link(root, name)
        if link is None:
            continue
        for col in list(link.findall("collision")):
            link.remove(col)


def _set_link_friction(root: ET.Element, link_name: str, mu: float) -> None:
    g = ET.SubElement(root, "gazebo", reference=link_name)
    ET.SubElement(g, "mu1").text = str(mu)
    ET.SubElement(g, "mu2").text = str(mu)
    ET.SubElement(g, "kp").text = "100000.0"
    ET.SubElement(g, "kd").text = "10.0"
    ET.SubElement(g, "maxVel").text = "10.0"


def _enable_kinematic_drive(root: ET.Element) -> None:
    """VelocityControl + no wheel/floor fight: wheels visual-only, icy chassis.

    Floor μ=100 in AWS house otherwise pins mesh wheels against VC.
    VC overwrites twist each step → planar 'perfect' motion without rolling.
    """
    _strip_collisions(root, WHEEL_LINKS)
    # MPU mesh collision can snag thresholds; keep visual only.
    _strip_collisions(root, ("mpu_link",))
    _set_link_friction(root, "base_link", 0.0)


def _plugin_velocity_control() -> ET.Element:
    g = ET.Element("gazebo")
    p = ET.SubElement(
        g,
        "plugin",
        filename="gz-sim-velocity-control-system",
        name="gz::sim::systems::VelocityControl",
    )
    ET.SubElement(p, "topic").text = "/model/tank_sim/cmd_vel"
    return g


def _plugin_mecanum_drive() -> ET.Element:
    g = ET.Element("gazebo")
    p = ET.SubElement(
        g,
        "plugin",
        filename="gz-sim-mecanum-drive-system",
        name="gz::sim::systems::MecanumDrive",
    )
    ET.SubElement(p, "front_left_joint").text = "FL_joint"
    ET.SubElement(p, "front_right_joint").text = "FR_wheel_joint"
    ET.SubElement(p, "back_left_joint").text = "RL_joint"
    ET.SubElement(p, "back_right_joint").text = "RR_joint"
    ET.SubElement(p, "wheel_separation").text = str(WHEEL_SEP)
    ET.SubElement(p, "wheelbase").text = str(WHEEL_BASE)
    ET.SubElement(p, "wheel_radius").text = str(WHEEL_RADIUS)
    ET.SubElement(p, "topic").text = "/model/tank_sim/cmd_vel"
    ET.SubElement(p, "odom_topic").text = "/model/tank_sim/odometry"
    ET.SubElement(p, "frame_id").text = "odom_gt"
    ET.SubElement(p, "child_frame_id").text = "base_footprint"
    ET.SubElement(p, "odom_publish_frequency").text = "50"
    return g


def _plugin_odometry() -> ET.Element:
    g = ET.Element("gazebo")
    p = ET.SubElement(
        g,
        "plugin",
        filename="gz-sim-odometry-publisher-system",
        name="gz::sim::systems::OdometryPublisher",
    )
    ET.SubElement(p, "odom_frame").text = "odom_gt"
    ET.SubElement(p, "robot_base_frame").text = "base_footprint"
    ET.SubElement(p, "odom_publish_frequency").text = "50"
    ET.SubElement(p, "dimensions").text = "2"
    ET.SubElement(p, "odom_topic").text = "/model/tank_sim/odometry"
    # Do not publish TF — rgbd_odometry owns odom->base_footprint
    ET.SubElement(p, "tf_topic").text = "/model/tank_sim/tf_gt_unused"
    return g


def _plugin_joint_state() -> ET.Element:
    """Publish only movable joints.

    Fixed joints kept via preserveFixedJoint must NOT appear in /joint_states,
    otherwise robot_state_publisher fights tf_static → TF_OLD_DATA / flicker.
    """
    g = ET.Element("gazebo")
    p = ET.SubElement(
        g,
        "plugin",
        filename="gz-sim-joint-state-publisher-system",
        name="gz::sim::systems::JointStatePublisher",
    )
    for jname in (
        "FL_joint",
        "FR_wheel_joint",
        "RL_joint",
        "RR_joint",
        "pan_joint",
        "tilt_joint",
    ):
        ET.SubElement(p, "joint_name").text = jname
    return g


def _plugin_joint_trajectory() -> ET.Element:
    g = ET.Element("gazebo")
    p = ET.SubElement(
        g,
        "plugin",
        filename="gz-sim-joint-trajectory-controller-system",
        name="gz::sim::systems::JointTrajectoryController",
    )
    for jname, pgain in (("pan_joint", 20.0), ("tilt_joint", 20.0)):
        ET.SubElement(p, "joint_name").text = jname
        ET.SubElement(p, "position_p_gain").text = str(pgain)
        ET.SubElement(p, "position_i_gain").text = "0.1"
        ET.SubElement(p, "position_d_gain").text = "1.0"
        ET.SubElement(p, "position_i_min").text = "-1"
        ET.SubElement(p, "position_i_max").text = "1"
        ET.SubElement(p, "position_cmd_min").text = "-10"
        ET.SubElement(p, "position_cmd_max").text = "10"
    ET.SubElement(p, "topic").text = "/model/tank_sim/joint_trajectory"
    return g


def _sensor_rgbd(root: ET.Element) -> None:
    # Sensor on realsense_camera_link (X=fwd). Stamp optical for rtabmap/CameraInfo.
    # PointCloud XYZ stay in camera_link → rewrite_frame_id in launch.
    g = ET.SubElement(root, "gazebo", reference="realsense_camera_link")
    sensor = ET.SubElement(
        g, "sensor", name="realsense_d415", type="rgbd_camera"
    )
    ET.SubElement(sensor, "always_on").text = "1"
    ET.SubElement(sensor, "update_rate").text = str(RS_FPS)
    ET.SubElement(sensor, "visualize").text = "false"
    ET.SubElement(sensor, "topic").text = "camera"
    ET.SubElement(sensor, "gz_frame_id").text = "realsense_color_optical_frame"
    cam = ET.SubElement(sensor, "camera")
    ET.SubElement(cam, "horizontal_fov").text = str(RS_HFOV)
    img = ET.SubElement(cam, "image")
    ET.SubElement(img, "width").text = str(RS_WIDTH)
    ET.SubElement(img, "height").text = str(RS_HEIGHT)
    ET.SubElement(img, "format").text = "R8G8B8"
    clip = ET.SubElement(cam, "clip")
    ET.SubElement(clip, "near").text = str(RS_NEAR)
    ET.SubElement(clip, "far").text = str(RS_FAR)
    noise = ET.SubElement(cam, "noise")
    ET.SubElement(noise, "type").text = "gaussian"
    ET.SubElement(noise, "mean").text = "0.0"
    ET.SubElement(noise, "stddev").text = "0.007"


def _sensor_csi(root: ET.Element) -> None:
    g = ET.SubElement(root, "gazebo", reference="csi_camera_link")
    sensor = ET.SubElement(g, "sensor", name="csi_imx219", type="camera")
    ET.SubElement(sensor, "always_on").text = "1"
    ET.SubElement(sensor, "update_rate").text = str(CSI_FPS)
    ET.SubElement(sensor, "visualize").text = "false"
    ET.SubElement(sensor, "topic").text = "csi_cam/image"
    ET.SubElement(sensor, "gz_frame_id").text = "csi_optical_frame"
    cam = ET.SubElement(sensor, "camera")
    ET.SubElement(cam, "horizontal_fov").text = str(CSI_HFOV)
    img = ET.SubElement(cam, "image")
    ET.SubElement(img, "width").text = str(CSI_WIDTH)
    ET.SubElement(img, "height").text = str(CSI_HEIGHT)
    ET.SubElement(img, "format").text = "R8G8B8"
    clip = ET.SubElement(cam, "clip")
    ET.SubElement(clip, "near").text = "0.05"
    ET.SubElement(clip, "far").text = "30.0"


def _bump_wheel_joint_limits(root: ET.Element, omega_max: float) -> None:
    for jname in ("FL_joint", "FR_wheel_joint", "RL_joint", "RR_joint"):
        joint = _find_joint(root, jname)
        if joint is None:
            continue
        lim = joint.find("limit")
        if lim is None:
            lim = ET.SubElement(joint, "limit")
        lim.set("velocity", str(omega_max))
        # Keep existing effort if present; raise floor so mecanum mode can push.
        try:
            effort = float(lim.get("effort", "0.5"))
        except ValueError:
            effort = 0.5
        lim.set("effort", str(max(effort, 5.0)))


def _verify(root: ET.Element) -> list[str]:
    warnings: list[str] = []
    required_links = [
        "base_footprint",
        "base_link",
        "pan_link",
        "tilt_link",
        "CSI_link",
        "realsense_link",
        *WHEEL_LINKS,
    ]
    for name in required_links:
        if _find_link(root, name) is None:
            warnings.append(f"missing link: {name}")
    for jname in ("pan_joint", "tilt_joint", "FL_joint", "FR_wheel_joint", "RL_joint", "RR_joint"):
        if _find_joint(root, jname) is None:
            warnings.append(f"missing joint: {jname}")
    base = _find_link(root, "base_link")
    if base is not None:
        col = base.find("collision")
        if col is not None and col.find("geometry/box") is None:
            warnings.append("base_link collision is not a box (expected AABB)")
    return warnings


def process(
    input_path: Path,
    output_path: Path,
    drive: str,
    *,
    mesh_pkg: str = "tank_description",
    profile: str = "gz",
) -> None:
    tree = ET.parse(input_path)
    root = tree.getroot()
    root.set("name", "tank_sim" if profile == "gz" else "tank")

    _rewrite_mesh_uris(root, mesh_pkg)
    _strip_gazebo(root)
    _bump_wheel_joint_limits(root, OMEGA_WHEEL_MAX)

    # Root link inertia (gz requires mass on root)
    bf = _find_link(root, "base_footprint")
    if bf is None:
        raise SystemExit("base_footprint missing in input URDF")
    _set_inertial(bf, mass=0.01, ixx=1e-5, iyy=1e-5, izz=1e-5)

    # CSI_link had mass=0 from SolidWorks
    csi = _find_link(root, "CSI_link")
    if csi is not None:
        _set_inertial(csi, mass=0.001, ixx=1e-7, iyy=1e-7, izz=1e-7)

    # SW realsense_link / CSI_link ≈ optical (Z fwd). Gazebo cameras look along +X,
    # so insert *_camera_link with RPY that maps optical→camera_link, attach sensors
    # there, stamp that frame (matches PointCloud XYZ). Optical frames = REP-103 kids.
    cam_rpy = "0 -1.57079632679 1.57079632679"
    _ensure_link_joint(
        root, "realsense_link", "realsense_camera_link", "realsense_camera_joint", cam_rpy
    )
    _ensure_link_joint(root, "CSI_link", "csi_camera_link", "csi_camera_joint", cam_rpy)
    _ensure_optical_frame(
        root,
        "realsense_camera_link",
        "realsense_color_optical_frame",
        "realsense_color_optical_joint",
        already_optical=False,
    )
    _ensure_optical_frame(
        root,
        "realsense_camera_link",
        "realsense_depth_optical_frame",
        "realsense_depth_optical_joint",
        already_optical=False,
    )
    _ensure_optical_frame(
        root, "csi_camera_link", "csi_optical_frame", "csi_optical_joint", already_optical=False
    )

    if profile == "gz":
        _add_preserve_fixed(root)

        if drive == "mecanum":
            _add_wheel_friction(root, 1.0)
            root.append(_plugin_mecanum_drive())
        else:
            # Perfect holonomic: no wheel contacts (AWS floor μ=100 was locking the tank).
            _enable_kinematic_drive(root)
            root.append(_plugin_velocity_control())
            root.append(_plugin_odometry())

        root.append(_plugin_joint_state())
        root.append(_plugin_joint_trajectory())
        _sensor_rgbd(root)
        _sensor_csi(root)
    # profile == "viz": clean URDF only (meshes + frames, no gazebo plugins)

    for w in _verify(root):
        print(f"[urdf_postprocess] WARN: {w}", file=sys.stderr)

    _indent(root)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    # ElementTree writes without XML declaration sometimes; force one
    xml_body = ET.tostring(root, encoding="unicode")
    output_path.write_text(
        '<?xml version="1.0" encoding="utf-8"?>\n' + xml_body + "\n",
        encoding="utf-8",
    )
    print(
        f"[urdf_postprocess] wrote {output_path} "
        f"(profile={profile}, drive={drive}, mesh_pkg={mesh_pkg})"
    )


def maybe_decimate(raw_dir: Path, out_dir: Path) -> None:
    """Optional mesh decimation. No-op if raw/ missing or open3d unavailable."""
    if not raw_dir.is_dir():
        print("[urdf_postprocess] --decimate: no raw/ dir, skipping")
        return
    try:
        import open3d as o3d  # type: ignore
    except ImportError:
        print("[urdf_postprocess] --decimate: open3d not installed, skipping")
        return
    out_dir.mkdir(parents=True, exist_ok=True)
    for src in sorted(raw_dir.glob("*.STL")) + sorted(raw_dir.glob("*.stl")):
        dst = out_dir / src.name
        if dst.exists() and dst.stat().st_mtime >= src.stat().st_mtime:
            continue
        mesh = o3d.io.read_triangle_mesh(str(src))
        target = max(2000, int(len(mesh.triangles) * 0.05))
        simplified = mesh.simplify_quadric_decimation(target)
        o3d.io.write_triangle_mesh(str(dst), simplified)
        print(f"[urdf_postprocess] decimated {src.name} -> {target} tris")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--input", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument(
        "--drive",
        choices=("velocity", "mecanum"),
        default="velocity",
        help="velocity=holonomic VelocityControl (default); mecanum=experimental",
    )
    ap.add_argument(
        "--mesh-pkg",
        default="tank_description",
        help="ROS package name for mesh URIs (default: tank_description)",
    )
    ap.add_argument(
        "--profile",
        choices=("gz", "viz"),
        default="gz",
        help="gz=Gazebo plugins+sensors (default); viz=clean URDF for RViz/RSP",
    )
    ap.add_argument(
        "--decimate",
        action="store_true",
        help="Decimate meshes from urdf_export/meshes/raw/ if present",
    )
    ap.add_argument(
        "--raw-dir",
        type=Path,
        default=None,
        help="Override raw mesh directory for --decimate",
    )
    args = ap.parse_args()

    if args.decimate:
        raw = args.raw_dir or (args.input.parent.parent / "meshes" / "raw")
        out_meshes = args.output.parent.parent / "meshes"
        maybe_decimate(raw, out_meshes)

    if not args.input.is_file():
        raise SystemExit(f"input URDF not found: {args.input}")
    process(
        args.input,
        args.output,
        args.drive,
        mesh_pkg=args.mesh_pkg,
        profile=args.profile,
    )


if __name__ == "__main__":
    main()
