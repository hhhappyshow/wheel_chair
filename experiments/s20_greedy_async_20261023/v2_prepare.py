"""Build the current S20 native cuRoboV2 configuration, without author edits."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from compare import ROOT, SCENE, UPSTREAM
sys.path.insert(0, str(ROOT/"reproductions/nvidia_curobo_v2_20261023"))
from roboticarm_newton import make_robot_urdf, scene_paths


def prepare(output):
    from curobo.robot_builder import RobotBuilder

    output.mkdir(parents=True, exist_ok=True)
    source, _, models = scene_paths(UPSTREAM/"policy/vendor/roboticarm")
    root = ET.fromstring(make_robot_urdf(source, models))
    payload = root.find("link[@name='payload_box']")
    for node in list(payload):
        payload.remove(node)
    urdf = output/"s20_current.urdf"
    urdf.write_text(ET.tostring(root, encoding="unicode"))
    builder = RobotBuilder(urdf_path=str(urdf.resolve()), asset_path=str(source.resolve()),
                           tool_frames=["suction_contact_frame"])
    builder.fit_collision_spheres(use_collision_mesh=True, iterations=200,
                                 clip_links={"elfin_base": ("z", 0.)})
    builder.compute_collision_matrix(prune_collisions=False)
    config = builder.build()
    builder.save(config, str(output/"robot_fitted.yml"))
    robot = yaml.safe_load((output/"robot_fitted.yml").read_text())
    kin = robot["kinematics"]
    kin["extra_collision_spheres"] = {"payload_box": 256}
    kin["collision_link_names"].append("payload_box")
    kin["self_collision_ignore"]["payload_box"] = ["suction_panel", "suction_contact_frame"]
    # Keep vendor position/velocity limits; these are simulation generator
    # settings, not measured S20 acceleration/jerk or torque capabilities.
    kin["cspace"]["max_acceleration"] = 3.0
    kin["cspace"]["max_jerk"] = 60.0
    robot["load_dynamics"] = False
    (output/"robot.yml").write_text(yaml.safe_dump({"robot_cfg": robot}, sort_keys=False))
    parent = {j.find("child").get("link"): j for j in root.findall("joint")}
    link, chain, radius = "suction_contact_frame", [], 0.
    while link in parent:
        joint = parent[link]
        xyz = np.fromstring(joint.find("origin").get("xyz", "0 0 0"), sep=" ")
        length = float(np.linalg.norm(xyz))
        radius += length
        chain.append(dict(joint=joint.get("name"), translation_m=xyz.tolist(), length_m=length))
        link = joint.find("parent").get("link")
    (output/"reach_bound.json").write_text(json.dumps(dict(
        radius_m=radius, center_link=link, target="suction_contact_frame origin",
        method="Triangle-inequality outer sphere from complete URDF base-to-TCP joint-origin translations",
        interpretation="Conservative outer bound only; does not certify downward orientation, collision or IK",
        chain=chain, maximum_placed_box_top_above_floor_m=1.0,
        scene=str(SCENE), dynamics="Disabled for this kinematic planning experiment"), indent=2))
    print(json.dumps(dict(robot=str(output/"robot.yml"), radius_m=radius)), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    prepare(parser.parse_args().output)
