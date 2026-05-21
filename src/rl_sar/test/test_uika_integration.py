#!/usr/bin/env python3

from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import yaml


ROOT = Path(__file__).resolve().parents[3]
HIMLOCO_RUN = Path("/home/esd/project/himloco_lab/logs/himloco_rsl_rl/uika/2026-05-23_15-02-49")


def test_uika_policy_and_robot_assets_are_registered():
    assert (ROOT / "policy/uika/base.yaml").exists()
    assert (ROOT / "policy/uika/himloco/config.yaml").exists()
    assert (ROOT / "policy/uika/himloco/policy.pt").exists()
    assert (ROOT / "src/rl_sar_zoo/uika_description/mjcf/scene.xml").exists()

    fsm_all = (ROOT / "src/rl_sar/fsm_robot/fsm_all.hpp").read_text()
    assert '#include "fsm_uika.hpp"' in fsm_all


def test_uika_policy_matches_himloco_export_layout():
    config = (ROOT / "policy/uika/himloco/config.yaml").read_text()
    assert 'model_name: "policy.pt"' in config
    assert "num_observations: 45" in config
    assert 'observations: ["commands", "ang_vel", "gravity_vec", "dof_pos", "dof_vel", "actions"]' in config
    assert "observations_history: [0, 1, 2, 3, 4, 5]" in config
    assert 'observations_history_priority: "time"' in config
    assert "num_of_dofs: 12" in config
    assert "wheel_indices: []" in config
    assert "joint_mapping: [0, 3, 6, 9, 1, 4, 7, 10, 2, 5, 8, 11]" in config
    assert "default_dof_pos: [-0.80, 0.80, -0.75, 0.75," in config


def test_uika_deploy_config_matches_training_export():
    exported = yaml.load((HIMLOCO_RUN / "params/deploy.yaml").read_text(), Loader=yaml.FullLoader)
    config = yaml.safe_load((ROOT / "policy/uika/himloco/config.yaml").read_text())["uika/himloco"]
    base = yaml.safe_load((ROOT / "policy/uika/base.yaml").read_text())["uika"]

    assert config["observations"] == [
        "commands",
        "ang_vel",
        "gravity_vec",
        "dof_pos",
        "dof_vel",
        "actions",
    ]
    assert config["observations_history"] == list(range(exported["history_length"]))
    assert config["observations_history_priority"] == "time"
    assert config["joint_mapping"] == exported["joint_ids_map"]
    assert base["joint_mapping"] == exported["joint_ids_map"]
    expected_default_dof_pos = [-0.80, 0.80, -0.75, 0.75, 0.05, 0.05, 0.05, 0.05, 0.70, 0.70, 0.70, 0.70]
    assert config["default_dof_pos"] == expected_default_dof_pos
    assert base["default_dof_pos"] == expected_default_dof_pos
    assert config["action_scale"] == exported["actions"]["JointPositionAction"]["scale"]
    action_clip = exported["actions"]["JointPositionAction"]["clip"]
    assert config["clip_actions_lower"] == [bounds[0] for bounds in action_clip]
    assert config["clip_actions_upper"] == [bounds[1] for bounds in action_clip]
    assert config["commands_scale"] == exported["observations"]["velocity_commands"]["scale"]
    command_ranges = exported["commands"]["base_velocity"]["ranges"]
    assert config["commands_limit_lower"] == [
        command_ranges["lin_vel_x"][0],
        command_ranges["lin_vel_y"][0],
        command_ranges["ang_vel_z"][0],
    ]
    assert config["commands_limit_upper"] == [
        command_ranges["lin_vel_x"][1],
        command_ranges["lin_vel_y"][1],
        command_ranges["ang_vel_z"][1],
    ]
    assert config["ang_vel_scale"] == exported["observations"]["base_ang_vel"]["scale"][0]
    assert config["dof_pos_scale"] == exported["observations"]["joint_pos_rel"]["scale"][0]
    assert config["dof_vel_scale"] == exported["observations"]["joint_vel_rel"]["scale"][0]


def test_uika_effort_limits_match_urdf_not_lab_asset_override():
    config = yaml.safe_load((ROOT / "policy/uika/himloco/config.yaml").read_text())["uika/himloco"]
    base = yaml.safe_load((ROOT / "policy/uika/base.yaml").read_text())["uika"]
    joint_mapping = config["joint_mapping"]

    urdf_tree = ET.parse(ROOT / "src/rl_sar_zoo/uika_description/urdf/uika_description.urdf")
    urdf_efforts_by_joint = {
        joint.attrib["name"]: float(joint.find("limit").attrib["effort"])
        for joint in urdf_tree.findall(".//joint")
        if joint.find("limit") is not None
    }
    efforts_sdk_order = [urdf_efforts_by_joint[name] for name in base["joint_names"]]
    efforts_training_order = [efforts_sdk_order[i] for i in joint_mapping]

    assert config["torque_limits"] == efforts_training_order
    assert base["torque_limits"] == efforts_training_order

    mjcf_tree = ET.parse(ROOT / "src/rl_sar_zoo/uika_description/mjcf/uika.xml")
    for motor in mjcf_tree.findall(".//actuator/motor"):
        effort = urdf_efforts_by_joint[motor.attrib["joint"]]
        assert motor.attrib["ctrlrange"] == f"-{effort:g} {effort:g}"


def test_uika_default_angles_map_to_lab_init_state_sdk_order():
    config = yaml.safe_load((ROOT / "policy/uika/himloco/config.yaml").read_text())["uika/himloco"]
    mapping = config["joint_mapping"]

    default_sdk_order = [None] * len(mapping)
    for training_index, sdk_index in enumerate(mapping):
        default_sdk_order[sdk_index] = config["default_dof_pos"][training_index]

    assert default_sdk_order == [
        -0.80, 0.05, 0.70,
         0.80, 0.05, 0.70,
        -0.75, 0.05, 0.70,
         0.75, 0.05, 0.70,
    ]


def test_observation_history_order_matches_training_wrapper(tmp_path):
    source = tmp_path / "check_history.cpp"
    binary = tmp_path / "check_history"
    source.write_text(
        r'''
#include "observation_buffer.hpp"
#include <vector>
#include <stdexcept>

int main() {
    ObservationBuffer buffer(1, {45}, 6, "time");
    for (int frame = 1; frame <= 6; ++frame) {
        std::vector<float> obs(45, static_cast<float>(frame));
        buffer.insert(obs);
    }

    auto history = buffer.get_obs_vec({0, 1, 2, 3, 4, 5});
    if (history.size() != 270) return 1;
    for (int frame = 0; frame < 6; ++frame) {
        float expected = static_cast<float>(6 - frame);
        for (int i = 0; i < 45; ++i) {
            if (history[frame * 45 + i] != expected) return 2;
        }
    }
    return 0;
}
'''
    )
    subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-I",
            str(ROOT / "src/rl_sar/library/core/observation_buffer"),
            str(source),
            str(ROOT / "src/rl_sar/library/core/observation_buffer/observation_buffer.cpp"),
            "-o",
            str(binary),
        ],
        check=True,
    )
    subprocess.run([str(binary)], check=True)


def test_uika_mujoco_leg_meshes_have_collision_enabled():
    tree = ET.parse(ROOT / "src/rl_sar_zoo/uika_description/mjcf/uika.xml")
    colliding_meshes = {
        geom.attrib["mesh"]
        for geom in tree.findall(".//geom")
        if geom.attrib.get("type") == "mesh"
        and geom.attrib.get("contype", "1") != "0"
        and geom.attrib.get("conaffinity", "1") != "0"
    }

    for leg in ("FL", "FR", "RL", "RR"):
        for segment in ("hip", "thigh", "calf", "foot"):
            assert f"{leg}_{segment}" in colliding_meshes
