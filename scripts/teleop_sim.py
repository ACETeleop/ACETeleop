from isaacgym import gymapi, gymtorch
import torch
import math
import numpy as np
from pathlib import Path
import argparse
import time
import yaml
from copy import deepcopy
from typing import Dict, Any

from avp_stream.utils.isaac_utils import *
from avp_stream.utils.se3_utils import *
from avp_stream.utils.trn_constants import *

import ace_teleop
from ace_teleop.control.teleop import ACETeleop

def load_config(config_file_name: str) -> Dict[str, Any]:
    robot_config_path = (
        f"{ace_teleop.__path__[0]}/configs/robot" / Path(config_file_name)
    )
    with Path(robot_config_path).open("r") as f:
        cfg = yaml.safe_load(f)["robot_cfg"]

    return cfg


def np2tensor(
    data: Dict[str, np.ndarray], device: torch.device
) -> Dict[str, torch.Tensor]:
    return {
        key: torch.tensor(value, dtype=torch.float32, device=device)
        for key, value in data.items()
    }


class Sim:
    def __init__(
        self,
        cfg: Dict[str, Any],
        collision: bool = False,
        print_freq: bool = False,
        debug: bool = False,
    ) -> None:
        self.print_freq = print_freq
        self.debug = debug
        self.num_envs = 1

        # initialize gym
        self.gym = gymapi.acquire_gym()

        # configure sim
        self.device = "cpu"
        self.sim_params = default_sim_params(use_gpu=(self.device == "cuda:0"))

        # create sim
        self.sim = self.gym.create_sim(0, 0, gymapi.SIM_PHYSX, self.sim_params)
        if self.sim is None:
            raise Exception("Failed to create sim")

        # add ground plane
        plane_params = gymapi.PlaneParams()
        plane_params.distance = 0.0
        plane_params.normal = gymapi.Vec3(0.0, 0.0, 1.0)
        self.gym.add_ground(self.sim, plane_params)

        # create environment
        num_envs = 1
        num_per_row = int(math.sqrt(num_envs))
        env_spacing = 1.25
        env_lower = gymapi.Vec3(-env_spacing, 0.0, -env_spacing)
        env_upper = gymapi.Vec3(env_spacing, env_spacing, env_spacing)

        np.random.seed(17)

        self.env = self.gym.create_env(self.sim, env_lower, env_upper, num_per_row)

        # add axis asset
        if self.debug:
            axis_root = Path(ace_teleop.__path__[0]) / "assets" / "axis"
            self.axis = self.load_axis("normal", axis_root)
            self.small_axis = self.load_axis("small", axis_root)
            self.huge_axis = self.load_axis("huge", axis_root)

        # add robot asset
        robot_asset_root = Path(ace_teleop.__path__[0]) / "assets"
        robot_asset_file = cfg["urdf_path"]
        asset_options = self.get_asset_options()
        self.sphere = self.gym.create_sphere(self.sim, 0.008, asset_options)

        # load robot
        robot_asset = self.gym.load_asset(
            self.sim, str(robot_asset_root), robot_asset_file, asset_options
        )
        self.dof = self.gym.get_asset_dof_count(robot_asset)
        self.robot_dof_props = self.gym.get_asset_dof_properties(robot_asset)
        self.set_dof_properties()

        if collision:
            self.set_collision_properties(robot_asset)

        if self.debug:
            self.add_actors()

        # set robot pose
        self.set_robot_pose(robot_asset)

        # create default viewer
        self.create_viewer()
        self.gym.prepare_sim(self.sim)
        if self.debug:
            self.initialize_tensors()

    def load_axis(self, size: str, root: Path) -> Any:
        return load_axis(self.gym, self.sim, self.device, size, str(root))

    def get_asset_options(self) -> gymapi.AssetOptions:
        asset_options = gymapi.AssetOptions()
        asset_options.fix_base_link = True
        asset_options.default_dof_drive_mode = gymapi.DOF_MODE_POS

        return asset_options

    def set_dof_properties(self) -> None:
        for i in range(self.dof):
            self.robot_dof_props["stiffness"][i] = 1000.0
            self.robot_dof_props["damping"][i] = 1000.0

    def set_collision_properties(self, robot_asset: Any) -> None:
        shape_properties = gymapi.RigidShapeProperties()
        shape_properties.friction = 20.0
        self.gym.set_asset_rigid_shape_properties(robot_asset, [shape_properties])

    def add_actors(self) -> None:

        self.head_axis = self.gym.create_actor(
            self.env, self.axis, gymapi.Transform(), "head", 0
        )
        self.right_wrist_axis = self.gym.create_actor(
            self.env, self.axis, gymapi.Transform(), "right_wrist", 1
        )
        self.left_wrist_axis = self.gym.create_actor(
            self.env, self.axis, gymapi.Transform(), "left_wrist", 2
        )

        self.add_spheres()
        self.add_small_axes()
        self.env_axis = self.gym.create_actor(
            self.env, self.huge_axis, gymapi.Transform(), "env_axis", 103
        )

    def add_spheres(self) -> None:
        for i in range(25):
            finger_1 = self.gym.create_actor(
                self.env, self.sphere, gymapi.Transform(), f"right_finger_{i}", 3 + i
            )
            self.set_sphere_color(finger_1, i)

        for i in range(25):
            finger_2 = self.gym.create_actor(
                self.env, self.sphere, gymapi.Transform(), f"left_finger_{i}", 28 + i
            )
            self.set_sphere_color(finger_2, i)

    def set_sphere_color(self, actor: Any, index: int) -> None:
        color = (
            gymapi.Vec3(1, 1, 0)
            if index in [0, 4, 9, 14, 19, 24]
            else gymapi.Vec3(1, 1, 1)
        )
        self.gym.set_rigid_body_color(
            self.env, actor, 0, gymapi.MESH_VISUAL_AND_COLLISION, color
        )

    def add_small_axes(self) -> None:
        for i in range(25):
            self.gym.create_actor(
                self.env,
                self.small_axis,
                gymapi.Transform(),
                f"right_finger_{i}",
                53 + i,
            )

        for i in range(25):
            self.gym.create_actor(
                self.env,
                self.small_axis,
                gymapi.Transform(),
                f"left_finger_{i}",
                78 + i,
            )

    def set_robot_pose(self, robot_asset: Any) -> None:
        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(0, 0, 1.1)
        pose.r = gymapi.Quat(0, 0, 0, 1)

        self.robot_handle = self.gym.create_actor(
            self.env, robot_asset, pose, "robot", 1, 1
        )
        self.gym.set_actor_dof_properties(
            self.env, self.robot_handle, self.robot_dof_props
        )
        self.gym.set_actor_dof_states(
            self.env,
            self.robot_handle,
            np.zeros(self.dof, gymapi.DofState.dtype),
            gymapi.STATE_ALL,
        )
        self.gym.set_actor_dof_position_targets(
            self.env, self.robot_handle, np.zeros(self.dof, dtype=np.float32)
        )

    def create_viewer(self) -> None:
        self.viewer = self.gym.create_viewer(self.sim, gymapi.CameraProperties())
        if self.viewer is None:
            raise Exception("Failed to create viewer")

        cam_pos = gymapi.Vec3(1, 1, 2)
        cam_target = gymapi.Vec3(0, 0, 1)
        self.gym.viewer_camera_look_at(self.viewer, None, cam_pos, cam_target)

    def step(
        self, cmd: np.ndarray, transformation: Dict[str, torch.Tensor] = None
    ) -> None:
        if self.print_freq:
            start = time.time()

        # Set robot DOF
        states = np.zeros(cmd.shape, dtype=gymapi.DofState.dtype)
        states["pos"] = cmd
        self.gym.set_actor_dof_states(
            self.env, self.robot_handle, states, gymapi.STATE_POS
        )

        # Step the physics
        self.gym.simulate(self.sim)

        if self.debug:
            refresh_tensors(self.gym, self.sim)

        self.gym.fetch_results(self.sim, True)
        self.gym.render_all_camera_sensors(self.sim)

        if self.debug:
            new_root_state = self.modify_root_state(transformation)
            env_side_actor_idxs = torch.arange(0, 103, dtype=torch.int32)
            self.gym.set_actor_root_state_tensor_indexed(
                self.sim,
                gymtorch.unwrap_tensor(new_root_state),
                gymtorch.unwrap_tensor(env_side_actor_idxs),
                len(env_side_actor_idxs),
            )

        self.gym.step_graphics(self.sim)
        self.gym.draw_viewer(self.viewer, self.sim, True)
        self.gym.sync_frame_time(self.sim)

        if self.print_freq:
            end = time.time()
            print("Frequency:", 1 / (end - start))

    def modify_root_state(
        self, transformations: Dict[str, torch.Tensor]
    ) -> torch.Tensor:
        self.visionos_head = transformations["head"]
        self.visionos_head[0, 2, 3] += 0.25

        self.sim_right_wrist = transformations["right_wrist"]
        self.sim_right_wrist[0, 2, 3] += 1.1

        self.sim_left_wrist = transformations["left_wrist"]
        self.sim_left_wrist[0, 2, 3] += 1.1

        sim_right_fingers = torch.cat(
            [
                self.sim_right_wrist @ finger
                for finger in transformations["right_fingers"]
            ],
            dim=0,
        )
        sim_left_fingers = torch.cat(
            [
                self.sim_left_wrist @ finger
                for finger in transformations["left_fingers"]
            ],
            dim=0,
        )

        new_root_state = deepcopy(self.root_state)
        new_root_state[:, 0, :7] = mat2posquat(self.visionos_head)
        new_root_state[:, 1, :7] = mat2posquat(self.sim_right_wrist)
        new_root_state[:, 2, :7] = mat2posquat(self.sim_left_wrist)
        new_root_state[:, 3:28, :7] = mat2posquat(sim_right_fingers)
        new_root_state[:, 28:53, :7] = mat2posquat(sim_left_fingers)
        new_root_state[:, 53:78, :7] = mat2posquat(sim_right_fingers)
        new_root_state[:, 78:103, :7] = mat2posquat(sim_left_fingers)
        new_root_state = new_root_state.view(-1, 13)

        return new_root_state

    def initialize_tensors(self) -> None:
        refresh_tensors(self.gym, self.sim)

        # get rigid body state tensor
        _rb_states = self.gym.acquire_rigid_body_state_tensor(self.sim)
        self.rb_states = gymtorch.wrap_tensor(_rb_states).view(self.num_envs, -1, 13)

        # get actor root state tensor
        _root_states = self.gym.acquire_actor_root_state_tensor(self.sim)
        root_states = gymtorch.wrap_tensor(_root_states).view(self.num_envs, -1, 13)
        self.root_state = root_states

        self.gym.simulate(self.sim)
        self.gym.fetch_results(self.sim, True)
        self.gym.step_graphics(self.sim)
        self.gym.draw_viewer(self.viewer, self.sim, False)
        self.gym.sync_frame_time(self.sim)

    def end(self) -> None:
        self.gym.destroy_viewer(self.viewer)
        self.gym.destroy_sim(self.sim)


def main() -> None:
    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        "-c",
        choices=["h1_inspire", "xarm_ability", "gr1", "franka"],
        default="h1_inspire",
    )
    parser.add_argument("--ip", default="localhost")
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    config_file_name = f"{args.config}.yml"
    cfg = load_config(config_file_name)

    teleoperator = ACETeleop(cfg, args.ip, debug=args.debug)
    simulator = Sim(cfg, print_freq=False, debug=args.debug)

    try:
        while True:
            if args.debug:
                cmd, latest = teleoperator.step()
                simulator.step(cmd, np2tensor(latest, simulator.device))
            else:
                cmd = teleoperator.step()
                simulator.step(cmd)
    except KeyboardInterrupt:
        simulator.end()
        exit(0)


if __name__ == "__main__":
    main()
