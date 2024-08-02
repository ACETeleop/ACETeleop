import numpy as np
import pinocchio as pin

from pathlib import Path
from typing import Dict, Optional, Sequence, Tuple
from ace_teleop.dynamixel.robots.robot import Robot
from ace_teleop.dynamixel.driver import DynamixelDriver


class DynamixelRobot(Robot):
    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        urdf: str = "",
        ee_link_name: str = "",
        joint_ids: Sequence[int] = None,
        joint_offsets: Optional[Sequence[float]] = None,
        joint_signs: Optional[Sequence[int]] = None,
        gripper_config: Optional[Tuple[int, float, float]] = None,
        is_ACE: bool = True,
        baudrate: int = 57600,
    ):
        urdf_path = Path(__file__).resolve().parent.parent / "urdf" / Path(urdf)
        print(f"attempting to connect to port: {port}")

        self.gripper_open_close: Optional[Tuple[float, float]]
        self.is_ACE = is_ACE

        if gripper_config is not None:
            assert joint_offsets is not None
            assert joint_signs is not None

            joint_ids = tuple(joint_ids) + (gripper_config[0],)
            joint_offsets = tuple(joint_offsets) + (0.0,)
            joint_signs = tuple(joint_signs) + (1,)
            self.gripper_open_close = (
                gripper_config[1] * np.pi / 180,
                gripper_config[2] * np.pi / 180,
            )
        else:
            self.gripper_open_close = None

        # set joint config
        self._joint_ids = joint_ids

        self._joint_offsets = np.array(joint_offsets)
        self._joint_signs = np.array(joint_signs)

        # check
        assert len(self._joint_ids) == len(self._joint_offsets), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_offsets: {len(self._joint_offsets)}"
        )
        assert len(self._joint_ids) == len(self._joint_signs), (
            f"joint_ids: {len(self._joint_ids)}, "
            f"joint_signs: {len(self._joint_signs)}"
        )
        assert np.all(
            np.abs(self._joint_signs) == 1
        ), f"joint_signs: {self._joint_signs}"

        self._driver = DynamixelDriver(joint_ids, port=port, baudrate=baudrate)
        self._driver.set_torque_mode(False)

        self._torque_on = False
        self._last_pos = None
        self._alpha = 0.99

        # Init kinematics model of the robot
        robot_urdf_path = urdf_path
        # robot_urdf_path = urdf
        print(f"Loading URDF from {robot_urdf_path}")
        self.model: pin.Model = pin.buildModelFromUrdf(str(robot_urdf_path))
        self.data: pin.Data = self.model.createData()

        frame_mapping: Dict[str, int] = {}
        for i, frame in enumerate(self.model.frames):
            frame_mapping[frame.name] = i
        self.ee_frame_id = frame_mapping[ee_link_name]

    def compute_ee_pose(self, joint_pos: np.ndarray):

        pin.forwardKinematics(self.model, self.data, joint_pos)
        oMf: pin.SE3 = pin.updateFramePlacement(self.model, self.data, self.ee_frame_id)

        return oMf.homogeneous

    @property
    def num_dofs(self) -> int:
        return len(self._joint_ids)

    def get_joint_state(self) -> np.ndarray:
        pos = (self._driver.get_joints() - self._joint_offsets) * self._joint_signs
        assert len(pos) == self.num_dofs

        if self.gripper_open_close is not None:
            # map pos to [0, 1]
            g_pos = (pos[-1] - self.gripper_open_close[0]) / (
                self.gripper_open_close[1] - self.gripper_open_close[0]
            )
            g_pos = min(max(0, g_pos), 1)
            pos[-1] = g_pos

        if self._last_pos is None:
            self._last_pos = pos
        else:
            # exponential smoothing
            pos = self._last_pos * (1 - self._alpha) + pos * self._alpha
            self._last_pos = pos

        if self.is_ACE:
            new_pos = np.append(pos, 0)
            return new_pos
        return pos

    def map_to_valid_range(self, radians_array):
        mapped_radians = np.mod(radians_array, 2 * np.pi)
        return mapped_radians

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        print("command                   : ", [f"{x:.3f}" for x in joint_state])
        set_value = (joint_state + self._joint_offsets).tolist()
        print("_set value                 : ", [f"{x:.3f}" for x in set_value])
        set_value = self.map_to_valid_range(set_value)
        print("set value                 : ", [f"{x:.3f}" for x in set_value])
        self._driver.set_joints(set_value)

    def set_torque_mode(self, mode: bool):
        if mode == self._torque_on:
            return
        self._driver.set_torque_mode(mode)
        self._torque_on = mode

    def get_observations(self) -> Dict[str, np.ndarray]:
        return {"joint_state": self.get_joint_state()}
