import numpy as np

from dataclasses import dataclass
from typing import Sequence, Tuple
from ace_teleop.dynamixel.robots.dynamixel_robot import DynamixelRobot


@dataclass
class DynamixelRobotConfig:
    joint_ids: Sequence[int]
    """The joint ids of dynamixel robot. Usually (1, 2, 3 ...)."""

    joint_offsets: Sequence[float]
    """The joint offsets of robot. There needs to be a joint offset for each joint_id and should be a multiple of pi/2."""

    joint_signs: Sequence[int]
    """The joint signs is -1 for all dynamixel"""

    gripper_config: Tuple[int, int, int]
    """reserved"""

    is_ACE: bool = True
    """Dynamxiel robot is ACE or not. Default is True."""

    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_offsets)
        assert len(self.joint_ids) == len(self.joint_signs)

    def make_robot(
        self,
        port: str = "/dev/ttyUSB0",
        urdf: str = "",
        ee_link_name: str = "",
    ) -> DynamixelRobot:
        return DynamixelRobot(
            port=port,
            urdf=urdf,
            ee_link_name=ee_link_name,
            joint_ids=self.joint_ids,
            joint_offsets=list(self.joint_offsets),
            joint_signs=list(self.joint_signs),
            gripper_config=self.gripper_config,
            is_ACE=self.is_ACE,
        )


PORT_CONFIG_MAP = {
    #! right agent fah
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISG7I-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            2 * np.pi / 2,
            -2 * np.pi / 2,
            2 * np.pi / 2,
            3 * np.pi / 2,
            4 * np.pi / 2,
            2 * np.pi / 2,
        ),
        joint_signs=(-1, -1, -1, -1, -1, -1),
        gripper_config=None,
    ),
    #! left agent fah
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0QI3-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            1 * np.pi / 2,
            4 * np.pi / 2,
            1 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
            2 * np.pi / 2,
        ),
        joint_signs=(-1, -1, -1, -1, -1, -1),
        gripper_config=None,
    ),
    #! right agent jacob
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT89FKBF-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
        4*np.pi/2, 
        3*np.pi/2, 
        3*np.pi/2, 
        2*np.pi/2, 
        3*np.pi/2, 
        4*np.pi/2
        ),
        joint_signs=(-1, -1, -1, -1, -1, -1),
        gripper_config=None,
    ),
    #! left agent jacob
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8ISW9S-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            3*np.pi/2, 
            4*np.pi/2, 
            2*np.pi/2, 
            3*np.pi/2, 
            0*np.pi/2, 
            4*np.pi/2
        ),
        joint_signs=(-1, -1, -1, -1, -1, -1),
        gripper_config=None,
    ),
}
