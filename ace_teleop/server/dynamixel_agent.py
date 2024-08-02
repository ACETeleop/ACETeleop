import os

import numpy as np

from ace_teleop.server.agent import Agent
from ace_teleop.dynamixel.calibration.config import PORT_CONFIG_MAP


class DynamixelAgent(Agent):
    def __init__(
        self,
        port: str,
        urdf: str = "",
        ee_link_name: str = "",
    ) -> None:
        assert os.path.exists(port), port
        assert port in PORT_CONFIG_MAP, f"Port {port} not in config map"

        config = PORT_CONFIG_MAP[port]
        self._robot = config.make_robot(port=port, urdf=urdf, ee_link_name=ee_link_name)

    def get_ee(self) -> np.ndarray:
        return self._robot.compute_ee_pose(self._robot.get_joint_state())

    def get_joints(self) -> np.ndarray:
        return self._robot.get_joint_state()
