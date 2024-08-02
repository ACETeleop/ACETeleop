import numpy as np
from avp_stream import VisionProStreamer as AVPStreamer
from typing import Tuple, List, Dict, Any

from .controller import ACEController

def decode_msg(
    message: Dict[str, np.ndarray]
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    right_wrist_pose = message["right_wrist"][0]
    left_wrist_pose = message["left_wrist"][0]

    right_fingers = message["right_fingers"][:, 0:3, 3]
    left_fingers = message["left_fingers"][:, 0:3, 3]

    return left_wrist_pose, right_wrist_pose, left_fingers, right_fingers

class ACETeleop:
    def __init__(self, cfg: Dict[str, Any], ip: str, debug: bool = False):
        self.streamer = AVPStreamer(ip=ip)
        self.controller = ACEController(cfg)

        self.debug = debug

        self.left_arm_indices: List[int] = cfg["left_arm_indices"]
        self.right_arm_indices: List[int] = cfg["right_arm_indices"]

    def step(self) -> Tuple[List[float], Any]:
        data = decode_msg(self.streamer.latest)

        self.controller.update(*data)

        cmd = self.controller.qpos

        if self.debug:
            return cmd, self.streamer.latest.copy()
        else:
            return cmd
