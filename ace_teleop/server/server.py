from concurrent import futures
import grpc
import numpy as np

from ace_teleop.configs.server.ace_const import *
from ace_teleop.stream.streamer import HandTrackingServicer, handtracking_pb2_grpc
from ace_teleop.server.utils import *

class Server:
    def __init__(self, cfg: dict) -> None:
        
        self.mode = cfg["mode"]

        agent_cfg = {}
        self.enable_agent = {}
        self.wrist = {}
        for name in ["left", "right"]:
            agent_cfg[name] = cfg.get(f"{name}_agent", None)

            self.enable_agent[name] = agent_cfg[name] is not None

            if not self.enable_agent[name]:
                self.initialized = True
                print("Single agent mode do not support auto initialization")

            self.wrist[name] = None

        if not self.enable_agent["left"] and not self.enable_agent["right"]:
            raise ValueError("No agent is enabled")

        self.cfg = cfg
        self.init_cfg()
        self.init_server()

    def init_cfg(self) -> None:

        self.is_ACE = self.cfg["is_ACE"]

        self.pos_scale = self.cfg["pos_scale"]
        self.roll_scale = self.cfg["roll_scale"]
        self.pitch_scale = self.cfg["pitch_scale"]
        self.yaw_scale = self.cfg["yaw_scale"]

        self.roll_limit = self.cfg["roll_limit"]
        self.pitch_limit = self.cfg["pitch_limit"]
        self.yaw_limit = self.cfg["yaw_limit"]

        self.roll_offset = self.cfg.get("roll_offset", 0)

        self.wrist_init_rot, self.wrist_init_pos = {}, {}
        self.center_l, self.radius_l = {}, {}
        for name in ["left", "right"]:
            if self.enable_agent[name]:
                wrist_cfg = self.cfg[f"{name}_wrist"]
                self.wrist_init_rot[name] = wrist_cfg[f"{name}_wrist_init_rot"]
                self.wrist_init_pos[name] = wrist_cfg[f"{name}_wrist_init_pos"]
                self.center_l[name] = wrist_cfg[f"{name}_center"]
                self.radius_l[name] = wrist_cfg[f"{name}_radius"]
    
    def init_server(self) -> None:
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
        self.servicer = HandTrackingServicer()
        handtracking_pb2_grpc.add_HandTrackingServiceServicer_to_server(
            self.servicer, self.server
        )
        self.server.add_insecure_port("[::]:12345")

        self.server.start()

        self.servicer.head = np.dot(YUP2ZUP_INV_2D, HEAD)
        self.servicer.points_right[:] = default_keypoint
        self.servicer.points_left[:] = default_keypoint
        self.servicer.matrix_right = np.eye(4)
        self.servicer.matrix_left = np.eye(4)

    def run(self) -> None:
        while True:
            self.servicer.update_event.set()