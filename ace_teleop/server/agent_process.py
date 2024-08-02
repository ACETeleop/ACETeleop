import numpy as np
import cv2

from multiprocessing import Process, Queue, Event
from ace_teleop.server.dynamixel_agent import DynamixelAgent
from ace_teleop.configs.server.ace_const import *
from ace_teleop.cam.single_hand_detector import SingleHandDetector, hand_pos


class AgentProcess(Process):
    def __init__(
        self, mode: str, cfg: dict, name: str, process_event: Event, res_queue: Queue
    ) -> None:
        super(AgentProcess, self).__init__()
        self.dynamixel_cfg = cfg["dynamixel_cfg"]

        hand_cfg = cfg["hand_cfg"]
        self.cam_num = hand_cfg["cam_num"]
        self.hand_type = hand_cfg["hand_type"]
        self.mode = mode

        self.name = name

        self.process_event = process_event
        self.res_queue = res_queue

    def init(self) -> None:
        self.agent = DynamixelAgent(**self.dynamixel_cfg)

        cap = cv2.VideoCapture(self.cam_num)

        if not cap.isOpened():
            print(f"Error: Could not open {self.name} webcam.")
            cap.release()
            exit()

        self.cap = cap

        self.detector = SingleHandDetector(hand_type=self.hand_type, selfie=False)

    def compute(self) -> None:
        wrist = self.agent.get_ee()

        ret, frame = self.cap.read()

        fingers = hand_pos(ret, frame, self.detector)

        if fingers is not None:
            if self.mode == "normal":
                if self.name == "left":
                    fingers = np.dot(fingers, R_x_90_ccw_rot)
                    fingers = np.dot(fingers, R_z_90_cw_rot)
                else:
                    fingers = np.dot(fingers, R_y_90_ccw_rot)
                    fingers = np.dot(fingers, R_x_90_ccw_rot)
            elif self.mode == "mirror":
                if self.name == "left":
                    fingers = np.dot(fingers, R_x_90_ccw_rot)
                    fingers = np.dot(fingers, R_z_90_ccw_rot)
                else:
                    fingers = np.dot(fingers, R_y_90_cw_rot)
                    fingers = np.dot(fingers, R_x_90_ccw_rot)
        self.res_queue.put([wrist, fingers, frame])

    def run(self) -> None:
        self.init()
        while True:
            if self.process_event.is_set():
                self.compute()
                self.process_event.clear()

    def terminate(self) -> None:
        return super().terminate()
