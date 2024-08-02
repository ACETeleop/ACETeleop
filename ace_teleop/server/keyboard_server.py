
from pynput.keyboard import Key, Listener, KeyCode
import numpy as np
import time

from ace_teleop.configs.server.ace_const import *
from ace_teleop.server.server import Server

class KeyboardServer(Server):
    def __init__(self, cfg: dict, dx: int = 0.02) -> None:
        super().__init__(cfg)

        self.cur_finger_joint_pos = {}
        self.cur_ee_pos = self.wrist_init_pos
        self.cur_ee_rot = self.wrist_init_rot

        for name in ["left", "right"]:
            self.wrist[name] = np.eye(4)
            self.cur_finger_joint_pos[name] = np.zeros((25,3))

        self.dx = dx

        self.listener = Listener(
            on_press=self.on_press)
        self.listener.daemon = True
        self.listener.start()

    def on_press(self, key):
        # Left hand
        if key == KeyCode(char='w'):
            self.cur_ee_pos["left"][1] += self.dx
        if key == KeyCode(char='a'):
            self.cur_ee_pos["left"][0] -= self.dx
        if key == KeyCode(char='s'):
            self.cur_ee_pos["left"][1] -= self.dx
        if key == KeyCode(char='d'):
            self.cur_ee_pos["left"][0] += self.dx
        
        # Right hand
        if key == Key.up:
            self.cur_ee_pos["right"][1] += self.dx
        if key == Key.left:
            self.cur_ee_pos["right"][0] -= self.dx
        if key == Key.down:
            self.cur_ee_pos["right"][1] -= self.dx
        if key == Key.right:
            self.cur_ee_pos["right"][0] += self.dx

    def run(self) -> None:
        while True:
            for name in ["left", "right"]:
                self.wrist[name][:3, 3] = self.cur_ee_pos[name]
                self.wrist[name][:3, :3] = self.cur_ee_rot[name]

                if self.is_ACE:
                    self.wrist[name] = np.dot(R_z_90_ccw_pose, self.wrist[name])
                self.wrist[name] = np.dot(YUP2ZUP_INV_2D, self.wrist[name])

            self.servicer.points_right[:] = self.cur_finger_joint_pos["right"]
            self.servicer.points_left[:] = self.cur_finger_joint_pos["left"]
            self.servicer.matrix_right = self.wrist["right"]
            self.servicer.matrix_left = self.wrist["left"]

            self.servicer.update_event.set()
            
            time.sleep(0.1)