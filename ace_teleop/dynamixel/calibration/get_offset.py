import tyro
import numpy as np

from typing import Tuple
from dataclasses import dataclass, field
from ace_teleop.dynamixel.driver import DynamixelDriver


@dataclass
class Args:
    port: str = "/dev/ttyUSB0"
    """The port connected to."""

    type: str = "left"
    """The type of the agent (left or right)."""

    start_joints: Tuple[float, ...] = field(init=False)
    """The start joint angles (in radians)."""

    joint_signs: Tuple[float, ...] = field(init=False)
    """The joint signs is -1 for all dynamixel"""

    gripper: bool = False
    """Reserved for later work"""

    def __post_init__(self):
        if self.type == "right":
            self.start_joints = (0, 1.571, 0, 0, 0, 0)
            self.joint_signs = (-1, -1, -1, -1, -1, -1)
        else:  # default to left agent
            self.start_joints = (0, -1.571, 0, 0, 0, 0)
            self.joint_signs = (-1, -1, -1, -1, -1, -1)

        assert len(self.joint_signs) == len(self.start_joints)
        for idx, j in enumerate(self.joint_signs):
            assert (
                j == -1 or j == 1
            ), f"Joint idx: {idx} should be -1 or 1, but got {j}."

    @property
    def num_robot_joints(self) -> int:
        return len(self.start_joints)

    @property
    def num_joints(self) -> int:
        extra_joints = 1 if self.gripper else 0
        return self.num_robot_joints + extra_joints


def get_config(args: Args) -> None:
    joint_ids = list(range(1, args.num_joints + 1))
    driver = DynamixelDriver(joint_ids, port=args.port, baudrate=57600)

    def get_error(offset: float, index: int, joint_state: np.ndarray) -> float:
        joint_sign_i = args.joint_signs[index]
        joint_i = joint_sign_i * (joint_state[index] - offset)
        start_i = args.start_joints[index]
        return np.abs(joint_i - start_i)

    for _ in range(10):
        driver.get_joints()

    driver.set_torque_mode(False)
    while True:
        try:
            best_offsets = []
            curr_joints = driver.get_joints()
            for i in range(args.num_robot_joints):
                best_offset = 0
                best_error = 1e6
                for offset in np.linspace(-8 * np.pi, 8 * np.pi, 8 * 4 + 1):
                    error = get_error(offset, i, curr_joints)
                    if error < best_error:
                        best_error = error
                        best_offset = offset
                best_offsets.append(best_offset)

            print()
            print("true value                 : ", [f"{x:.3f}" for x in curr_joints])
            print("best offsets               : ", [f"{x:.3f}" for x in best_offsets])
            print(
                "best offsets function of pi: ["
                + ", ".join(
                    [f"{int(np.round(x/(np.pi/2)))}*np.pi/2" for x in best_offsets]
                )
                + " ]",
            )

            if args.gripper:
                print(
                    "gripper open (degrees)       ",
                    np.rad2deg(driver.get_joints()[-1]) - 0.2,
                )
                print(
                    "gripper close (degrees)      ",
                    np.rad2deg(driver.get_joints()[-1]) - 42,
                )

        except KeyboardInterrupt:
            driver.close()
            return ()


def main(args: Args) -> None:
    get_config(args)


if __name__ == "__main__":
    main(tyro.cli(Args))
