import argparse
import yaml
from pathlib import Path
import ace_teleop

def load_config(config_file_name):
    config_path = (
        f"{ace_teleop.__path__[0]}/configs/server" / Path(config_file_name)
    )
    with Path(config_path).open("r") as f:
        cfg = yaml.safe_load(f)["robot_cfg"]

    return cfg


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        "-c",
        choices=[
            "h1_inspire",
            "h1_inspire_mirror",
            "xarm_ability",
            "xarm_ability_mirror",
            "single_test",
            'gr1_gripper',
            'gr1_gripper_mirror',
            'franka_gripper',
            'franka_gripper_mirror',
            'single_demo'
        ],
        default="h1_inspire",
    )
    parser.add_argument("--ip", default="localhost")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("--keyboard", action="store_true")
    parser.add_argument("--mode", "-m", default="normal")
    args = parser.parse_args()

    config_file_name = f"{args.config}.yml"
    cfg = load_config(config_file_name)

    if args.keyboard:
        from ace_teleop.server.keyboard_server import KeyboardServer
        server = KeyboardServer(cfg=cfg)
    else:
        from ace_teleop.server.ace_server import ACEServer
        server = ACEServer(cfg=cfg, debug=args.debug)

    server.run()
