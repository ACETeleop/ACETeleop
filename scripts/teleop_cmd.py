from pathlib import Path
import argparse
import yaml
from typing import Dict, Any

import ace_teleop
from ace_teleop.control.teleop import ACETeleop

def load_config(config_file_name: str) -> Dict[str, Any]:
    robot_config_path = (
        f"{ace_teleop.__path__[0]}/configs/robot" / Path(config_file_name)
    )
    with Path(robot_config_path).open("r") as f:
        cfg = yaml.safe_load(f)["robot_cfg"]

    return cfg

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

    try:
        while True:
            if args.debug:
                cmd, latest = teleoperator.step()
            else:
                cmd = teleoperator.step()
    except KeyboardInterrupt:
        exit(0)

if __name__ == "__main__":
    main()

