# Unitree H1 Description (URDF & MJCF)

## CHANGELOG

* 2023-09-08: v1.0.0

## Overview

This package contains a simplified robot description (URDF & MJCF) of the H1 Humanoid
Robot developed by [Unitree Robotics](https://www.unitree.com/).

<p align="center">
  <img src="images/H1.png" width="500"/>
</p>

H1 Humanoid have 19 joints:

```text
root [⚓] => /pelvis/
    left_hip_yaw_joint [⚙+Z] => /left_hip_yaw_link/
        left_hip_roll_joint [⚙+X] => /left_hip_roll_link/
            left_hip_pitch_joint [⚙+Y] => /left_hip_pitch_link/
                left_knee_joint [⚙+Y] => /left_knee_link/
                    left_ankle_joint [⚙+Y] => /left_ankle_link/
    right_hip_yaw_joint [⚙+Z] => /right_hip_yaw_link/
        right_hip_roll_joint [⚙+X] => /right_hip_roll_link/
            right_hip_pitch_joint [⚙+Y] => /right_hip_pitch_link/
                right_knee_joint [⚙+Y] => /right_knee_link/
                    right_ankle_joint [⚙+Y] => /right_ankle_link/
    torso_joint [⚙+Z] => /torso_link/
        left_shoulder_pitch_joint [⚙+Y] => /left_shoulder_pitch_link/
            left_shoulder_roll_joint [⚙+X] => /left_shoulder_roll_link/
                left_shoulder_yaw_joint [⚙+Z] => /left_shoulder_yaw_link/
                    left_elbow_joint [⚙+Y] => /left_elbow_link/
        right_shoulder_pitch_joint [⚙+Y] => /right_shoulder_pitch_link/
            right_shoulder_roll_joint [⚙+X] => /right_shoulder_roll_link/
                right_shoulder_yaw_joint [⚙+Z] => /right_shoulder_yaw_link/
                    right_elbow_joint [⚙+Y] => /right_elbow_link/
```

## Usages

### [MuJoCo](https://github.com/google-deepmind/mujoco)

```bash
# Python
pip install mujoco
python -m mujoco.viewer --mjcf=scene.xml

# C++
simulate scene.xml
```

### RViz

```bash
roslaunch h1_description display.launch
```

### Gazebo

```bash
roslaunch h1_description gazebo.launch
```

## License

This model is released under a [BSD-3-Clause License](LICENSE).
