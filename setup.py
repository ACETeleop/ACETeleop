from setuptools import setup, find_packages

with open("README.md", "r") as fh:

    long_description = fh.read()

setup(
    name="ace_teleop",
    version="0.1.0",
    packages=find_packages(),
    description="Core elements of ACE-teleoperation system",
    long_description=open("README.md").read(),
    long_description_content_type="text/markdown",
    author="Shiqi Yang, Minghuan Liu, Yuzhe Qin, Jialong Li.",
    install_requires=[
        "numpy",
        "pin",
        "nlopt",
        "avp_stream",
        "dex_retargeting==0.1.1",
        "torch",
        "pynput",
        "pytransform3d",
        "opencv-python",
        "mediapipe",
        "pyquaternion",
        "dynamixel_sdk",
        "tyro",
    ],
)