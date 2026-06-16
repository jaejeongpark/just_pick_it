import os
from glob import glob

from setuptools import find_packages, setup

package_name = "pinky_amr_2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
        (
            os.path.join("share", package_name, "behavior_trees"),
            glob("behavior_trees/*.xml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ane",
    maintainer_email="mjzizou@gmail.com",
    description="PICKY2 AMR State Machine for just_pick_it system",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "odom_logger = pinky_amr_2.odom_logger:main",
            "scan_logger = pinky_amr_2.scan_logger:main",
            "obstacle_stop = pinky_amr_2.obstacle_stop:main",
            "tf_frame_adapter = pinky_amr_2.tf_frame_adapter:main",
            "picky2_state_machine = pinky_amr_2.state_machine:main",
        ],
    },
)
