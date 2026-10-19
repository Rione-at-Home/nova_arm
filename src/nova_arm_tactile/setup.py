from glob import glob

from setuptools import find_packages, setup

package_name = "nova_arm_tactile"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/udev", glob("udev/*.rules")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ri-one",
    maintainer_email="gilsocojp@gmail.com",
    description="FlexiTac pads on the nova_arm grippers: driver, grasp action, viewer",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tactile_node = nova_arm_tactile.tactile_node:main",
            "grasp_node = nova_arm_tactile.grasp_node:main",
            "tactile_gui = nova_arm_tactile.gui:main",
            "fake_gripper = nova_arm_tactile.fake_gripper:main",
        ],
    },
)