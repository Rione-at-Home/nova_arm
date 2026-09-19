from setuptools import find_packages, setup

package_name = 'nova_arm_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
        data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch",
            ["launch/arm_driver.launch.py"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ri-one',
    maintainer_email='gilsocojp@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arm_driver = nova_arm_driver.ArmDriver:main',
            'nova_teleop = nova_arm_driver.nova_teleop:main',
            'nova_pose_manager = nova_arm_driver.nova_pose_manager:main',
            'nova_demo = nova_arm_driver.nova_demo:main',
            'qr_trigger = nova_arm_driver.qr_trigger:main',
            'HeadDriver = nova_arm_driver.head_driver:main',
            'presenter_coordinator = nova_arm_driver.presentor_coordinator:main',
            'HeadNode = nova_arm_driver.HeadNode:main',
        ],
    },
)
