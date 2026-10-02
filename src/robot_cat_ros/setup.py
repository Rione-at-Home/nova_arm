from setuptools import setup

setup(
    name='robot_cat_ros', version='0.2.0', packages=['robot_cat_ros'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/robot_cat_ros']),
        ('share/robot_cat_ros', ['package.xml', 'README.md', 'LICENSE']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Sandeep', maintainer_email='maintainer@example.com',
    description='Animated cat face with queued bilingual offline speech',
    license='MIT',
    entry_points={'console_scripts': [
        'face = robot_cat_ros.face:main',
        'speech = robot_cat_ros.node:main',
        'controls = robot_cat_ros.controls:main',
    ]},
)
