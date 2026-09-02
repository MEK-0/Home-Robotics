from setuptools import find_packages, setup

package_name = 'home_robotics_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mek',
    maintainer_email='esat.kolay19@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rail_state_publisher = home_robotics_control.rail_state_publisher:main',
            'mujoco_joint_state_bridge = home_robotics_control.mujoco_joint_state_bridge:main',
        ],
    },
)