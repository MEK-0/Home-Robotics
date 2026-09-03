from pathlib import Path

from setuptools import find_packages, setup

package_name = 'home_robotics_control'
repository_root = Path(__file__).resolve().parents[3]
shared_files = []
for source_root in (repository_root / 'config', repository_root / 'simulation'):
    for path in source_root.rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts:
            relative = path.relative_to(repository_root)
            shared_files.append(
                (
                    str(Path('share') / package_name / relative.parent),
                    [str(Path('../../..') / relative)],
                )
            )

setup(
    name=package_name,
    version='0.1.0',
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
    ] + shared_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mek',
    maintainer_email='esat.kolay19@gmail.com',
    description='Authoritative MuJoCo runtime for Home Robotics ros2_control.',
    license='NOASSERTION',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            (
                'rail_state_publisher = '
                'home_robotics_control.rail_state_publisher:main'
            ),
            (
                'mujoco_joint_state_bridge = '
                'home_robotics_control.mujoco_joint_state_bridge:main'
            ),
        ],
    },
)
