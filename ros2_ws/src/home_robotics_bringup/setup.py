from setuptools import find_packages, setup

package_name = 'home_robotics_bringup'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/phase2_control.launch.py']),
        ('share/' + package_name + '/config', ['config/controllers.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mek',
    maintainer_email='esat.kolay19@gmail.com',
    description='Phase 2 ros2_control bringup for Home Robotics.',
    license='NOASSERTION',
    extras_require={'test': ['pytest']},
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            (
                'controller_startup = '
                'home_robotics_bringup.controller_startup:main'
            ),
            'mujoco_runtime = home_robotics_bringup.mujoco_runtime:main',
        ]
    },
)
