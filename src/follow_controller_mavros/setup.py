from setuptools import find_packages, setup

package_name = 'follow_controller_mavros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/bringup.launch.py',
            'launch/follow.launch.py',
            'launch/circle.launch.py',
            'launch/vel_debug.launch.py',
            'launch/offboard_mode_test.launch.py',
            'launch/offboard_arm_test.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/px4_custom.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='btp',
    maintainer_email='ishaankathiriya2005@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pd_follow_controller_mavros = follow_controller_mavros.pd_follow_controller_mavros:main',
            'offboard_vel_debug = follow_controller_mavros.offboard_vel_debug:main',
            'offboard_control = follow_controller_mavros.offboard_control:main',
            'offboard_mode_test = follow_controller_mavros.offboard_mode_test:main',
            'arm_node = follow_controller_mavros.arm_node:main',
            'fake_pose_pub = follow_controller_mavros.fake_pose_pub:main',
            'flight_manager = follow_controller_mavros.flight_manager:main',
            'circle_node = follow_controller_mavros.circle_node:main',
        ],
    },
)
