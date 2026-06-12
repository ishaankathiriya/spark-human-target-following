from setuptools import find_packages, setup

package_name = 'htf_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/offboard.launch.py',
            'launch/follow.launch.py',
            'launch/pd_follow.launch.py',
            'launch/hover_test.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HTF',
    maintainer_email='autonomous.teamkaizen@gmail.com',
    description='Offboard velocity controller for the HTF quadrotor',
    license='MIT',
    entry_points={
        'console_scripts': [
            'offboard_controller  = htf_control.offboard_controller:main',
            'follow_controller    = htf_control.follow_controller:main',
            'pd_follow_controller = htf_control.pd_follow_controller:main',
            'debug_plotter        = htf_control.debug_plotter:main',
            'debug_logger         = htf_control.debug_logger:main',
            'hover_test           = htf_control.hover_test:main',
            'ground_truth_node    = htf_control.ground_truth_node:main',
            'human_world_estimator = htf_control.human_world_estimator:main',
        ],
    },
)
