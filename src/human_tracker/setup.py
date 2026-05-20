from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'human_tracker'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ishaan',
    maintainer_email='autonomous.teamkaizen@gmail.com',
    description='Human detection and tracking using YOLOv8 + ByteTrack',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_node = human_tracker.tracker_node:main',
            'tracker_3d_node = human_tracker.tracker_3d_node:main',
        ],
    },
)
