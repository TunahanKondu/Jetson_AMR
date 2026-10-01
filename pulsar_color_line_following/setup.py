from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'pulsar_color_line_following'

setup(
    name=package_name,
    version='0.1.4',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pulsar Robotics',
    maintainer_email='pulsar@example.com',
    description=(
        'Orange line detection, safe alignment control and RTP/H.264 streaming'),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'color_line_detector = '
            'pulsar_color_line_following.color_detector_node:main',
            'color_line_controller = '
            'pulsar_color_line_following.color_controller_node:main',
        ],
    },
)
