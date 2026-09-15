from glob import glob
from setuptools import find_packages, setup

package_name = 'pulsar_color_line_following'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pulsar_robotic',
    maintainer_email='pulsar_robotic@todo.todo',
    description='Color line following for the Pulsar robot.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'color_line_detector = pulsar_color_line_following.color_detector_node:main',
            'color_line_controller = pulsar_color_line_following.color_controller_node:main',
        ],
    },
)
