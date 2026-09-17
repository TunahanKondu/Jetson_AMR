from setuptools import find_packages, setup

package_name = 'plc_udp_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pulsar_robotic',
    maintainer_email='tkondu7042@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [        	
            'plc_udp_bridge_node = plc_udp_bridge.plc_udp_bridge_node:main',    
            'plc_mission_adapter_node = plc_udp_bridge.plc_mission_adapter_node:main',            
        ],
    },
)
