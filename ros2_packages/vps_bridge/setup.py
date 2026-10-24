from setuptools import setup
import os
from glob import glob

package_name = 'vps_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'vps_bridge_node = vps_bridge.vps_bridge_node:main',
            'ir_preprocessor = vps_bridge.ir_preprocessor:main',
        ],
    },
)
