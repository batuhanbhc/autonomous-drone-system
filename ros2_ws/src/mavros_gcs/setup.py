from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mavros_gcs'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ↓ register the launch directory
        (os.path.join('share', package_name, 'launch'),
            glob('mavros_gcs/launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='batuhan',
    maintainer_email='batuhan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'info_panel       = mavros_gcs.nodes.info_panel:main',
            'teleop_keyboard  = mavros_gcs.nodes.teleop_keyboard:main',
            'stream_viewer    = mavros_gcs.nodes.stream_viewer:main',
            'gcs_heartbeat    = mavros_gcs.nodes.heartbeat:main',
        ],
    },
)