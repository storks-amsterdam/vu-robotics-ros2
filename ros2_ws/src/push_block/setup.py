import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'push_block'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files from the launch directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        # Include all model files
        (os.path.join('share', package_name, 'models', 'push_block'), glob(os.path.join('models', 'push_block', '*.sdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aleksey',
    maintainer_email='aleksey@storks.amsterdam',
    description='A package to demonstrate MoveIt planning in Gazebo to push a block.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'block_pusher = push_block.block_pusher:main',
            'block_pusher_collision = push_block.block_pusher_collision:main',
        ],
    },
)