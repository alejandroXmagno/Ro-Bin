from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'QUICKSTART.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lechonka',
    maintainer_email='lechonka@todo.todo',
    description='Autonomous exploration package for Rover Robotics mini rover',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'autonomous_explorer = rover_exploration.autonomous_explorer:main',
            'blazepose_detector = rover_exploration.blazepose_detector:main',
        ],
    },
)
