from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'week_8'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'models', 'turtlebot3_world_auro'),
            glob(os.path.join('models', 'turtlebot3_world_auro', '*'))),
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*.yaml')) + glob(os.path.join('maps', '*.pgm'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='pedro.ribeiro@york.ac.uk',
    description=('Mapping with slam_toolbox, and navigation with Nav2, '
                 'for one simulated TurtleBot3'),
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
