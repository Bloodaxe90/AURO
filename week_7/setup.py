from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'week_7'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.yaml'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'models'), glob(os.path.join('models', '*.sdf'))),
        (os.path.join('share', package_name, 'pddl'), glob(os.path.join('pddl', '*.pddl'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='pedro.ribeiro@york.ac.uk',
    description='Several TurtleBot3s in one arena, each in its own namespace, with barrels',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'barrel_manager = week_7.barrel_manager:main',
            'move_action_node = week_7.move_action_node:main',
        ],
    },
)
