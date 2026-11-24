from glob import glob

from setuptools import find_packages, setup

package_name = 'week_7_solution'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/launch', glob('launch/*.yaml')),
        ('share/' + package_name + '/pddl', glob('pddl/*.pddl')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pedro Ribeiro',
    maintainer_email='pedro.ribeiro@york.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_action_node = week_7_solution.move_action_node:main',
            'load_action_node = week_7_solution.load_action_node:main',
            'unload_action_node = week_7_solution.unload_action_node:main',
            'take_action_node = week_7_solution.take_action_node:main',
            'put_action_node = week_7_solution.put_action_node:main',
            'teleop_keyboard = week_7_solution.teleop_keyboard:main'
        ],
    },
)
