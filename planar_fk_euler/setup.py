from setuptools import find_packages, setup

package_name = 'planar_fk_euler'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kkf525',
    maintainer_email='kkf525@york.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planar_fk_euler = planar_fk_euler.planar_fk_euler:main',
            'joint2 = planar_fk_euler.joint2:main',
            'pose_plotter = planar_fk_euler.pose_plotter:main'
        ],
    },
)
