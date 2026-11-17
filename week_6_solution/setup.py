from setuptools import find_packages, setup

package_name = 'week_6_solution'

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
            'task9_data_collector = week_6_solution.task9_data_collector:main',
            'task10_plot_data = week_6_solution.task10_plot_data:main',
            'task11_fit_data = week_6_solution.task11_fit_data:main',
        ],
    },
)
