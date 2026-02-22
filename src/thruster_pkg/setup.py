from setuptools import find_packages, setup

package_name = 'thruster_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/thruster_run_config.yaml'
        ]),
        ('share/' + package_name + '/launch', [
            'launch/thruster.launch.py'
        ])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nuwave',
    maintainer_email='nuwave@todo.todo',
    description='package for thrusters',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'thruster_node = '+package_name+'.thruster_driver_simple:main',
            'new_thruster_node = '+package_name+'.new_thruster_driver:main',
            'thruster_sub = '+package_name+'.thruster_sub:main',
            'thruster_test = '+package_name+'.thruster_test:main',
            'PCA9685 = '+package_name+'.PCA9685:main'
        ],
    },
)
