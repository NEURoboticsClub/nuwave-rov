from setuptools import find_packages, setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/joystick_config.yaml',
            'config/thruster_config.yaml',
            'config/new_thruster_config.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gallifrey',
    maintainer_email='sirdeshpande.m@northeastern.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thruster_controller_node=controller.thruster_controller_node:main',
            'joystick_identify=controller.joystick_identify:main',
        ],
    },
)
