import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'mate_rov_sim'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
]
# Install every file under models/, keeping the folder structure (meshes/, materials/, ...)
for path in glob('models/**/*', recursive=True):
    if os.path.isfile(path):
        data_files.append((os.path.join('share', package_name, os.path.dirname(path)), [path]))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Your MATE team',
    maintainer_email='team@example.com',
    description='Gazebo Sim simulation of the NUWave MATE ROV, driven by the real NUWave nodes',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sim_motor_interface = mate_rov_sim.sim_motor_interface:main',
            'imu_sim = mate_rov_sim.imu_sim:main',
            'depth_sensor_sim = mate_rov_sim.depth_sensor_sim:main',
            'camera_sim = mate_rov_sim.camera_sim:main',
            'keyboard_teleop = mate_rov_sim.keyboard_teleop:main',
            'joy_mux = mate_rov_sim.joy_mux:main',
        ],
    },
)
