from setuptools import find_packages, setup

package_name = 'power_monitor_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/power_monitor_pkg/config', ['config/power_monitor_run_config.yaml']),
        ('share/power_monitor_pkg/launch', ['launch/multi_power_monitor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nuwave-rov',
    maintainer_email='definitely@not.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'power_monitor_pub = power_monitor_pkg.power_monitor_pub:main',
        ],
    },
)
