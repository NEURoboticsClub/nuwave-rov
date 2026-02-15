from setuptools import find_packages, setup

package_name = 'adc_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nuwave-rov',
    maintainer_email='definitely@not.com',
    description='Reads adc data from an AD7991YRJZ-1500RL7 and publishes it as ROS2 messages.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'adc_reader = '+package_name+'.adc_reader:main'
        ],
    },
)
