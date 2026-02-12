from setuptools import find_packages, setup

package_name = 'houston_pkg'

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
        ]),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nuwave',
    maintainer_email='neumate2022@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'houston = houston_pkg.houston:main'
        ],
    },
)
