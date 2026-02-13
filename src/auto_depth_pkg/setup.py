from setuptools import find_packages, setup

package_name = 'auto_depth_pkg'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'auto_depth_node=auto_depth_pkg.auto_depth_node:main',
            'auto_depth_test=auto_depth_pkg.auto_depth_test:main'
        ],
    },
)
