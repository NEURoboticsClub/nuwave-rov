from setuptools import find_packages, setup

package_name = 'camera_pkg'

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
    maintainer='charstieve',
    maintainer_email='charlotteolivia930@gmail.com',
    description='ExploreHD cameras',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'camera_publisher = camera_pkg.openCVStreaming:main',
            'camera_subscriber = camera_pkg.testCVStreaming:main',
        ],
    },
)
