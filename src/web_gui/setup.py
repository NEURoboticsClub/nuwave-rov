from setuptools import find_packages, setup
from glob import glob

package_name = 'web_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/static', glob('static/*')),
    ],
    install_requires=['setuptools', 'aiohttp'],
    zip_safe=True,
    maintainer='Cameron Zifcak',
    maintainer_email='cameronzifcak@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bridge_node = web_gui.bridge_node:main'
        ],
    },
)
