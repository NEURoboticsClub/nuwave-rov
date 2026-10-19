import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'web_gui'


def static_data_files():
    """Install the static dir recursively, preserving subdirectory layout.

    allows it to load all subdirectories of static like vendor/, and not just the top level files
    """
    entries = []
    for root, _dirs, files in os.walk('static'):
        paths = [os.path.join(root, f) for f in files]
        if paths:
            entries.append(('share/' + package_name + '/' + root, paths))
    return entries

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        *static_data_files(),
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
