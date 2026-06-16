from setuptools import find_packages, setup

package_name = 'crab_recognition_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', ['crab_recognition_models/30epochs_best.pt']),
    ],
    install_requires=['setuptools', 'numpy', 'ultralytics'],
    zip_safe=True,
    maintainer='nuwave',
    maintainer_email='miayim@gmail.com',
    description='Crab recognition node using YOLO on compressed camera topics.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'crab_recognition = crab_recognition_pkg.crab_recognition_node:main',
        ],
    },
)
