from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'processor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vscode',
    maintainer_email='ziangliu@tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'beamforming_node = processor.beamforming_node:main',
        'beamforming_car = processor.beamforming_node_circular_grid:main'
        ],
    },
)
