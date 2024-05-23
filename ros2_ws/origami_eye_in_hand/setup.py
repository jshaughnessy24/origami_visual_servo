from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'origami_eye_in_hand'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/origami_eye_in_hand_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jenny',
    maintainer_email='jshaughnessy@wpi.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_controller = origami_eye_in_hand.velocity_controller:main',
            'single_module_jacobian_server = origami_eye_in_hand.single_module_jacobian_server:main',
        ],
    },
)
