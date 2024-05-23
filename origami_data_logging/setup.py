from setuptools import find_packages, setup

package_name = 'origami_data_logging'

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
    maintainer='jenny',
    maintainer_email='jshaughnessy@wpi.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_detector = origami_data_logging.marker_detector:main',
            'bag_data_logger = origami_data_logging.bag_data_logger:main',
            'image_to_video_converter = origami_data_logging.image_to_video_converter:main',
        ],
    },
)
