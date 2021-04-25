from setuptools import setup
import os
from glob import glob


package_name = 'calibration'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niven',
    maintainer_email='sieniven@gmail.com',
    description='Calibration capabilities for Detection and Depth estimation in ROS2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate_stereo = calibration.stereo_calibrator:main',
            'user_input_node = calibration.user_input_node:main',
            'stereo_image_saver_node = calibration.stereo_image_saver_node:main',
            'publish_camera_info = calibration.publish_camera_info:main',
        ],
    },
)
