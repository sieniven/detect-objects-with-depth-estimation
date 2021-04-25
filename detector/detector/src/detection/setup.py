import os
from setuptools import setup
from glob import glob

package_name = 'detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niven',
    maintainer_email='sieniven@gmail.com',
    description='Package for detection capability',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'copilot_detector = detection.detector_main:main'
        ],
    },
)
