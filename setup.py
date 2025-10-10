from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'trsa_lab1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'video'), glob('video/*.mov')),
    (os.path.join('share', package_name, 'calibration/images'), glob('calibration/images/*')),
    (os.path.join('share', package_name, 'calibration'), ['calibration/ost.yaml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='antoniopalves',
    maintainer_email='antoniopalves@todo.todo',
    description='TRSA Lab 1 package with camera drivers and calibration tools',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_driver = trsa_lab1.camera_driver:main',
            'image_reader = trsa_lab1.image_reader:main',
            'camera_calibration_pub = trsa_lab1.camera_calibration_pub:main',
            'camera_rectifier = trsa_lab1.camera_rectifier:main',
            'image_convert = trsa_lab1.image_convert:main',
            'object_detection = trsa_lab1.object_detection:main',
            'camera_driver2 = trsa_lab1.camera_driver2:main',
            'image_reader2 = trsa_lab1.image_reader2:main',
        ],
    },
)
