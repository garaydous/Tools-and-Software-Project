from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'image_click_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maria',
    maintainer_email='marialuisgarrido100@gmail.com',
    description='Image-based click teleop interface for UR5 robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_publisher = image_click_teleop.camera_publisher:main',
            'click_teleop = image_click_teleop.click_teleop_node:main',
        ],
    },
)
