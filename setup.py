import os
from glob import glob
from setuptools import setup

package_name = 'norlab_controllers_ros'

setup(
    name='norlab_controllers_ros',
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dominic Baril',
    maintainer_email='dominic.baril@norlab.ulaval.ca',
    description='ROS 2 wrapper for norlab_controllers',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = norlab_controllers_ros.controller_node:main',
        ],
    },
)
