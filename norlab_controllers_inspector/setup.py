from setuptools import setup

package_name = 'norlab_controllers_inspector'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    # install_requires=['setuptools', 'norlab_controllers_msgs'],
    zip_safe=True,
    maintainer='Cyril',
    maintainer_email='cygof@ulaval.ca',
    description='Inspector for robot controllers, evaluating their accuracy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspector_node = norlab_controllers_inspector.inspector_node:main',
        ],
    },
)
