from setuptools import find_packages, setup

package_name = 'thermal_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omnibot',
    maintainer_email='vincen7.nguyen@gmail.com',
    description='ROS2 package for thermal camera node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thermal_camera_node = thermal_camera.thermal_camera_node:main'
        ],
    },
)