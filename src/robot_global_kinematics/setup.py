from setuptools import find_packages, setup

package_name = 'robot_global_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='meeppo',
    maintainer_email='meeppo@todo.todo',
    description='Global coordinate kinematics for differential drive robot Robin',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'global_coordinate_node = '
            'robot_global_kinematics.global_coordinate_node:main',
        ],
    },
)