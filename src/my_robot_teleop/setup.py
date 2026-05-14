from setuptools import find_packages, setup

package_name = 'my_robot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'sensor_msgs',
    ],
    zip_safe=True,
    maintainer='acer',
    maintainer_email='thanglequydon2005@gmail.com',
    description='Teleop keyboard controller for omni robot',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts':  [
            'omni_teleop = my_robot_teleop.teleop:main',
        ],
    },
)
