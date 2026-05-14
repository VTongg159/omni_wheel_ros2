from setuptools import find_packages, setup

package_name = 'my_robot_arm_controller'

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
    maintainer='acer',
    maintainer_email='acer@robot',
    description='Arm keyboard controller for my_robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller=my_robot_arm_controller.arm_controller:main',
        ],
    },
)
