from setuptools import find_packages, setup

package_name = 'omni_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/controller.yaml']),
        ('share/' + package_name + '/launch', ['launch/controller.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eleclabs',
    maintainer_email='eleclabs@example.com',
    description='3-wheel Kiwi/omni robot kinematics controller.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'omni_kinematics_node = omni_robot_controller.omni_kinematics_node:main',
            'omni_odometry_node = omni_robot_controller.omni_odometry_node:main',
            'omni_hardware_interface = omni_robot_controller.omni_hardware_interface:main',
        ],
    },
)
