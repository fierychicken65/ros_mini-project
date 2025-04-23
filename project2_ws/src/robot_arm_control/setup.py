from setuptools import setup

package_name = 'robot_arm_control'

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
    zip_safe=True,
    maintainer='gayatrid',
    maintainer_email='gayatridavuluri147@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'arm_teleop_control = robot_arm_control.arm_teleop_control:main',
             'arm_inv_kine_control = robot_arm_control.arm_inv_kine_control:main',
             'vacuum_gripper = robot_arm_control.vacuum_gripper:main'
        ],
    },
)
