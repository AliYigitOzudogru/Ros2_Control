from setuptools import setup, find_packages

package_name = 'aero_arm_control'

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
    maintainer='ubuntu',
    maintainer_email='n.cem110@gmail.com',
    description='Aero arm control package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_teleop = aero_arm_control.arm_teleop:main',
            'ps4_rover_controller = aero_arm_control.ps4_rover_controller:main'
        ],
    },
)

