from setuptools import setup

package_name = 'cr3_teleop'

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
    maintainer='you',
    maintainer_email='you@example.com',
    description='Keyboard teleop for CR3 MoveIt Servo',
    license='Apache License 2.0',
    tests_require=['pytest'],
    scripts=[
        'scripts/keyboard_servo_teleop.py',
    ],
)