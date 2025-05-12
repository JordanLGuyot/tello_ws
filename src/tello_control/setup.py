from setuptools import setup

package_name = 'tello_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],             # installs tello_control/
    data_files=[
        # Register package with ament index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='YOUR_EMAIL@example.com',
    description='Keyboard tele-op node for the DJI Tello (ROS 2 Humble).',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_control = tello_control.tello_control:main',
        ],
    },
)