from setuptools import find_packages, setup

package_name = 'seam_controller'

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
    maintainer_email='your_email@example.com',
    description='Seam tracking controller and motor control nodes',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_node = seam_controller.pid_node:main',
            'cmd_vel_to_motor = seam_controller.cmd_vel_to_motor:main',
            'control_logger = seam_controller.control_logger:main',
            'tracking_logger = seam_controller.tracking_logger:main',
        ],
    },
)