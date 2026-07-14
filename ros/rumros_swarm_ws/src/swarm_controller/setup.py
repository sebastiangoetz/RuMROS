from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'swarm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='example',
    maintainer_email='mail@example.com',
    description='This controller implements a variety of swarm behaviors. Switching between behaviors is implemented via String messages on ROS topics.',
    license='TBA',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_main = swarm_controller.node_main:main'
        ],
    },
)
