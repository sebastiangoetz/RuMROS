from setuptools import find_packages, setup

package_name = 'lidar_vis'

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
    maintainer='aka',
    maintainer_email='alexander.kassuba@mailbox.tu-dresden.de',
    description='Visualizes and clusters data from RuMROS LiDAR messages in real time.',
    license='TBA',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'visualizer = lidar_vis.lidar_visualization:main'
        ],
    },
)
