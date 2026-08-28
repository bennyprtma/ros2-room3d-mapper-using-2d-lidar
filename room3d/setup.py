from setuptools import setup
from glob import glob

package_name = 'room3d'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benny',
    maintainer_email='benny@example.com',
    description='Deteksi ruang 3D dengan lidar tilting + odometri encoder.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_node = room3d.odom_node:main',
            'cloud_mapper = room3d.cloud_mapper:main',
            'arrow_teleop = room3d.arrow_teleop:main',
        ],
    },
)
