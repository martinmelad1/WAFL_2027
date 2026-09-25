from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'hardware_wafl2026'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'materials', 'textures'), glob('materials/textures/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        # INCLUDE YOUR CONFIG FOLDER
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # INCLUDE YOUR MAP FOLDER
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='montasser',
    maintainer_email='montasser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_covariance_fix = hardware_wafl2026.odom_covariance_fix:main',
            'pose_persistence_node = hardware_wafl2026.pose_persistence_node:main',
        ],
    },
)
