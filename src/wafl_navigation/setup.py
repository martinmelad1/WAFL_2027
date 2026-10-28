import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'wafl_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WAFL Team',
    maintainer_email='you@example.com',
    description='WAFL base_controller and navigation config',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'base_controller = wafl_navigation.base_controller:main',
            'smoke_test = wafl_navigation.smoke_test:main',
        ],
    },
)