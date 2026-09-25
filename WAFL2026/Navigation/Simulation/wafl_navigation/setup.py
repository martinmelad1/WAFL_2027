from setuptools import setup
from glob import glob
import os

package_name = 'wafl_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[

        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        ('share/' + package_name,
         ['package.xml']),

        (os.path.join(
            'share',
            package_name,
            'launch'),
         glob('launch/*.py')),

        (os.path.join(
            'share',
            package_name,
            'config'),
         glob('config/*.yaml')),

        (os.path.join(
            'share',
            package_name,
            'maps'),
         glob('maps/*'))

    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mariam',
    maintainer_email='test@test.com',

    description='Navigation package',

    license='Apache License 2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'caster_controller = wafl_navigation.caster_controller:main',
        ],
    },
)
