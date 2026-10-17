import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'wafl_simulation'


def tree(src, dest):
    """Install a whole directory tree (models/ with meshes) under share/<pkg>/."""
    out = {}
    for root, _dirs, files in os.walk(src):
        if files:
            target = os.path.join('share', package_name, dest, os.path.relpath(root, src))
            out[target] = [os.path.join(root, f) for f in files]
    return list(out.items())


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ] + tree('models', 'models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WAFL 2027 simulation owner',
    maintainer_email='todo@example.com',
    description='WAFL 2027 Gazebo Harmonic simulation assets',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # sim_fork_adapter.py is written by a teammate; register it here when it lands:
            # 'sim_fork_adapter = wafl_simulation.sim_fork_adapter:main',
        ],
    },
)
