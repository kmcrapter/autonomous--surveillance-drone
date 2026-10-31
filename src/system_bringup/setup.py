from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'system_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Simulation Team',
    maintainer_email='engineer@antigravity.internal',
    description='System Orchestrator, Configuration, and Launch Files',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orchestrator = system_bringup.system_orchestrator:main',
        ],
    },
)
