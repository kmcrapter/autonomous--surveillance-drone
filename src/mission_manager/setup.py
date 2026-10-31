from setuptools import setup, find_packages

package_name = 'mission_manager'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robotics Simulation Team',
    maintainer_email='engineer@antigravity.internal',
    description='Autonomous Mission State Machine and 3D Flight Controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = mission_manager.mission_node:main',
        ],
    },
)
