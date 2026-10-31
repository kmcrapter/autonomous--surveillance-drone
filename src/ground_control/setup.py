from setuptools import setup, find_packages

package_name = 'ground_control'

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
    description='Streamlit Ground Control Station Dashboard',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dashboard = ground_control.run_dashboard:main',
        ],
    },
)
