from setuptools import setup
import os
from glob import glob

package_name = 'personnel_security'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Antigravity Robotics Team',
    maintainer_email='engineer@antigravity.internal',
    description='Multi-Layer Personnel Identification, Secure ID Authentication & Suspicious Activity Detection',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'personnel_tag_node = personnel_security.personnel_tag_node:main',
            'credential_authentication_node = personnel_security.credential_authentication_node:main',
            'face_verification_node = personnel_security.face_verification_node:main',
            'identity_correlation_node = personnel_security.identity_correlation_node:main',
            'activity_assessment_node = personnel_security.activity_assessment_node:main',
            'personnel_event_logger = personnel_security.personnel_event_logger:main',
        ],
    },
)
