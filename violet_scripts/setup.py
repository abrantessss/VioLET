import os
from glob import glob
from setuptools import setup

package_name = 'violet_scripts'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Mission scripts for VioLET',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'easyglider_circle = violet_scripts.easyglider_circle:main',
            'easyglider_lemniscate = violet_scripts.easyglider_lemniscate:main',
            'easyglider_line = violet_scripts.easyglider_line:main',
            'shuttle_circle = violet_scripts.shuttle_circle:main',
            'shuttle_lemniscate = violet_scripts.shuttle_lemniscate:main',
            'shuttle_line = violet_scripts.shuttle_line:main',
            'shuttle_waypoint = violet_scripts.shuttle_waypoint:main',
            'combined_mixer_test = violet_scripts.combined_mixer_test:main',
            'combined_rate_controller_test = violet_scripts.combined_rate_controller_test:main',
            'combined_attitude_controller_test = violet_scripts.combined_attitude_controller_test:main',
            'shuttle_rate_controller_test = violet_scripts.shuttle_rate_controller_test:main',
        ],
    },
)
