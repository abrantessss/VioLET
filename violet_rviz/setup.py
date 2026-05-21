import os
from glob import glob
from setuptools import setup

package_name = 'violet_rviz'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='RViz plotting tools for VioLET telemetry',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plot_bridge = violet_rviz.plot_bridge:main',
        ],
    },
)
