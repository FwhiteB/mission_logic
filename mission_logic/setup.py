from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'mission_logic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='white',
    maintainer_email='white@todo.todo',
    description='Mission-level decision logic and simulated magnetic field nodes.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'magnetic_field_node = mission_logic.magnetic_field_node:main',
            'mission_node = mission_logic.mission_node:main',
        ],
    },
)
