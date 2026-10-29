import os
from glob import glob 
from setuptools import find_packages, setup

package_name = 'rover_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/rover_description']),
    ('share/rover_description', ['package.xml']),
    (os.path.join('share', 'rover_description', 'urdf'),
        glob('urdf/*')),
    (os.path.join('share', 'rover_description', 'launch'),
        glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), 
        glob('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),
    glob('worlds/*')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pari',
    maintainer_email='raoparinitha03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cmd_vel_relay = rover_description.cmd_vel_relay:main',
        ],
    },
)
