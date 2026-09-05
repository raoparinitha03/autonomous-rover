import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_vision_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools', 'onnxruntime', 'numpy', 'pyserial'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='YOLO-ONNX vision pipeline + serial arm bridge (ROS2 port).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = robot_vision_pkg.vision_node:main',
            'arm_action_server = robot_vision_pkg.arm_action_server:main',
        ],
    },
)
