from glob import glob
from setuptools import find_packages, setup
import os 

package_name = 'recognizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/recognizer_model.launch.py']),
        ('share/' + package_name, ['launch/lidar_node.launch.py']),
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*.xacro')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pawelmanka',
    maintainer_email='pawelmanka0@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'recognizer_robot_model = recognizer.recognizer_robot_model:main'
        ],
    },
)
