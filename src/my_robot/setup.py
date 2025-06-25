from setuptools import setup
import os
from glob import glob

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf'), 
            glob(os.path.join('urdf', '*.*'))),
        (os.path.join('share', package_name, 'meshes'), 
            glob(os.path.join('meshes', '*.*'))),
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='woody.smolko@gmail.com',
    description='My robot package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_control = my_robot.joy_control:main',
            'acker_odom = my_robot.acker_odom:main',
            'rfdetr = my_robot.rfdetr:main'
        ],
    },
)