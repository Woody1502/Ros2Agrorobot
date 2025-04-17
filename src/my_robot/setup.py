from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages
package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
          (os.path.join('share', package_name, 'launch'), glob( '*launch.py*')),
  (os.path.join('share', package_name,'urdf'), glob('robot.urdf')),
   (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='woody.smolko@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'state_publisher = my_robot.state_publisher:main'

            
        ],
    },
)
