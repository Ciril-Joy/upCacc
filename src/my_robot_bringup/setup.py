import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # This is the critical section that needs updating
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # This line installs ALL .launch.py files from the launch directory
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        
        # This line installs the ekf.yaml config file
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        
        # This line installs your URDF file
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='Bringup package for my robot (Pi)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)