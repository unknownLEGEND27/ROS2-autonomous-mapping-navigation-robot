import os
from glob import glob
from setuptools import find_packages, setup
package_name = 'rc_car_bringup'
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Preyansh', maintainer_email='preyansh@todo.com',
    description='Launch files for RC car robot', license='MIT',
    entry_points={'console_scripts': []},
)
