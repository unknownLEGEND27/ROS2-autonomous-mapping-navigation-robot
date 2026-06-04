from setuptools import find_packages, setup

package_name = 'rc_car_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/pid_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Preyansh',
    maintainer_email='preyansh@todo.com',
    description='PID heading controller using IMU feedback',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pid_node = rc_car_control.pid_node:main',
        ],
    },
)
