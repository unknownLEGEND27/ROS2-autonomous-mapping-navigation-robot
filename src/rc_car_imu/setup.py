from setuptools import find_packages, setup

package_name = 'rc_car_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/imu_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Preyansh',
    maintainer_email='preyansh@todo.com',
    description='MPU6500/9250 IMU driver node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'imu_node = rc_car_imu.imu_node:main',
        ],
    },
)
