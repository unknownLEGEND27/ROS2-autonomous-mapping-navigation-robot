from setuptools import find_packages, setup

package_name = 'rc_car_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/motor_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Preyansh',
    maintainer_email='preyansh@todo.com',
    description='L298N motor driver node for 4WD RC car',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_node = rc_car_driver.motor_node:main',
        ],
    },
)
