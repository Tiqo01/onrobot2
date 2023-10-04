from setuptools import setup

package_name = 'onrobot2_rg_modbus_tcp'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zioud',
    maintainer_email='tariqzioud@gmail.com',
    description='A stack to communicate with OnRobot RG grippers using the Modbus/TCP protocol',
    license='MIT',
    requires=['pymodbus'],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
