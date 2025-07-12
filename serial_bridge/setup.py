from setuptools import find_packages, setup

package_name = 'serial_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ifan',
    maintainer_email='syedakil.nafi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'serial_bridge_node = serial_bridge.serial_bridge_node:main',
                'serial_v2 = serial_bridge.serial_bridge_v2:main',
                'serial_v3 = serial_bridge.serial_v3:main',
                'serial_arm = serial_bridge.serial_arm:main',
                'serial_mod = serial_bridge.serial_mod:main',
        ],
    },
)
