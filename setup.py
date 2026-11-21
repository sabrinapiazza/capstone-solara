from glob import glob
from setuptools import setup

package_name = 'solara_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sabrina Piazza',
    description='Solara: room-scale autonomous rover (lidar SLAM + Nav2)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_node = solara_pkg.base_node:main',
        ],
    },
)
