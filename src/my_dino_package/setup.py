from setuptools import setup
import os
from glob import glob

package_name = 'my_dino_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='admin@todo.todo',
    description='Semantic 3D reconstruction',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dino_nvblox_node = my_dino_package.dino_node:main',
            'semantic_to_nvblox_bridge = my_dino_package.semantic_to_nvblox_bridge:main',
        ],
    },
)