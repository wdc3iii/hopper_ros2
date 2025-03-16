from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'local_mapper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Install launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noelcs',
    maintainer_email='noelcs@caltech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_mapper_node = local_mapper.local_mapper_node:main'
        ],
    },
)
