from setuptools import find_packages, setup

package_name = 'local_mapper_client'

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
    maintainer='wcompton',
    maintainer_email='wcompton@caltech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'seg_prompt_client_node = local_mapper_client.seg_prompt_client_node:main',
            'fake_poly_node = local_mapper_client.fake_poly_node:main',
        ],
    },
)
