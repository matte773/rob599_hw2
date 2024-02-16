import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rob599_hw2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matt',
    maintainer_email='millema5@oregonstate.edu',
    description='ROB599 ROS 2 Homework 2',
    license='BSD 3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speedlimiter = rob599_hw2.speedlimiter:main', 
            'twistpublisher = rob599_hw2.twistpublisher:main',  
            'twistchecker = rob599_hw2.twistchecker:main',
        ],
    },
)
