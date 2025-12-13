from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ex_assignment1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shady',
    maintainer_email='shadyrafat60@gmail.com',
    description='Simulation assignment package',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'observer = ex_assignment1.observer:main',
            'action_client = ex_assignment1.action_client:main',
            'action_server = ex_assignment1.action_server:main',
            'searcher = ex_assignment1.searcher:main',
            'image_modifier = ex_assignment1.image_modifier:main',
            ],
    },
)
