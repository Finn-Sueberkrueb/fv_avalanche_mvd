from setuptools import find_packages, setup
import subprocess, os, platform
from glob import glob


package_name = 'fv_avalanche_mvd'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz'))  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='parallels@todo.todo',
    description='Minimal demonstration to find a simple w-lan radio transmitter.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'search_sequence = fv_avalanche_mvd.search_sequence:main',
            'signal_simulation = fv_avalanche_mvd.signal_simulation:main',
            'transponder_localization = fv_avalanche_mvd.transponder_localization:main',
        ],
    },
)
