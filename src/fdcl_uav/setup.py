from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'fdcl_uav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kanishke Gamagedara',
    maintainer_email='kanishkegb@gwu.edu',
    description='FDCL UAV Simulator',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'estimator = fdcl_uav.estimator:main',
            'control = fdcl_uav.control:main',
            'trajectory = fdcl_uav.trajectory:main',
            'gui = fdcl_uav.gui:main',
        ],
    },
)
