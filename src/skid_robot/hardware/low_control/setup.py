import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'low_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ĐÃ SỬA: Tự động quét và copy toàn bộ các file .launch.py trong thư mục launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='binh',
    maintainer_email='binh85980344@gmail.com',
    description='Package for controlling low-level robot functions',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematic = low_control.kinematic:main',
        ],
    },
)
