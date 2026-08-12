import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ekf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Sử dụng glob luôn cho cả thư mục launch nếu sau này bạn có nhiều file launch
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # Đã cấu hình glob để quét toàn bộ file cấu hình YAML
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='binh',
    maintainer_email='binh85980344@gmail.com',
    description='ROS 2 EKF Localization Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ekf.minimal_publisher:main',
            'listener = ekf.minimal_subscriber:main',
        ],
    },
)
