from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'behavior_executive'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='admin',
    maintainer_email='admin@todo.todo',
    description=(
        'Behavior executive layer between the VLM (target selection) and '
        'Nav2: a fixed library of navigation behaviors run as an explicit '
        'task tree with on_failure/on_exhausted recovery chains.'
    ),
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_executor_node = behavior_executive.behavior_executor_node:main',
        ],
    },
)
