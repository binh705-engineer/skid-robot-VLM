import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'reasoning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Cai dat toan bo config/*.yaml vao share/reasoning/config/
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # Cai dat toan bo launch/*.launch.py vao share/reasoning/launch/
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='haibotlab',
    maintainer_email='haibotlab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'vlm_client_node = reasoning.vlm_client_node:main',
            'trigger_input_node = reasoning.trigger_input_node:main',
            'coordinate_mapper_node = reasoning.coordinate_mapper_node:main',
            #Behaviors:
            'vlm_client_behaviors_node = reasoning.vlm_client_behaviors_node:main',
        ],
    },
)
