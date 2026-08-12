import os
from setuptools import find_packages, setup

package_name = 'description'

# Hàm "Bùa chú" giúp tự động chui vào từng ngóc ngách của thư mục meshes và urdf để copy không rớt 1 file nào
def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # Gọi hàm quét đệ quy ở đây để tóm gọn 2 thư mục 'urdf' và 'meshes'
    data_files=package_files([
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ], ['urdf', 'meshes']),
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='binh',
    maintainer_email='binh85980344@gmail.com',
    description='Package chứa mô hình URDF và Meshes của robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)

