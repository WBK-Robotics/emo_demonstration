import os
from glob import glob
from setuptools import setup

package_name = 'emo_demonstration'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]


def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:

        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('lib', 'python3.10','site-packages', path)

                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)

                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

data_files = package_files(data_files,[os.path.join('emo_demonstration','urdf')])
data_files.append((os.path.join('share', package_name), glob('launch/*launch.xml')))
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools','pybullet','pybullet_industrial'],
    zip_safe=True,
    maintainer='sdmbot',
    maintainer_email='jan-baumgaertner@web.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_publisher = emo_demonstration.ur5_trajectory_publisher:main',
            'board_calibration = emo_demonstration.board_calibration:main'
        ],
    },
)
