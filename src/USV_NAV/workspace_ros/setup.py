from setuptools import find_packages, setup
import os

package_name = 'workspace_ros'

def collect_files(dirpath, extensions):
    files = []
    if not os.path.isdir(dirpath):
        return files
    for root, _, filenames in os.walk(dirpath):
        for filename in filenames:
            for ext in extensions:
                if filename.endswith(ext):
                    files.append(os.path.join(root, filename))
                    break
    return files

config_files = collect_files('config', ['.yaml'])
launch_files = collect_files('launch', ['.launch.py'])
script_files = collect_files('scripts', ['.py'])
yolo_files = collect_files('YOLOv11', ['.pt'])

data_files = [
    ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    ('share/' + package_name, ['package.xml']),
]

if config_files:
    data_files.append(('share/' + package_name + '/config', config_files))
if launch_files:
    data_files.append(('share/' + package_name + '/launch', launch_files))
if script_files:
    data_files.append(('share/' + package_name + '/scripts', script_files))
if yolo_files:
    data_files.append(('share/' + package_name + '/YOLOv11', yolo_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['scripts', 'scripts.*']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='USV_NAV',
    maintainer_email='noreply@usv-nav.local',
    description='ROS 2 package customized for the USV autonomous navigation system.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gnss_odom_map_tf = scripts.gnss_odom_map_tf:main',
            'kamikaze = scripts.kamikaze:main',
            'nav2_cmd_vel_to_mavros = scripts.nav2_cmd_vel_to_mavros:main',
            'static_transform_publisher = scripts.static_transform_publisher:main',
            'target_buoy = scripts.target_buoy:main',
        ],
    },
)