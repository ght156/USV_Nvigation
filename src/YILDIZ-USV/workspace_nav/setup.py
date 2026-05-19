from setuptools import find_packages, setup
import os

package_name = 'workspace_nav'

def collect_files(dirpath, extensions):
    files = []
    if not os.path.isdir(dirpath):
        return files
    for root, _, filenames in os.walk(dirpath):
        for filename in filenames:
            if filename.startswith('.') or filename.endswith('.pyc') or filename.endswith('~'):
                continue
            for ext in extensions:
                if filename.endswith(ext):
                    files.append(os.path.join(root, filename))
                    break
    return files

config_files = collect_files('config', ['.yaml'])
launch_files = collect_files('launch', ['.launch.py'])
script_files = collect_files('workspace_nav', ['.py'])
map_files = collect_files('map', ['.pgm'])
json_files = collect_files('json', ['.json'])

data_files = [
    ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    (f'share/{package_name}', ['package.xml']),
]

if config_files:
    data_files.append((f'share/{package_name}/config', config_files))
if launch_files:
    data_files.append((f'share/{package_name}/launch', launch_files))
if script_files:
    data_files.append((f'share/{package_name}/scripts', script_files))
if map_files:
    data_files.append((f'share/{package_name}/map', map_files))
if json_files:
    data_files.append((f'share/{package_name}/json', json_files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['workspace_nav', 'workspace_nav.*']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YILDIZ USV',
    maintainer_email='yildiz.usv@outlook.com',
    description='Nav2-based autonomous route planning package customized for the TEKNOFEST Unmanned Surface Vehicle (USV) Competition.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_transform = workspace_nav.waypoint_transform:main',
            'waypoint_with_state = workspace_nav.waypoint_with_state:main',
            'mission_bridge = workspace_nav.mission_bridge:main',
        ],
    },
)