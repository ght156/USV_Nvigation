from setuptools import find_packages, setup
import os

package_name = "usv_docking"


def collect_files(dirpath, extensions):
    files = []
    if not os.path.isdir(dirpath):
        return files
    for root, _, filenames in os.walk(dirpath):
        for filename in filenames:
            if filename.startswith(".") or filename.endswith((".pyc", "~")):
                continue
            for ext in extensions:
                if filename.endswith(ext):
                    files.append(os.path.join(root, filename))
                    break
    return files


config_files = collect_files("config", [".yaml"])
launch_files = collect_files("launch", [".launch.py"])
script_files = collect_files("scripts", [".sh"])

data_files = [
    ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
    (f"share/{package_name}", ["package.xml", "README.md"]),
]

if config_files:
    data_files.append((f"share/{package_name}/config", config_files))
if launch_files:
    data_files.append((f"share/{package_name}/launch", launch_files))
if script_files:
    data_files.append((f"share/{package_name}/scripts", script_files))

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="YILDIZ USV",
    maintainer_email="yildiz.usv@outlook.com",
    description="Corridor-gated USV docking controller for differential-drive boats.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "docking_controller = usv_docking.docking_controller:main",
            "docking_pose_estimator_v2 = usv_docking.docking_pose_estimator_v2:main",
            "docking_fsm_v2 = usv_docking.docking_fsm_v2:main",
            "docking_motion_controller_v2 = usv_docking.docking_motion_controller_v2:main",
            "docking_safety_v2 = usv_docking.docking_safety_v2:main",
        ],
    },
)
