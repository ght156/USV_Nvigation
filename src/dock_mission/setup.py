from setuptools import find_packages, setup
import os

package_name = "dock_mission"


def collect_files(dirpath, extensions):
    files = []
    if not os.path.isdir(dirpath):
        return files
    for root, _, filenames in os.walk(dirpath):
        for filename in filenames:
            if filename.startswith("."):
                continue
            for ext in extensions:
                if filename.endswith(ext):
                    files.append(os.path.join(root, filename))
                    break
    return files


config_files = collect_files("config", [".yaml"])
launch_files = collect_files("launch", [".launch.py"])

data_files = [
    ("share/ament_index/resource_index/packages", [os.path.join("resource", package_name)]),
    (f"share/{package_name}", ["package.xml", "README.md"]),
]
if config_files:
    data_files.append((f"share/{package_name}/config", config_files))
if launch_files:
    data_files.append((f"share/{package_name}/launch", launch_files))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=data_files,
    install_requires=["setuptools"],
    extras_require={
        "test": ["pytest"],
    },
    zip_safe=True,
    maintainer="YILDIZ USV",
    maintainer_email="yildiz.usv@outlook.com",
    description="USV dock mission FSM, entry validator, speed arbitrator (Phase 2 skeleton).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dock_mission_node = dock_mission.dock_mission_node:main",
            "dock_entry_validator = dock_mission.entry_validator_node:main",
            "speed_arbitrator = dock_mission.speed_arbitrator_node:main",
        ],
    },
)
