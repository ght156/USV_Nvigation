#!/usr/bin/env python3

import importlib.util
from pathlib import Path


def load_nav2_launch():
    path = Path(__file__).resolve().parents[1] / "launch" / "nav2_boat.launch.py"
    spec = importlib.util.spec_from_file_location("nav2_boat_logging_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_native_ros_logs_use_usv_home_and_keep_ten_startup_dirs(tmp_path, monkeypatch):
    monkeypatch.setenv("USV_HOME", str(tmp_path))
    launch = load_nav2_launch()
    paths = [launch._configure_native_ros_logging() for _ in range(12)]

    native_root = tmp_path / "logs" / "navigation" / "ros"
    assert all(Path(path).is_relative_to(native_root) for path in paths)
    startup_dirs = [path for path in native_root.iterdir() if path.is_dir()]
    assert len(startup_dirs) == 10
    assert all((path / ".navigation_ros_startup").is_file() for path in startup_dirs)
    assert Path(paths[-1]).is_dir()
    assert Path(paths[-1]).parent == native_root
