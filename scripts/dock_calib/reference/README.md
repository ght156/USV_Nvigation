# reference/ 说明

`compute_dock_offset_sim.py` 是从 git 历史（commit `4f513d4`）恢复的**仿真**辅助脚本。

- 它按 Gazebo `dock_2022/model.sdf` 的几何计算 `detection_cfg_sim.yml` 的 dock_offset；
- **不适用于实船**，也不读取实船 TF / 相机；
- 仅作为历史参考保留。

实船标定请用上一级的：

```bash
python3 ../dock_offset_calib.py --help
```
