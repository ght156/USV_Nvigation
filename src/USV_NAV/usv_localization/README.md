# usv_localization

C++ node `usv_map_odom_tf_node`: publish **map→odom** (and optional **odom→base_link**) from:

1. `map.yaml` **`ref_gnss*`** → map ENU origin  
2. `/gi320/gngga` → GNSS quality (`fix_type` / HDOP)  
3. `/gi320/inspvaa` → fused lat/lon/yaw used as the pose that locks / checks the transform  

## Reset conditions

| Condition | Default |
|-----------|---------|
| GNSS quality drops then recovers | `min_fix_type:=4`, hold `gnss_degrade_hold_s:=2` |
| Map pose vs GNSS ENU mismatch | `map_gnss_mismatch_m:=5` (same order as `rtk_lio_fuse.max_innov_m`) |
| FC `/mavros/local_position/odom` jump | `fc_origin_jump_m:=5` |
| Prolonged IMU-only / poor GNSS | `imu_only_timeout_s:=30` |

## Run

```bash
source install/setup.bash   # or your prefix that has bw_gi320_driver + usv_localization
ros2 launch usv_localization usv_map_odom_tf.launch.py \
  map_config_yaml:=/path/to/map.yaml
```
