#!/usr/bin/env python3
"""
广播 map -> odom：平移由「GNSS 相对地图锚点 ENU」减「MAVROS local_position/odom 平移」得到，
再经 map_odom_yaw_deg 将 ENU 向量与姿态旋入「实际 Nav2 map 轴」（默认 0°）。

地图锚点默认从 Nav2 map 所用 YAML（map_config_yaml）解析，
键 map_origin_ref_key（默认 ref_gnss_10）对应项为 [longitude, latitude]，与 waypoint_transform / map_server 同源。
不设 map_config_yaml、改用手工经纬度时须显式给定 map_origin_latitude/longitude。

不与 MAVROS local_position TF 做任何 NED/ENU 换算；直接使用话题中的量。

订阅必须使用与 MAVROS 常见发布端一致的 QoS（多为 Sensor BEST_EFFORT），否则会因 RELIABILITY 不匹配而永远收不到数据。

若对每一对有效 NavSatFix+Odometry 都重算并发布，map→odom 会随 GNSS/融合噪声抖动（影响 AMCL 等）。
initialize_once:=true（默认）时仅在首次合格数据对上计算变换、深拷贝缓存并加锁，再按 republish_hz
仅重发该缓存（时间戳刷新为当前时刻），之后忽略后续 GPS/odom 对变换的再估计；需持续跟 drift
或在线修正时请设 initialize_once:=false（恢复每次有效对即重算、无锁、无周期重发定时器）。
"""

import copy
import math
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster


WGS84_A = 6378137.0


def lat_lon_from_map_yaml(path: str, ref_key: str) -> tuple:
    """与 map.yaml 中 ref_gnss* 约定一致：[longitude, latitude]（度）。返回 (lat, lon)。"""
    p = Path(path).expanduser()
    try:
        p = p.resolve()
    except Exception:
        pass
    if not p.is_file():
        raise FileNotFoundError(f'地图 YAML 不存在或不可读: {path}')
    with p.open(encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    if not isinstance(cfg, dict):
        raise ValueError('地图 YAML 根节点须为 mapping')
    if ref_key not in cfg:
        raise KeyError(f'地图 YAML 中无键 {ref_key}；请与 Nav2 map_server 所用文件一致并含 ref_gnss*')
    arr = cfg[ref_key]
    if not isinstance(arr, (list, tuple)) or len(arr) < 2:
        raise ValueError(f'{ref_key} 须为 [longitude, latitude] 列表')
    lon = float(arr[0])
    lat = float(arr[1])
    return lat, lon


def latlon_to_enu_east_north(lat_deg: float, lon_deg: float, lat0_deg: float, lon0_deg: float):
    """小范围近似：相对 (lat0,lon0) 的东向、北向位移 (m)。"""

    dlat = math.radians(lat_deg - lat0_deg)
    dlon = math.radians(lon_deg - lon0_deg)
    north = dlat * WGS84_A
    east = dlon * WGS84_A * math.cos(math.radians(lat0_deg))
    return east, north


class GnssOdomMapTf(Node):
    def __init__(self):
        super().__init__('gnss_odom_map_tf')
        try:
            self.declare_parameter('use_sim_time', False)
        except rclpy.exceptions.ParameterAlreadyDeclaredException:
            pass

        self.declare_parameter('global_topic', '/mavros/global_position/global')
        self.declare_parameter('local_odom_topic', '/mavros/local_position/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        # 与 Nav2 map_server 同一份 YAML；非空则从中读 map_origin_ref_key，忽略下面手工经纬度。
        self.declare_parameter('map_config_yaml', '')
        # 对应 map 世界系 (0,0) 的 GNSS 角点（与制图 ref 命名一致）；默认与旧 launch 常量 ref_gnss_10 对齐。
        self.declare_parameter('map_origin_ref_key', 'ref_gnss_10')
        self.declare_parameter('map_origin_latitude', 0.0)
        self.declare_parameter('map_origin_longitude', 0.0)
        # 实测 MAVROS/FMU 时间与 ROS 「当前时刻」常有偏差，或过小的 age 会令首帧永远不通过。
        # 默认 0=不做「相对 now」新鲜度校验；需要紧缩时再设为正数（秒）。
        self.declare_parameter('max_data_age_sec', 0.0)
        self.declare_parameter('initialize_once', True)
        self.declare_parameter('republish_hz', 20.0)
        # 地图平面与 ENU 的固定绕 z 偏角（度）。+ 为绕 map 的 +z 逆时针（俯视）。
        self.declare_parameter('map_odom_yaw_deg', 0.0)

        self._map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self._odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value

        yaml_path = self.get_parameter('map_config_yaml').get_parameter_value().string_value.strip()
        ref_key = self.get_parameter('map_origin_ref_key').get_parameter_value().string_value.strip()
        ref_key = ref_key or 'ref_gnss_10'
        manual_lat = self.get_parameter('map_origin_latitude').get_parameter_value().double_value
        manual_lon = self.get_parameter('map_origin_longitude').get_parameter_value().double_value

        self._datum_source = ''
        if yaml_path:
            self._lat0, self._lon0 = lat_lon_from_map_yaml(yaml_path, ref_key)
            self._datum_source = f'{yaml_path}[{ref_key}]'
            self.get_logger().info(f'已从地图配置读取锚点 {ref_key} → ({self._lat0:.8f}, {self._lon0:.8f}) °')
        else:
            self._lat0, self._lon0 = manual_lat, manual_lon
            self._datum_source = 'manual'
            if manual_lat == 0.0 and manual_lon == 0.0:
                self.get_logger().fatal(
                    '未设置 map_config_yaml，且 map_origin_latitude/longitude 均为 0。'
                    '请传入与 map_server 相同的 YAML，或改为手工纬度/经度参数。')
                raise ValueError('gnss_odom_map_tf: missing map datum')
            self.get_logger().warn(
                '未使用 map_config_yaml：正用手工 map_origin_latitude/longitude（请确认与栅格原点一致）。')
        self._max_age = self.get_parameter('max_data_age_sec').get_parameter_value().double_value
        self._initialize_once = self.get_parameter('initialize_once').get_parameter_value().bool_value
        self._republish_hz = self.get_parameter('republish_hz').get_parameter_value().double_value
        self._yaw_deg = self.get_parameter('map_odom_yaw_deg').get_parameter_value().double_value

        self._locked = False
        self._cached_transform = None
        self._republish_timer = None

        gt = self.get_parameter('global_topic').get_parameter_value().string_value
        lo = self.get_parameter('local_odom_topic').get_parameter_value().string_value

        self._last_fix = None  # NavSatFix
        self._last_odom = None  # Odometry

        self._broadcaster = TransformBroadcaster(self)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.create_subscription(NavSatFix, gt, self._on_fix, sensor_qos)
        self.create_subscription(Odometry, lo, self._on_odom, sensor_qos)

        self.get_logger().info(
            'Subscribed with BEST_EFFORT QoS (match typical MAVROS sensor streams).'
        )
        self._skip_age_logged = False

        self.get_logger().info(
            f'gnss_odom_map_tf: datum={self._datum_source} '
            f'lon_lat=({self._lon0:.8f},{self._lat0:.8f})°, '
            f'map={self._map_frame} odom={self._odom_frame}, '
            f'initialize_once={self._initialize_once}, republish_hz={self._republish_hz}, '
            f'map_odom_yaw_deg={self._yaw_deg}'
        )

    @staticmethod
    def _stamp_is_nonzero(stamp_msg) -> bool:
        return bool(stamp_msg.sec != 0 or stamp_msg.nanosec != 0)

    def _on_fix(self, msg: NavSatFix):
        if self._initialize_once and self._locked:
            return
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return
        self._last_fix = msg
        self._try_publish()

    def _on_odom(self, msg: Odometry):
        if self._initialize_once and self._locked:
            return
        self._last_odom = msg
        self._try_publish()

    def _republish_cached(self):
        if self._cached_transform is None:
            return
        t = copy.deepcopy(self._cached_transform)
        t.header.stamp = self.get_clock().now().to_msg()
        self._broadcaster.sendTransform(t)

    def _try_publish(self):
        if self._initialize_once and self._locked:
            return

        if self._last_fix is None or self._last_odom is None:
            return

        now = self.get_clock().now()
        if self._max_age > 0.0:
            t_fix = Time.from_msg(self._last_fix.header.stamp)
            t_odom = Time.from_msg(self._last_odom.header.stamp)
            # 任一 stamp 为 0（常见 MAVROS/FMU）：不用「now − stamp」判断是否过期，否则会永远连不上 TF
            fix_z = self._stamp_is_nonzero(self._last_fix.header.stamp)
            o_z = self._stamp_is_nonzero(self._last_odom.header.stamp)
            if fix_z and (now - t_fix).nanoseconds * 1e-9 > self._max_age:
                self._warn_skip_age_once('fix_vs_now')
                return
            if o_z and (now - t_odom).nanoseconds * 1e-9 > self._max_age:
                self._warn_skip_age_once('odom_vs_now')
                return

        t_fix_msg = Time.from_msg(self._last_fix.header.stamp)
        t_odom_msg = Time.from_msg(self._last_odom.header.stamp)
        stamp_out = (
            self._last_fix.header.stamp
            if t_fix_msg.nanoseconds >= t_odom_msg.nanoseconds
            else self._last_odom.header.stamp
        )
        if not (
            self._stamp_is_nonzero(self._last_fix.header.stamp)
            and self._stamp_is_nonzero(self._last_odom.header.stamp)
        ):
            stamp_out = self.get_clock().now().to_msg()

        lat = self._last_fix.latitude
        lon = self._last_fix.longitude
        east, north = latlon_to_enu_east_north(lat, lon, self._lat0, self._lon0)
        pox = self._last_odom.pose.pose.position.x
        poy = self._last_odom.pose.pose.position.y

        dx = east - pox
        dy = north - poy
        yaw = math.radians(self._yaw_deg)
        c = math.cos(yaw)
        s = math.sin(yaw)
        rx = c * dx - s * dy
        ry = s * dx + c * dy
        half = 0.5 * yaw
        qz = math.sin(half)
        qw = math.cos(half)

        t = TransformStamped()
        t.header.stamp = stamp_out
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._odom_frame
        t.transform.translation.x = rx
        t.transform.translation.y = ry
        # 平面 Nav2：map->odom 不做竖直耦合（GNSS 高程标定单独约定）
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        if self._initialize_once:
            self._cached_transform = copy.deepcopy(t)
            self._locked = True
            self._broadcaster.sendTransform(t)
            eff_hz = self._republish_hz if self._republish_hz > 0.0 else 10.0
            if self._republish_hz <= 0.0:
                self.get_logger().warn(
                    'republish_hz<=0：将按 10Hz 周期性重发 map→odom，否则 TF 会过期断开'
                )
            if self._republish_timer is None:
                self._republish_timer = self.create_timer(
                    1.0 / eff_hz, self._republish_cached)
        else:
            self._broadcaster.sendTransform(t)

    def _warn_skip_age_once(self, reason: str):
        if self._skip_age_logged:
            return
        self._skip_age_logged = True
        self.get_logger().warn(
            f'暂不发布 map→odom（{reason}）；max_data_age_sec={self._max_age} 与时间戳不匹配。'
            ' 可尝试 max_data_age_sec:=0；或校对 use_sim_time / MAVROS header.stamp。'
        )


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GnssOdomMapTf()
    except ValueError:
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())
