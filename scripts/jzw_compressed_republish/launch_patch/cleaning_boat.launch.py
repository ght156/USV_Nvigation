# Copyright 2026
# SPDX-License-Identifier: Apache-2.0
#
# 清扫船感知双容器：AI 检测链与 AprilTag 定位链数据源不同，各自独立容器
#
#   容器 A cleaning_boat_ai_container（component_container_mt）：
#     - orbbec_camera::OBCameraNodeDriver ×N   → Gemini 330 相机（front/back,各配一个参数 YAML）
#                                                 参数声明与加载逻辑完全共用官方
#                                                 orbbec_camera/launch/gemini_330_series.launch.py
#                                                 （官方 247 参数为共同默认池,分辨率等差异项放各自 YAML）
#     - astra_camera::OBCameraNodeFactory      → /camera_front_astra/rgbd（RGBD 3D 检测输入）
#                                                 已注释停用,数据源切换为 Gemini 330,取消注释即可恢复
#     - ai_detector::OverlayControllerNode     → 叠加开关服务 /ai_detector/overlay_switch
#                                                 + 状态广播 /ai_detector/overlay_switch_status
#     - ai_detector::AiDetectorNode            → 双通道：海康 RTSP overlay + RGBD trash 3D
#     - rtsp2::Rtsp2MultiSourceNode            → 订 /ai_detector/cam_front_hik/overlay_image 推流
#
#   容器 B cleaning_boat_apriltag_container（component_container_mt）：
#     - video_camera::VideoCameraNode          → /camera_back_video/color/**（后视摄像头）
#     - perception::AprilTagLocalizationNode   → /apriltag_node/detections（码头/船坞检测）
#
#   ros2 launch perception_launch cleaning_boat.launch.py
#   ros2 launch perception_launch cleaning_boat.launch.py ai_detector_config_file:=/path/to/x.yaml
#   ros2 launch perception_launch cleaning_boat.launch.py use_intra_process_comms:=false
#   Gemini 330 参数与官方 gemini_330_series.launch.py 完全同名共用,常用示例：
#   ros2 launch perception_launch cleaning_boat.launch.py camera_name:=camera_front_gemini
#   ros2 launch perception_launch cleaning_boat.launch.py serial_number:=CL9xxxxxxxx
#   ros2 launch perception_launch cleaning_boat.launch.py config_file_path:=/path/to/params.yaml
#   ros2 launch perception_launch cleaning_boat.launch.py load_config_json_file_path:=/path/to/OrbbecSDKConfig.json
#   两台 Gemini 330 并存（官方 247 参数为共同默认,差异项放各自 YAML,空路径=不启用该相机）：
#   ros2 launch perception_launch cleaning_boat.launch.py \
#       front_params_file:=/path/to/front.yaml back_params_file:=/path/to/back.yaml

import importlib.util
import os
import sys

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _truthy(raw: str) -> bool:
    return raw.lower() in ('true', '1', 'yes')


# 后端相机压缩转发：主机 rviz 改订该压缩话题，避免裸图(1920x1080~62MB/s)跨网抢带宽
# （脚本见 /scripts/jzw_compressed_republish/compressed_republish.py）
_COMPRESSED_SCRIPT = os.path.expanduser(
    '~/jzw_ws/scripts/jzw_compressed_republish/compressed_republish.py')


# Gemini 330(orbbec_camera):动态加载官方 gemini_330_series.launch.py,
# 完全共用其参数声明与 load_parameters 加载逻辑,官方文件一字不改
_OB_LAUNCH = os.path.join(
    get_package_share_directory('orbbec_camera'), 'launch', 'gemini_330_series.launch.py')
if not os.path.isfile(_OB_LAUNCH):
    raise FileNotFoundError(
        f'未找到 orbbec_camera 官方 launch:{_OB_LAUNCH}。请先构建 orbbec_camera 包。')
_OB_SPEC = importlib.util.spec_from_file_location('orbbec_gemini_330', _OB_LAUNCH)
assert _OB_SPEC is not None and _OB_SPEC.loader is not None
_OB_MOD = importlib.util.module_from_spec(_OB_SPEC)
_OB_SPEC.loader.exec_module(_OB_MOD)


def _orbbec_launch_arguments():
    """共用官方 launch 的全部参数声明(约 200 个),仅替换 camera_name 默认值。"""
    ob_args = []
    for entity in _OB_MOD.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            if entity.name == 'camera_name':
                entity = DeclareLaunchArgument(
                    'camera_name',
                    default_value='camera_front_gemini',
                    description='Gemini 330 命名空间与 TF 帧前缀（覆盖官方默认 camera）',
                )
            ob_args.append(entity)
    return ob_args


_ORBBEC_ARGS = _orbbec_launch_arguments()


def _load_astra_params(params_file: str):
    """读取 dabai_dcw YAML；camera_namespace 仅给 launch 用，不传给节点。"""
    with open(params_file, 'r', encoding='utf-8') as f:
        config_params = yaml.safe_load(f) or {}
    camera_namespace = config_params.pop('camera_namespace', 'camera_front_astra')
    return camera_namespace, config_params


def _launch_setup(context, *args, **kwargs):
    astra_share = get_package_share_directory('astra_camera')
    ai_share = get_package_share_directory('ai_detector')
    rtsp_share = get_package_share_directory('rtsp2')
    video_share = get_package_share_directory('video_camera')
    apriltag_prefix = get_package_prefix('apriltag_localization')

    astra_params_file = LaunchConfiguration('astra_params_file').perform(context).strip()
    if not astra_params_file:
        astra_params_file = os.path.join(astra_share, 'params', 'dabai_dcw_params.yaml')
    astra_ns, astra_params = _load_astra_params(astra_params_file)

    ai_cfg = LaunchConfiguration('ai_detector_config_file').perform(context).strip()
    if not ai_cfg:
        ai_cfg = os.path.join(ai_share, 'config', 'hikvision_overlay_rgbd.yaml')

    rtsp_cfg = LaunchConfiguration('rtsp2_config_file').perform(context).strip()
    if not rtsp_cfg:
        rtsp_cfg = os.path.join(rtsp_share, 'config', 'rtsp_overlay', 'rtsp_overlay.yaml')

    video_cfg = LaunchConfiguration('video_camera_params_file').perform(context).strip()
    if not video_cfg:
        video_cfg = os.path.join(video_share, 'config', 'params.yaml')

    attitude_cfg = LaunchConfiguration('attitude_params_file').perform(context).strip()
    if not attitude_cfg:
        attitude_cfg = os.path.join(
            get_package_share_directory('attitude_provider'), 'config', 'attitude_provider.yaml')

    apriltag_cfg = LaunchConfiguration('apriltag_params_file').perform(context).strip()
    if not apriltag_cfg:
        apriltag_cfg = os.path.join(apriltag_prefix, 'config', 'detection_cfg.yml')

    use_intra = _truthy(LaunchConfiguration('use_intra_process_comms').perform(context))
    intra = [{'use_intra_process_comms': True}] if use_intra else []

    # Gemini 330(orbbec_camera):官方 247 参数作为所有相机的共同默认池,
    # 每相机差异项(分辨率/帧率/序列号/camera_name 等)放各自 YAML,覆盖全局默认
    # (front_params_file / back_params_file 为空=不启用该相机)
    ob_base = _OB_MOD.load_parameters(context, _ORBBEC_ARGS)
    ob_nodes = []
    for slot in ('front', 'back'):
        params_file = LaunchConfiguration(f'{slot}_params_file').perform(context).strip()
        if not params_file:
            continue
        with open(params_file, 'r', encoding='utf-8') as f:
            yaml_params = yaml.safe_load(f) or {}
        params = dict(ob_base)
        params.update(yaml_params)
        ob_nodes.append(ComposableNode(
            package='orbbec_camera',
            plugin='orbbec_camera::OBCameraNodeDriver',
            name=f'orbbec_camera_{slot}',
            namespace=params.get('camera_name') or 'camera',
            parameters=[params],
            extra_arguments=intra,
        ))

    ai_params = [{
        'config_file': ai_cfg,
        'overlay_enabled_default': LaunchConfiguration('overlay_enabled_default'),
    }]
    controller_params = [{
        'overlay_enabled_default': LaunchConfiguration('overlay_enabled_default'),
    }]

    ai_container = ComposableNodeContainer(
        name='cleaning_boat_ai_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            # *ob_nodes,
            # astra(dabai_dcw)已停用:前端 RGBD 数据源切换为 Gemini 330,取消注释即可恢复
            # ComposableNode(
            #     package='astra_camera',
            #     plugin='astra_camera::OBCameraNodeFactory',
            #     name='astra_camera_front',
            #     namespace=astra_ns,
            #     parameters=[astra_params],
            #     extra_arguments=intra,
            # ),
            ComposableNode(
                package='ai_detector',
                plugin='ai_detector::OverlayControllerNode',
                name='overlay_controller',
                parameters=controller_params,
                extra_arguments=intra,
            ),
            ComposableNode(
                package='ai_detector',
                plugin='ai_detector::AiDetectorNode',
                name='ai_detector',
                parameters=ai_params,
                extra_arguments=intra,
            ),
            # 中立姿态提供者：GI320(主)/相机 IMU(回退) → /perception/attitude（相机光系姿态）
            ComposableNode(
                package='attitude_provider',
                plugin='attitude_provider::AttitudeProviderNode',
                name='attitude_provider',
                parameters=[attitude_cfg],
                extra_arguments=intra,
            ),
            ComposableNode(
                package='rtsp2',
                plugin='rtsp2::Rtsp2MultiSourceNode',
                name='rtsp2_multi_source_node',
                parameters=[rtsp_cfg],
                extra_arguments=intra,
            ),
        ],
        output='screen',
    )

    apriltag_container = ComposableNodeContainer(
        name='cleaning_boat_apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='video_camera',
                plugin='video_camera::VideoCameraNode',
                name='video_camera_node',
                parameters=[video_cfg],
                extra_arguments=intra,
            ),
            ComposableNode(
                package='apriltag_localization',
                plugin='perception::AprilTagLocalizationNode',
                name='apriltag_node',
                parameters=[apriltag_cfg],
                extra_arguments=intra,
            ),
        ],
        output='screen',
    )

    # ── 后端相机压缩转发（独立进程，与 apriltag 容器并行）──
    # 主机端 rviz 订 /camera_back_video/color/image_raw/compressed（JPEG），
    # 不再订裸图 /camera_back_video/color/image_raw，避免图传带宽被抽空。
    compressed_node = ExecuteProcess(
        cmd=[
            sys.executable, _COMPRESSED_SCRIPT,
            '--ros-args',
            '-p', 'in_topic:=' + LaunchConfiguration('compressed_in_topic').perform(context),
            '-p', 'out_topic:=' + LaunchConfiguration('compressed_out_topic').perform(context),
            '-p', 'scale:=' + LaunchConfiguration('compressed_scale').perform(context),
            '-p', 'jpeg_quality:=' + LaunchConfiguration('compressed_quality').perform(context),
            '-p', 'max_fps:=' + LaunchConfiguration('compressed_max_fps').perform(context),
        ],
        name='compressed_republish',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )
    # return [ai_container]
    return [apriltag_container, compressed_node]


def generate_launch_description():
    astra_share = get_package_share_directory('astra_camera')
    ai_share = get_package_share_directory('ai_detector')
    rtsp_share = get_package_share_directory('rtsp2')
    default_astra = os.path.join(astra_share, 'params', 'dabai_dcw_params.yaml')
    default_ai = os.path.join(ai_share, 'config', 'hikvision_overlay_rgbd.yaml')
    default_rtsp = os.path.join(rtsp_share, 'config', 'rtsp_overlay', 'rtsp_overlay.yaml')

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Unused; accepted for IncludeLaunchDescription from usv_app',
            ),
            DeclareLaunchArgument(
                'astra_params_file',
                default_value=default_astra,
                description='astra_camera dabai_dcw 参数 YAML（含 camera_namespace）',
            ),
            DeclareLaunchArgument(
                'ai_detector_config_file',
                default_value=default_ai,
                description='ai_detector 通道配置（默认 hikvision_overlay_rgbd.yaml：海康 overlay + RGBD 3D）',
            ),
            DeclareLaunchArgument(
                'rtsp2_config_file',
                default_value=default_rtsp,
                description='rtsp2 推流 YAML（默认 config/rtsp_overlay/rtsp_overlay.yaml）',
            ),
            DeclareLaunchArgument(
                'overlay_enabled_default',
                default_value='false',
                description='叠加开关初始状态（overlay_controller 与 ai_detector 共用）',
            ),
            DeclareLaunchArgument(
                'use_intra_process_comms',
                default_value='true',
                description='true 时为容器内 ComposableNode 启用进程内通信（零拷贝）',
            ),
            DeclareLaunchArgument(
                'video_camera_params_file',
                default_value=os.path.join(
                    get_package_share_directory('video_camera'), 'config', 'params.yaml'),
                description='video_camera（后视摄像头）参数 YAML',
            ),
            DeclareLaunchArgument(
                'apriltag_params_file',
                default_value=os.path.join(
                    get_package_prefix('apriltag_localization'), 'config', 'detection_cfg.yml'),
                description='apriltag_localization 参数 YAML（默认 detection_cfg.yml）',
            ),
            DeclareLaunchArgument(
                'attitude_params_file',
                default_value=os.path.join(
                    get_package_share_directory('attitude_provider'),
                    'config', 'attitude_provider.yaml'),
                description='attitude_provider 参数 YAML（姿态源/超时/滤波/帧名）',
            ),
            DeclareLaunchArgument(
                'front_params_file',
                default_value=os.path.join(
                    get_package_share_directory('orbbec_camera'),
                    'config', 'cleaning_boat', 'orbbec_gemini330_front.yaml'),
                description='前端 Gemini 330 参数 YAML（覆盖官方全局默认；空=不启用该相机）',
            ),
            DeclareLaunchArgument(
                'back_params_file',
                default_value='',
                description='后置 Gemini 330 参数 YAML（空=不启用；两台并存时各自填写 serial_number）',
            ),
            DeclareLaunchArgument(
                'compressed_in_topic',
                default_value='/camera_back_video/color/image_raw',
                description='压缩转发输入（后端相机裸图）',
            ),
            DeclareLaunchArgument(
                'compressed_out_topic',
                default_value='/camera_back_video/color/image_raw/compressed',
                description='压缩转发输出（主机 rviz 订阅此话题）',
            ),
            DeclareLaunchArgument(
                'compressed_scale',
                default_value='0.5',
                description='压缩转发缩放因子（0.5=960x540，省带宽；1.0=原始尺寸）',
            ),
            DeclareLaunchArgument(
                'compressed_quality',
                default_value='80',
                description='JPEG 质量 1-100',
            ),
            DeclareLaunchArgument(
                'compressed_max_fps',
                default_value='10.0',
                description='压缩转发限帧率（0=不限）',
            ),
        ]
        # 官方 gemini_330_series.launch.py 的全部参数声明(约 200 个,必须位于 OpaqueFunction 之前)
        + _ORBBEC_ARGS
        + [
            OpaqueFunction(function=_launch_setup),
        ]
    )
