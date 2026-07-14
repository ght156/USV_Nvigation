# AprilTag船坞定位接口

## tag码配置文件

1. 模块配置文件路径: `{安装路径}/install/apriltag_localization/config/detection_cfg.yml`，配置文件参数及解释如下：
```yaml
# config/my_node_params.yaml
apriltag_node:
  ros__parameters:
    camera_info_topic: "/zed/zed_node/rgb/color/rect/camera_info" # RGB图对应的相机内参topic
    image_topic: "/zed/zed_node/rgb/color/rect/image" # RGB图像topic
    detection_result_topic: "/apriltag_node/dock_pose"  # 输出topic名称，类型std_msgs::msg::Float64MultiArray，相机坐标系下(x,y,z,roll,pitch,yaw)
    frame_id: camera_left_link
    apriltag_family_name: tag36h11 # AprilTag的类型，tag36h11、tag25h9、tagCircle21h7、tagCircle49h12、tagStandard41h12、tagStandard52h13、tagCustom48h12
    tag_size: 0.1 # AprilTag码的实际物理尺寸，单位为m
    tag_ids: [0, 1] # tag码的唯一ID号，int类型，可自定义配置 tag_ids，支持多个，最少配置一个码
    #参数命名必须包含 tag_{tag_id}码配置船坞至二维码的距离/角度参数
    tag_0: # tag id为0的二维码中心至船坞中心的距离/角度参数
      dock_offset_x: 1.0 #单位m
      dock_offset_y: -2.0 #单位m
      dock_offset_z: 0.0 #单位m
      dock_offset_roll: 0.0 #单位:度
      dock_offset_pitch: 0.0 #单位:度
      dock_offset_yaw: 0.0 #单位:度
    tag_1: # tag id为1的的二维码中心至船坞中心的距离/角度参数
      dock_offset_x: 0.0 #单位m
      dock_offset_y: 0.0 #单位m
      dock_offset_z: 0.0 #单位m
      dock_offset_roll: 0.0 #单位:度
      dock_offset_pitch: 0.0 #单位:度
      dock_offset_yaw: 0.0 #单位角:度
```

2. Apriltag船坞定义配置文件使用样例
    >a. 假设船坞设置2个Apriltag码, 在[Aruco & AprilTag Generator](https://tagsgen.top/)中 选择确定个码的`family`和`tag_id`，例如选择 `tag36h11`，2个码ID分别选择`0`和`1`，码大小`0.5m`。
    b. 修改文件中的码信息：
    `apriltag_family_name: tag36h11`
    `tag_size: 0.5`
    `tag_ids:[0,1]`
    c. 每个码至船坞中心距离/角度参数配置，根据实际安装填写:
    `dock_offset_x`、`dock_offset_y`、`dock_offset_z`、`dock_offset_roll`、`dock_offset_pitch`、`dock_offset_yaw`配置
    d. 启动相机驱动节点，发出相机内参和相机RGB图像话题
    e. 启动二维码定位节点：`ros2 launch apriltag_localization apriltag_localization.launch.py`
3. 输出船坞中心结果
话题：`/apriltag_node/dock_pose`  （可通过配置文件修改）
类型`std_msgs::msg::Float64MultiArray`
相机坐标系下`(x,y,z,roll,pitch,yaw)`
xyz单位：米
roll、pitch、yaw单位: 弧度

