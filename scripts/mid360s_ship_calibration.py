#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Livox Mid-360S 船载安装角标定与点云校正
========================================

适用条件：
1. Livox Mid-360S
2. LiDAR X轴与船体纵轴基本对齐
3. LiDAR主要绕Y轴倾斜安装
4. ROS2 Humble + livox_ros_driver2
5. 标定阶段船舶尽量保持静止

功能：
1. 订阅 Mid-360S IMU
2. 静止采集指定时间
3. 根据重力估计 Roll / Pitch
4. 主要提取绕Y轴安装倾角 Pitch
5. 自动生成反向校正角
6. 订阅 PointCloud2
7. 对点云绕Y轴进行实时旋转
8. 发布校正后的 PointCloud2
9. 保存标定结果到 txt 文件

注意：
IMU测得的是 LiDAR 相对于重力方向的姿态。
如果船体本身存在纵倾，则：
    IMU Pitch = 安装Pitch + 船体Pitch
因此最终外参最好结合点云几何进行验证。
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2


class Mid360SShipCalibration(Node):

    def __init__(self):
        super().__init__('mid360s_ship_calibration')

        # =========================================================
        # 用户参数
        # =========================================================

        # 根据 ros2 topic list 修改
        self.imu_topic = '/livox/imu'
        self.cloud_topic = '/livox/lidar'

        # 校正后的点云
        self.corrected_cloud_topic = '/livox/lidar_corrected'

        # IMU静态采集时间
        self.calibration_time = 30.0

        # 是否在标定完成后自动使用IMU结果校正点云
        # （roll / pitch 一起校正；yaw 重力不可观测量，只能手动）
        self.auto_correct = True

        # 如果你已经知道角度，可以设置为 False，
        # 然后手动输入校正角
        self.manual_correction_pitch_deg = 0.0
        self.manual_correction_roll_deg = 0.0
        # yaw 只能从外部基准获得（艏向传感器/岸线几何/手动测量），
        # 自动模式下也会叠加该手动值
        self.manual_correction_yaw_deg = 0.0

        # 保存结果
        self.result_file = 'mid360s_calibration_result.txt'

        # =========================================================
        # 内部变量
        # =========================================================

        self.acc_data = []

        self.calibration_finished = False

        self.estimated_roll_deg = 0.0
        self.estimated_pitch_deg = 0.0

        self.correction_pitch_deg = 0.0
        self.correction_roll_deg = 0.0
        self.correction_yaw_deg = 0.0

        self.start_time = self.get_clock().now()

        # =========================================================
        # IMU Subscriber
        # =========================================================

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            200
        )

        # =========================================================
        # PointCloud Subscriber
        # =========================================================

        self.cloud_sub = self.create_subscription(
            PointCloud2,
            self.cloud_topic,
            self.cloud_callback,
            10
        )

        # =========================================================
        # Corrected PointCloud Publisher
        # =========================================================

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            self.corrected_cloud_topic,
            10
        )

        # =========================================================
        # Timer
        # =========================================================

        self.timer = self.create_timer(
            0.5,
            self.check_calibration
        )

        print()
        print("=" * 65)
        print(" Livox Mid-360S 船载安装角标定")
        print("=" * 65)
        print(f" IMU Topic       : {self.imu_topic}")
        print(f" PointCloud Topic: {self.cloud_topic}")
        print(f" Output Topic    : {self.corrected_cloud_topic}")
        print(f" 标定时间        : {self.calibration_time:.1f} s")
        print()
        print(" 请保持船体尽量静止...")
        print("=" * 65)

    # =============================================================
    # IMU
    # =============================================================

    def imu_callback(self, msg):

        if self.calibration_finished:
            return

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        if not all(np.isfinite([ax, ay, az])):
            return

        self.acc_data.append([ax, ay, az])

    # =============================================================
    # 检查标定时间
    # =============================================================

    def check_calibration(self):

        if self.calibration_finished:
            return

        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        remaining = self.calibration_time - elapsed

        if remaining > 0:

            print(
                f"\rIMU采集中... 剩余 {remaining:5.1f} s "
                f"样本数 = {len(self.acc_data)}",
                end='',
                flush=True
            )

            return

        print()

        if len(self.acc_data) < 100:

            self.get_logger().error(
                "IMU数据不足，请检查 /livox/imu"
            )

            return

        self.calculate_installation_angle()

    # =============================================================
    # 计算安装角
    # =============================================================

    def calculate_installation_angle(self):

        data = np.asarray(
            self.acc_data,
            dtype=np.float64
        )

        # ---------------------------------------------------------
        # 1. 基于中位数进行简单异常值剔除
        # ---------------------------------------------------------

        median = np.median(data, axis=0)

        distance = np.linalg.norm(
            data - median,
            axis=1
        )

        threshold = np.percentile(
            distance,
            95
        )

        valid_data = data[
            distance <= threshold
        ]

        # ---------------------------------------------------------
        # 2. 求平均加速度
        # ---------------------------------------------------------

        mean_acc = np.mean(
            valid_data,
            axis=0
        )

        std_acc = np.std(
            valid_data,
            axis=0
        )

        ax, ay, az = mean_acc

        g = np.linalg.norm(
            mean_acc
        )

        # ---------------------------------------------------------
        # 3. 根据重力方向计算 Roll / Pitch
        #
        # 注意：
        # 最终符号需要通过实际点云验证。
        # ---------------------------------------------------------

        roll = math.atan2(
            ay,
            az
        )

        pitch = math.atan2(
            -ax,
            math.sqrt(
                ay * ay +
                az * az
            )
        )

        self.estimated_roll_deg = math.degrees(
            roll
        )

        self.estimated_pitch_deg = math.degrees(
            pitch
        )

        # ---------------------------------------------------------
        # 4. 点云反向校正（roll / pitch 自动，yaw 叠加手动值）
        # ---------------------------------------------------------

        if self.auto_correct:

            self.correction_pitch_deg = (
                -self.estimated_pitch_deg
            )
            self.correction_roll_deg = (
                -self.estimated_roll_deg
            )

        else:

            self.correction_pitch_deg = (
                self.manual_correction_pitch_deg
            )
            self.correction_roll_deg = (
                self.manual_correction_roll_deg
            )

        self.correction_yaw_deg = (
            self.manual_correction_yaw_deg
        )

        self.calibration_finished = True

        # ---------------------------------------------------------
        # 5. 输出结果
        # ---------------------------------------------------------

        print()
        print("=" * 65)
        print(" 标定完成")
        print("=" * 65)

        print()
        print("IMU平均加速度")
        print("-" * 40)

        print(f"ax = {ax:+.6f} m/s²")
        print(f"ay = {ay:+.6f} m/s²")
        print(f"az = {az:+.6f} m/s²")

        print()
        print(f"|g| = {g:.6f} m/s²")

        print()
        print("IMU标准差")
        print("-" * 40)

        print(f"σx = {std_acc[0]:.6f}")
        print(f"σy = {std_acc[1]:.6f}")
        print(f"σz = {std_acc[2]:.6f}")

        print()
        print("姿态估计")
        print("-" * 40)

        print(
            f"Roll       = "
            f"{self.estimated_roll_deg:+.4f}°"
        )

        print(
            f"Pitch      = "
            f"{self.estimated_pitch_deg:+.4f}°"
        )

        print()
        print(
            f"点云校正角："
            f"Roll {self.correction_roll_deg:+.4f}°  "
            f"Pitch {self.correction_pitch_deg:+.4f}°  "
            f"Yaw {self.correction_yaw_deg:+.4f}°"
        )

        print()

        # ---------------------------------------------------------
        # 简单质量判断（Livox 驱动 IMU 输出单位为 g，自动识别）
        # ---------------------------------------------------------

        g_si = g * 9.81 if g < 3.0 else g
        if g < 3.0:
            print("（IMU 输出单位为 g，非 m/s²；角度计算为比值，不受影响）")

        if abs(g_si - 9.81) > 0.5:

            print(
                "⚠ 重力模长偏离9.81较大，"
                "标定期间可能存在运动或振动。"
            )

        if abs(self.estimated_roll_deg) > 1.0:

            print(
                "⚠ Roll超过1°。"
                "可能存在船体横倾或雷达并非纯Y轴安装。"
            )

        else:

            print(
                "✓ Roll较小，基本符合主要绕Y轴安装的假设。"
            )

        print()
        print(
            "现在开始实时发布校正后的点云："
        )

        print(
            self.corrected_cloud_topic
        )

        print("=" * 65)

        self.save_result(
            ax,
            ay,
            az,
            g,
            std_acc,
            len(valid_data)
        )

    # =============================================================
    # 点云处理
    # =============================================================

    def cloud_callback(self, msg):

        if not self.calibration_finished:
            return

        # ---------------------------------------------------------
        # PointCloud2读取XYZ
        # ---------------------------------------------------------

        try:

            points = np.asarray(
                list(
                    point_cloud2.read_points(
                        msg,
                        field_names=(
                            'x',
                            'y',
                            'z'
                        ),
                        skip_nans=True
                    )
                )
            )

        except Exception as e:

            self.get_logger().error(
                f"读取PointCloud2失败: {e}"
            )

            return

        if len(points) == 0:
            return

        # 某些ROS2版本返回结构化数组
        if points.dtype.names is not None:

            xyz = np.column_stack((
                points['x'],
                points['y'],
                points['z']
            ))

        else:

            xyz = np.asarray(
                points,
                dtype=np.float32
            )[:, :3]

        # ---------------------------------------------------------
        # 全姿态旋转：R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
        # ---------------------------------------------------------

        cr = math.radians(self.correction_roll_deg)
        cp = math.radians(self.correction_pitch_deg)
        cy = math.radians(self.correction_yaw_deg)

        Rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0, math.cos(cr), -math.sin(cr)],
            [0.0, math.sin(cr), math.cos(cr)],
        ])
        Ry = np.array([
            [math.cos(cp), 0.0, math.sin(cp)],
            [0.0, 1.0, 0.0],
            [-math.sin(cp), 0.0, math.cos(cp)],
        ])
        Rz = np.array([
            [math.cos(cy), -math.sin(cy), 0.0],
            [math.sin(cy), math.cos(cy), 0.0],
            [0.0, 0.0, 1.0],
        ])

        R = Rz @ Ry @ Rx

        corrected_xyz = (
            xyz @ R.T
        ).astype(np.float32)

        # ---------------------------------------------------------
        # 创建新的PointCloud2
        # ---------------------------------------------------------

        corrected_msg = (
            point_cloud2.create_cloud_xyz32(
                msg.header,
                corrected_xyz.tolist()
            )
        )

        # 使用新的frame名称
        corrected_msg.header.frame_id = (
            'livox_corrected_frame'
        )

        self.cloud_pub.publish(
            corrected_msg
        )

    # =============================================================
    # 保存结果
    # =============================================================

    def save_result(
        self,
        ax,
        ay,
        az,
        g,
        std_acc,
        sample_num
    ):

        try:

            with open(
                self.result_file,
                'w',
                encoding='utf-8'
            ) as f:

                f.write(
                    "Livox Mid-360S "
                    "船载安装角标定结果\n"
                )

                f.write(
                    "=" * 50 + "\n"
                )

                f.write(
                    f"有效IMU样本数: "
                    f"{sample_num}\n\n"
                )

                f.write(
                    "平均加速度\n"
                )

                f.write(
                    f"ax = {ax:+.8f} m/s²\n"
                )

                f.write(
                    f"ay = {ay:+.8f} m/s²\n"
                )

                f.write(
                    f"az = {az:+.8f} m/s²\n"
                )

                f.write(
                    f"|g| = {g:.8f} m/s²\n\n"
                )

                f.write(
                    "标准差\n"
                )

                f.write(
                    f"sigma_x = "
                    f"{std_acc[0]:.8f}\n"
                )

                f.write(
                    f"sigma_y = "
                    f"{std_acc[1]:.8f}\n"
                )

                f.write(
                    f"sigma_z = "
                    f"{std_acc[2]:.8f}\n\n"
                )

                f.write(
                    "姿态结果\n"
                )

                f.write(
                    f"Roll = "
                    f"{self.estimated_roll_deg:+.6f} deg\n"
                )

                f.write(
                    f"Pitch = "
                    f"{self.estimated_pitch_deg:+.6f} deg\n"
                )

                f.write(
                    f"Correction Roll = "
                    f"{self.correction_roll_deg:+.6f} deg\n"
                )

                f.write(
                    f"Correction Pitch = "
                    f"{self.correction_pitch_deg:+.6f} deg\n"
                )

                f.write(
                    f"Correction Yaw (manual) = "
                    f"{self.correction_yaw_deg:+.6f} deg\n"
                )

            print(
                f"\n标定结果已保存："
                f"{self.result_file}"
            )

        except Exception as e:

            self.get_logger().error(
                f"保存结果失败: {e}"
            )


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = Mid360SShipCalibration()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        print(
            "\n程序已停止。"
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

