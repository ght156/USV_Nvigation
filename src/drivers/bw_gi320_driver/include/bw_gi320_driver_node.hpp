#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include "bw_gi320_nmea_parser.hpp"
#include "bw_gi320_rtk_data.hpp"
#include "bw_gi320_udp_receiver.hpp"

#include "bw_gi320_driver/msg/gi320_gngga.hpp"
#include "bw_gi320_driver/msg/gi320_inspvaa.hpp"
#include "bw_gi320_driver/srv/reset_gga_odom.hpp"
#include "bw_gi320_driver/srv/reset_inspvaa_odom.hpp"

namespace bw_gi320_driver
{


class RtkNode : public rclcpp::Node
{
public:
    explicit RtkNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~RtkNode();

private:
    void receiveLoop();
    // 里程计数据结构 
    struct OdomData
    {
        // 当前累计里程（单位：m）
        double distance = 0.0;

        // 是否已经收到第一帧数据
        bool initialized = false;

        // 上一次经纬度
        double last_latitude = 0.0;
        double last_longitude = 0.0;

        void reset()
        {
            distance = 0.0;
            initialized = false;
            last_latitude = 0.0;
            last_longitude = 0.0;
        }
    };

    //根据两次GPS坐标计算距离
    double calcDistance(double lat1, double lon1, double lat2, double lon2);

    // 更新里程计
    void updateOdom(OdomData& odom, double latitude, double longitude);
    //发布函数
    void publishGngga(const RtkData& data);
    //发布函数
    void publishInspvaa(const RtkData& data);
    //srv回调
    void resetGgaOdomCallback(const std::shared_ptr<bw_gi320_driver::srv::ResetGgaOdom::Request> request,
                            std::shared_ptr<bw_gi320_driver::srv::ResetGgaOdom::Response> response);
    //srv回调
    void resetInspvaaOdomCallback(const std::shared_ptr<bw_gi320_driver::srv::ResetInspvaaOdom::Request> request,
                            std::shared_ptr<bw_gi320_driver::srv::ResetInspvaaOdom::Response> response);

private:
    // Parameters
    std::string local_ip_;
    int local_port_;

    // UDP
    UdpReceiver udp_receiver_;

    // Parser
    NmeaParser parser_;

    // Thread
    std::thread receive_thread_;
    std::atomic_bool running_;

    std::mutex odom_mutex_; //里程计锁

    OdomData gga_odom_; //gngga里程计

    OdomData inspvaa_odom_; //inspvaa里程计

    rclcpp::Publisher<bw_gi320_driver::msg::Gi320Gngga>::SharedPtr gngga_pub_; //gngga话题发布智能指针

    rclcpp::Publisher<bw_gi320_driver::msg::Gi320Inspvaa>::SharedPtr inspvaa_pub_;  //inspvaa话题发布智能指针

    rclcpp::Service<bw_gi320_driver::srv::ResetGgaOdom>::SharedPtr reset_gga_odom_srv_;

    rclcpp::Service<bw_gi320_driver::srv::ResetInspvaaOdom>::SharedPtr reset_inspvaa_odom_srv_;
};

}  // namespace rtk_driver