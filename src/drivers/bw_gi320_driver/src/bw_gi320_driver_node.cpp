#include "bw_gi320_driver_node.hpp"
#include <sstream>
#include <mlogger/mlogger.hpp>
#include <cmath>

namespace bw_gi320_driver
{

/**
 * @brief 构造函数：初始化 ROS2 节点及 UDP 接收器
 * @param options ROS2 节点选项
 *
 * 从参数服务器加载 IP 和端口配置，初始化 UDP 接收器，
 * 并启动后台接收线程。
 */
RtkNode::RtkNode(const rclcpp::NodeOptions & options)
: Node("rtk_driver_node", options),
  local_ip_("192.9.200.200"),      // 默认本机 IP
  local_port_(9000),               // 默认端口号
  running_(false)                  // 运行标志初始化为 false
{
    // 声明并获取 ROS2 参数
    declare_parameter("local_ip", local_ip_);
    declare_parameter("local_port", local_port_);

    get_parameter("local_ip", local_ip_);
    get_parameter("local_port", local_port_);

    MLOGGER_INFO(
        "RTK Driver start, local {}:{}",
        local_ip_,
        local_port_);

    // 初始化 UDP 接收器，失败则抛出异常
    if (!udp_receiver_.init(local_ip_, local_port_))
    {
        throw std::runtime_error("Failed to initialize UDP receiver.");
    }

    running_ = true;

    gngga_pub_ = this->create_publisher<bw_gi320_driver::msg::Gi320Gngga>("gi320/gngga", 10);

    inspvaa_pub_ = this->create_publisher<bw_gi320_driver::msg::Gi320Inspvaa>("gi320/inspvaa", 10);

    reset_gga_odom_srv_ =  this->create_service<bw_gi320_driver::srv::ResetGgaOdom>("gi320/reset_gga_odom",
                                                                std::bind(
                                                                    &RtkNode::resetGgaOdomCallback,
                                                                    this,
                                                                    std::placeholders::_1,
                                                                    std::placeholders::_2));

    reset_inspvaa_odom_srv_ = this->create_service<bw_gi320_driver::srv::ResetInspvaaOdom>("gi320/reset_inspvaa_odom",
                                                                    std::bind(
                                                                        &RtkNode::resetInspvaaOdomCallback,
                                                                        this,
                                                                        std::placeholders::_1,
                                                                        std::placeholders::_2));

    // 启动后台线程持续接收 UDP 数据
    receive_thread_ = std::thread(&RtkNode::receiveLoop, this);
}

/**
 * @brief 析构函数：停止接收线程并释放资源
 */
RtkNode::~RtkNode()
{
    running_ = false;          // 通知接收线程退出

    udp_receiver_.close();     // 关闭 UDP 套接字

    // 等待接收线程结束
    if (receive_thread_.joinable())
    {
        receive_thread_.join();
    }

    MLOGGER_INFO("RTK Driver stopped.");
}

/**
 * @brief 根据两次GPS坐标计算距离
 * @return 两点距离，单位m
 */
double RtkNode::calcDistance(
    double lat1,
    double lon1,
    double lat2,
    double lon2)
{
    constexpr double EARTH_RADIUS = 6378137.0; // WGS84半径 m


    //角度转弧度
    double rad_lat1 = lat1 * M_PI / 180.0;
    double rad_lat2 = lat2 * M_PI / 180.0;

    double delta_lat =
        (lat2 - lat1) * M_PI / 180.0;

    double delta_lon =
        (lon2 - lon1) * M_PI / 180.0;


    double a =
        sin(delta_lat / 2) * sin(delta_lat / 2)
        +
        cos(rad_lat1)
        *
        cos(rad_lat2)
        *
        sin(delta_lon / 2)
        *
        sin(delta_lon / 2);


    double c =
        2.0 * atan2(
            sqrt(a),
            sqrt(1.0 - a));


    return EARTH_RADIUS * c;
}

/**
 * @brief 更新里程计
 */
void RtkNode::updateOdom(
    OdomData& odom,
    double latitude,
    double longitude)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);

    //第一帧，只保存位置
    if (!odom.initialized)
    {
        odom.last_latitude = latitude;
        odom.last_longitude = longitude;

        odom.initialized = true;

        return;
    }


    //计算当前位置和上一帧距离
    double delta_distance =
        calcDistance(
            odom.last_latitude,
            odom.last_longitude,
            latitude,
            longitude);


    //累计
    odom.distance += delta_distance;

    //更新上一帧
    odom.last_latitude = latitude;
    odom.last_longitude = longitude;
}

//发布函数
void RtkNode::publishGngga(const RtkData& data)
{
    bw_gi320_driver::msg::Gi320Gngga msg;

    // 时间
    msg.stamp.sec = data.timestamp_sec;

    msg.stamp.nanosec = data.timestamp_nsec;

    msg.latitude = data.latitude;

    msg.longitude = data.longitude;

    msg.altitude = data.altitude;

    msg.fix_type = data.fix_type;

    msg.satellite_num = data.satellite_num;

    msg.hdop = data.hdop;

    //加入里程
    msg.odometer = gga_odom_.distance;

    gngga_pub_->publish(msg);
}

//发布函数
void RtkNode::publishInspvaa(const RtkData& data)
{
    bw_gi320_driver::msg::Gi320Inspvaa msg;
    msg.stamp.sec = data.timestamp_sec;

    msg.stamp.nanosec = data.timestamp_nsec;

    msg.latitude = data.latitude;

    msg.longitude = data.longitude;

    msg.altitude = data.altitude;

    msg.roll = data.roll;

    msg.pitch = data.pitch;

    msg.yaw = data.yaw;

    msg.velocity_n = data.velocity_n;

    msg.velocity_e = data.velocity_e;

    msg.velocity_u = data.velocity_u;

    //里程计
    msg.odometer = inspvaa_odom_.distance;

    inspvaa_pub_->publish(msg);
}

//srv回调
void RtkNode::resetGgaOdomCallback(const std::shared_ptr<bw_gi320_driver::srv::ResetGgaOdom::Request> request,
                            std::shared_ptr<bw_gi320_driver::srv::ResetGgaOdom::Response> response)
{
    std::lock_guard<std::mutex> lock(
    odom_mutex_);

    (void)request;

    gga_odom_.reset();

    response->success = true;
    response->message =
        "GGA odometer reset success";

    MLOGGER_INFO(
        "GGA odometer reset");
}

//srv回调
void RtkNode::resetInspvaaOdomCallback(const std::shared_ptr<bw_gi320_driver::srv::ResetInspvaaOdom::Request> request,
    std::shared_ptr<bw_gi320_driver::srv::ResetInspvaaOdom::Response> response)
{
    std::lock_guard<std::mutex> lock(
    odom_mutex_);

    (void)request;
    inspvaa_odom_.reset();

    response->success = true;
    response->message =
        "INSPVAA odometer reset success";

    MLOGGER_INFO(
        "INSPVAA odometer reset");
}

/**
 * @brief UDP 数据接收循环
 *
 * 持续从 UDP 接收器获取数据，按行解析 NMEA 句子，
 * 对每个有效句子调用解析器提取 RTK 数据并打印日志。
 */
void RtkNode::receiveLoop()
{
    std::string udp_data;

    while (running_)
    {
        // 接收 UDP 数据，超时或失败则继续等待
        if (!udp_receiver_.receive(udp_data))
        {
            continue;
        }

        std::stringstream ss(udp_data);

        std::string sentence;

        // 按行分割处理
        while (std::getline(ss, sentence))
        {
            // 去除行尾的回车符 '\r'
            if (!sentence.empty() && sentence.back() == '\r')
            {
                sentence.pop_back();
            }

            // 跳过空行
            if (sentence.empty())
            {
                continue;
            }

            RtkData data;

            // MLOGGER_INFO("NMEA raw: {}", sentence);

            // 解析 NMEA 句子，成功则输出结果
            if (parser_.parse(sentence, data))
            {

                if (data.has_gga)
                {
                    updateOdom(
                        gga_odom_,
                        data.latitude,
                        data.longitude);

                    publishGngga(data);
                }

                if (data.has_inspvaa)
                {
                    updateOdom(
                        inspvaa_odom_,
                        data.latitude,
                        data.longitude);

                    publishInspvaa(data);
                }


                MLOGGER_INFO(
                    "RTK odom: gga={:.3f} m, inspvaa={:.3f} m",
                    gga_odom_.distance,
                    inspvaa_odom_.distance);


                // TODO 发布ROS2消息
            }
        }
    }
}

}  // namespace rtk_driver