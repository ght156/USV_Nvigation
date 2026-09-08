#include "bw_gi320_udp_receiver.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>

#include <mlogger/mlogger.hpp>

namespace bw_gi320_driver
{

// 构造函数：初始化成员变量
UdpReceiver::UdpReceiver()
: socket_fd_(-1),          // 套接字描述符初始化为 -1（无效）
  initialized_(false)      // 初始化标志置为 false
{
}

// 析构函数：关闭 UDP 套接字
UdpReceiver::~UdpReceiver()
{
    close();
}

/**
 * @brief 初始化 UDP 接收器
 * @param local_ip   本机 IP 地址
 * @param local_port 本机端口号
 * @return true 初始化成功，false 初始化失败
 */
bool UdpReceiver::init(const std::string & local_ip, uint16_t local_port)
{
    // 创建 UDP 套接字
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);

    if (socket_fd_ < 0)
    {
        MLOGGER_ERROR("Create UDP socket failed.");
        return false;
    }

    // 设置接收超时（1s）
    timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    if (setsockopt(
            socket_fd_,
            SOL_SOCKET,
            SO_RCVTIMEO,
            &timeout,
            sizeof(timeout)) < 0)
    {
        MLOGGER_WARN("Set socket receive timeout failed.");
    }

    // 配置本地地址结构
    sockaddr_in local_addr{};
    std::memset(&local_addr, 0, sizeof(local_addr));

    local_addr.sin_family = AF_INET;                 // IPv4 协议族
    local_addr.sin_port = htons(local_port);         // 端口号（网络字节序）

    // 将 IP 字符串转换为二进制格式
    if (inet_pton(AF_INET, local_ip.c_str(), &local_addr.sin_addr) != 1)
    {
        MLOGGER_ERROR("Invalid local IP: {}", local_ip);

        close();
        return false;
    }

    // 将套接字绑定到指定的 IP 和端口
    if (bind(
            socket_fd_,
            reinterpret_cast<sockaddr *>(&local_addr),
            sizeof(local_addr)) < 0)
    {
        MLOGGER_ERROR(
            "Bind UDP {}:{} failed.",
            local_ip,
            local_port);

        close();
        return false;
    }

    initialized_ = true;

    MLOGGER_INFO(
        "UDP receiver bind {}:{} success.",
        local_ip,
        local_port);

    return true;
}

/**
 * @brief 接收 UDP 数据
 * @param data 输出参数，用于存放接收到的数据
 * @return true 成功接收数据，false 未接收到数据或出错
 */
bool UdpReceiver::receive(std::string & data)
{
    if (!initialized_)
    {
        return false;
    }

    // 发送端地址信息
    sockaddr_in sender_addr{};
    socklen_t sender_len = sizeof(sender_addr);

    // 从套接字接收数据
    ssize_t recv_len = recvfrom(
        socket_fd_,
        buffer_.data(),
        buffer_.size() - 1,      // 预留一个字节存放结束符
        0,
        reinterpret_cast<sockaddr *>(&sender_addr),
        &sender_len);

    if (recv_len <= 0)
    {
        return false;            // 超时或接收失败
    }

    buffer_[recv_len] = '\0';    // 添加字符串结束符

    data.assign(buffer_.data(), recv_len);  // 将数据拷贝到输出参数

    // // 将发送端地址从二进制转换为可读的字符串格式
    // char sender_ip[INET_ADDRSTRLEN];
    // inet_ntop(AF_INET, &sender_addr.sin_addr, sender_ip, INET_ADDRSTRLEN);

    // // 打印接收到的数据长度、发送端 IP:端口 以及数据内容（字符串形式）
    // // 注意：如果数据量很大，日志可能会很长，生产环境建议限制打印长度
    // MLOGGER_INFO(
    //     "Received {} bytes from {}:{}: {}",
    //     recv_len,
    //     sender_ip,
    //     ntohs(sender_addr.sin_port),
    //     data);

    return true;
}

/**
 * @brief 关闭 UDP 套接字并清理资源
 */
void UdpReceiver::close()
{
    if (socket_fd_ >= 0)
    {
        ::close(socket_fd_);     // 关闭套接字
        socket_fd_ = -1;         // 重置描述符

        MLOGGER_INFO("UDP socket closed.");
    }

    initialized_ = false;        // 重置初始化标志
}

}  // namespace rtk_driver