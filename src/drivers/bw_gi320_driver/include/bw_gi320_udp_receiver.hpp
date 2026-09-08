#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace bw_gi320_driver
{

class UdpReceiver
{
public:
    UdpReceiver();
    ~UdpReceiver();

    UdpReceiver(const UdpReceiver &) = delete;
    UdpReceiver & operator=(const UdpReceiver &) = delete;

    UdpReceiver(UdpReceiver &&) = default;
    UdpReceiver & operator=(UdpReceiver &&) = default;

    bool init(const std::string & local_ip, uint16_t local_port);

    bool receive(std::string & data);

    void close();

    bool isInitialized() const
    {
        return initialized_;
    }

private:
    int socket_fd_;
    bool initialized_;

    std::array<char, 2048> buffer_;
};

}  // namespace rtk_driver