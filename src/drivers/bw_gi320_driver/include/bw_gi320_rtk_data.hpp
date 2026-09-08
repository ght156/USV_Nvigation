#pragma once

#include <string>

namespace bw_gi320_driver
{

struct RtkData
{
    // UTC 时间转 UNIX时间
    uint64_t timestamp_sec = 0;
    uint32_t timestamp_nsec = 0;

    // 定位信息
    double latitude = 0.0;  //单位：度
    double longitude = 0.0; //单位：度
    double altitude = 0.0;  //高度单位：米

    // 定位状态
    uint8_t fix_type = 0;   //GPS 质量指示符
    uint8_t satellite_num = 0;  //使用中的卫星数
    double hdop = 0.0;  //水平精度因子

    // 姿态角（单位：deg）
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;

    // 速度（单位：m/s）
    double velocity_n = 0.0;
    double velocity_e = 0.0;
    double velocity_u = 0.0;

    // 当前数据有效标志
    bool has_gga = false;
    bool has_inspvaa = false;

    void clear()
    {
        *this = RtkData();
    }

    bool isValid() const
    {
        return has_gga || has_inspvaa;
    }
};

}  // namespace rtk_driver