#pragma once

#include <string>
#include <vector>

#include "bw_gi320_rtk_data.hpp"

namespace bw_gi320_driver
{

class NmeaParser
{
public:
    NmeaParser() = default;
    ~NmeaParser() = default;

    /**
     * @brief 解析一条RTK报文
     *
     * @param sentence 接收到的一整条报文
     * @param data 当返回true时，输出完整RTK数据
     *
     * @return true  当前收到完整的一组(GGA+INSPVAA)
     * @return false 数据尚未完整或解析失败
     */
    bool parse(const std::string & sentence, RtkData & data);

private:
    bool parseGGA(const std::string & sentence);
    bool parseINSPVAA(const std::string & sentence);

    bool verifyChecksum(const std::string & sentence) const;
    double nmeaToDegrees(const std::string & nmea_str, const std::string & dir);

    bool utcToUnix(const std::string & utc, uint64_t & sec, uint32_t & nsec);

    std::vector<std::string> split(
        const std::string & str,
        char delimiter) const;

private:
    RtkData current_data_;

    // 最近一次完整UTC日期
    int utc_year_ = 0;
    int utc_month_ = 0;
    int utc_day_ = 0;
};

}  // namespace rtk_driver