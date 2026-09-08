#include "bw_gi320_nmea_parser.hpp"

#include <iomanip>
#include <sstream>

#include <mlogger/mlogger.hpp>

namespace bw_gi320_driver
{

/**
 * @brief 解析 NMEA 句子
 * @param sentence NMEA 原始句子（含校验和）
 * @param data     输出参数，解析成功的 RTK 数据
 * @return true 解析成功且数据有效，false 解析失败
 *
 * 流程：校验和验证 → 识别句子类型（$GNGGA / $INSPVAA）→ 提取数据 → 判断有效性
 */
bool NmeaParser::parse(
    const std::string & sentence,
    RtkData & data)
{
    if (sentence.empty())
    {
        return false;
    }

    // 验证 NMEA 校验和
    if (!verifyChecksum(sentence))
    {
        MLOGGER_WARN("NMEA checksum failed.");
        return false;
    }

    // 根据句子前缀选择不同的解析器
    if (sentence.rfind("$GNGGA", 0) == 0)
    {
        // 解析 $GNGGA 语句（GPS 定位数据）
        if (!parseGGA(sentence))
        {
            return false;
        }
    }
    else if (sentence.rfind("$INSPVAA", 0) == 0)
    {
        // 解析 $INSPVAA 语句（惯导姿态数据）
        if (!parseINSPVAA(sentence))
        {
            return false;
        }
    }
    else
    {
        // 不支持的句子类型
        return false;
    }

    // 检查当前累积数据是否有效
    if (current_data_.isValid())
    {
        data = current_data_;     // 传出有效数据
        current_data_.clear();    // 清空缓存

        return true;
    }

    return false;
}

/**
 * @brief 解析 $GNGGA 语句
 * @param sentence $GNGGA 格式的 NMEA 句子
 * @return true 解析成功，false 解析失败
 *
 * $GNGGA 包含：UTC 时间、经纬度、定位质量、星数、海拔等信息
 * TODO: 待实现具体解析逻辑
 */
bool NmeaParser::parseGGA(const std::string & sentence)
{
    // 按逗号分割字段
    std::vector<std::string> fields;
    std::stringstream ss(sentence);
    std::string field;

    while (std::getline(ss, field, ','))
    {
        fields.push_back(field);
    }

    // GNGGA 最少需要 15 个字段（含校验和字段）
    // $GNGGA,utc,lat,lat_dir,lon,lon_dir,qual,sats,hdop,alt,a_units,undulation,u_units,age,stn_id*xx
    if (fields.size() < 15)
    {
        MLOGGER_WARN("GGA field count insufficient: {}", fields.size());
        return false;
    }

    try
    {
        // fields[0] = "$GNGGA"

        // fields[1] = "080316.00" (UTC 时间 hhmmss.ss)
         if (!utcToUnix(
                fields[1],
                current_data_.timestamp_sec,
                current_data_.timestamp_nsec))
        {
            MLOGGER_WARN(
                "GGA UTC convert failed: {}",
                fields[1]);

            return false;
        }

        // fields[2] = "3129.27096679" (纬度 DDmm.mm)
        // fields[3] = "N" (纬度方向)
        current_data_.latitude = nmeaToDegrees(fields[2], fields[3]);

        // fields[4] = "12022.12282662" (经度 DDDmm.mm)
        // fields[5] = "E" (经度方向)
        current_data_.longitude = nmeaToDegrees(fields[4], fields[5]);

        // fields[6] = "5" (GPS 质量指示符)
        current_data_.fix_type = static_cast<uint8_t>(std::stoi(fields[6]));

        // fields[7] = "31" (使用中的卫星数)
        current_data_.satellite_num = static_cast<uint8_t>(std::stoi(fields[7]));

        // fields[8] = "0.6" (水平精度因子 HDOP)
        current_data_.hdop = std::stod(fields[8]);

        // fields[9] = "4.9942" (天线海拔高度)
        // fields[10] = "M" (高度单位，忽略)
        current_data_.altitude = std::stod(fields[9]);

        // fields[11] = "7.6013" (大地水准面差距)
        // fields[12] = "M" (差距单位，忽略)

        // fields[13] = "1.0" (差分数据龄期)
        // fields[14] = "28*6B" (基站ID + 校验和)，需要截断
        // 这里不需要解析，跳过

        // 标记 GGA 已解析
        current_data_.has_gga = true;
        current_data_.has_inspvaa = false;

    MLOGGER_INFO(
        "GGA parsed: "
        "stamp={}.{}, "
        "lat={:.8f}, "
        "lon={:.8f}, "
        "alt={:.3f}, "
        "fix={}, "
        "sats={}, "
        "hdop={:.1f}",

        current_data_.timestamp_sec,
        current_data_.timestamp_nsec,

        current_data_.latitude,
        current_data_.longitude,
        current_data_.altitude,

        current_data_.fix_type,
        current_data_.satellite_num,
        current_data_.hdop);
    }
    catch (const std::exception & e)
    {
        MLOGGER_ERROR("GGA parse exception: {}", e.what());
        return false;
    }

    return true;
}

/**
 * @brief 解析 $INSPVAA 语句
 * @param sentence $INSPVAA 格式的 NMEA 句子
 * @return true 解析成功，false 解析失败
 *
 * $INSPVAA 包含：惯导系统输出的航向角、俯仰角、横滚角等姿态信息
 */
bool NmeaParser::parseINSPVAA(const std::string & sentence)
{
    // 按逗号分割
    std::vector<std::string> fields;

    std::stringstream ss(sentence);
    std::string field;

    while (std::getline(ss, field, ','))
    {
        fields.push_back(field);
    }

    // 实际格式：
    // $INSPVAA,UTC,lat,lon,height,ve,vn,vu,roll,pitch,yaw*CRC
    if (fields.size() < 11)
    {
        MLOGGER_WARN(
            "INSPVAA field count insufficient: {}",
            fields.size());

        return false;
    }

    try
    {
        // fields[0] = "$INSPVAA"

        // UTC时间
        if (!utcToUnix(
                fields[1],
                current_data_.timestamp_sec,
                current_data_.timestamp_nsec))
        {
            MLOGGER_ERROR(
                "INSPVAA UTC time parse failed: {}",
                fields[1]);

            return false;
        }

        // 位姿信息
        // 纬度
        current_data_.latitude = std::stod(fields[2]);

        // 经度
        current_data_.longitude = std::stod(fields[3]);

        // 椭球高
        current_data_.altitude = std::stod(fields[4]);

        // 东向速度
        current_data_.velocity_e = std::stod(fields[5]);

        // 北向速度
        current_data_.velocity_n = std::stod(fields[6]);

        // 天向速度
        current_data_.velocity_u = std::stod(fields[7]);

        // Roll
        current_data_.roll = std::stod(fields[8]);

        // Pitch
        current_data_.pitch = std::stod(fields[9]);

        // Yaw（去掉CRC）
        std::string yaw_str = fields[10];
        size_t star_pos = yaw_str.find('*');
        if (star_pos != std::string::npos)
        {
            yaw_str = yaw_str.substr(0, star_pos);
        }

        current_data_.yaw = std::stod(yaw_str);
        //标记inspvaa有效
        current_data_.has_inspvaa = true;
        current_data_.has_gga = false;

        MLOGGER_INFO(
            "INSPVAA parsed: "
            "stamp={}.{}, "
            "lat={:.8f}, "
            "lon={:.8f}, "
            "alt={:.3f}, "
            "ve={:.3f}, "
            "vn={:.3f}, "
            "vu={:.3f}, "
            "roll={:.3f}, "
            "pitch={:.3f}, "
            "yaw={:.3f}",

            current_data_.timestamp_sec,
            current_data_.timestamp_nsec,

            current_data_.latitude,
            current_data_.longitude,
            current_data_.altitude,

            current_data_.velocity_e,
            current_data_.velocity_n,
            current_data_.velocity_u,

            current_data_.roll,
            current_data_.pitch,
            current_data_.yaw);
    }
    catch (const std::exception & e)
    {
        MLOGGER_ERROR(
            "INSPVAA parse exception: {}",
            e.what());

        return false;
    }

    return true;
}

/**
 * @brief 验证 NMEA 句子的校验和
 * @param sentence 带校验和的 NMEA 句子（如 $GPGGA,...*XX）
 * @return true 校验和正确，false 校验和错误或格式不合法
 *
 * NMEA 校验和规则：从 '$' 之后到 '*' 之间的所有字符按位异或，
 * 结果与 '*' 后的两位十六进制数比较。
 */
bool NmeaParser::verifyChecksum(
    const std::string & sentence) const
{
    // 查找校验和分隔符 '*'
    auto star_pos = sentence.find('*');

    if (star_pos == std::string::npos)
    {
        return false;    // 缺少 '*'，格式不合法
    }

    // 逐字节异或计算校验和
    uint8_t checksum = 0;

    for (size_t i = 1; i < star_pos; ++i)
    {
        checksum ^= static_cast<uint8_t>(sentence[i]);
    }

    // 提取期望的校验和值（十六进制字符串）
    std::string checksum_str = sentence.substr(star_pos + 1);

    uint32_t expected = 0;

    std::stringstream ss;
    ss << std::hex << checksum_str;
    ss >> expected;

    return checksum == expected;
}

/**
 * @brief 将 NMEA 度分格式转换为十进制度
 * @param nmea_str  NMEA 格式字符串，如 "3129.27096679" (31度29.27096679分)
 * @param dir       方向，"N"/"S" 或 "E"/"W"
 * @return 十进制度，南纬/西经为负值
 */
double NmeaParser::nmeaToDegrees(const std::string & nmea_str, const std::string & dir)
{
    double nmea_val = std::stod(nmea_str);

    // 提取整数部分为度
    int degrees = static_cast<int>(nmea_val / 100);

    // 小数部分为分
    double minutes = nmea_val - degrees * 100;

    // 转换为十进制度
    double decimal_deg = degrees + minutes / 60.0;

    // 根据方向确定正负
    if (dir == "S" || dir == "W")
    {
        decimal_deg = -decimal_deg;
    }

    return decimal_deg;
}

/**
 * @brief 按指定分隔符分割字符串
 * @param str       待分割的字符串
 * @param delimiter 分隔符
 * @return 分割后的子字符串列表
 *
 * NMEA 句子通常以逗号 ',' 作为字段分隔符。
 */
std::vector<std::string> NmeaParser::split(
    const std::string & str,
    char delimiter) const
{
    std::vector<std::string> result;

    std::stringstream ss(str);

    std::string item;

    while (std::getline(ss, item, delimiter))
    {
        result.emplace_back(item);
    }

    return result;
}



bool NmeaParser::utcToUnix(const std::string & utc, uint64_t & sec, uint32_t & nsec)
{

    int year;
    int month;
    int day;

    int hour;
    int minute;
    int second;
    int millisecond;

    // inspvaa情况1：    // 完整时间 2026-7-3 8:3:16.870
    if (utc.find('-') != std::string::npos)
    {
        int ret = std::sscanf(
            utc.c_str(),
            "%d-%d-%d %d:%d:%d.%d",
            &year,
            &month,
            &day,
            &hour,
            &minute,
            &second,
            &millisecond);

        if (ret != 7)
        {
            MLOGGER_ERROR(
                "Invalid datetime format: {}",
                utc);

            return false;
        }

        // 更新日期缓存
        utc_year_ = year;
        utc_month_ = month;
        utc_day_ = day;
    }
    // GNGGA情况2：  // 只有时间 hhmmss.ss
    else
    {
        if (utc_year_ == 0)
        {
            MLOGGER_WARN(
                "No date information available for UTC time: {}",
                utc);

            return false;
        }

        double time_value = std::stod(utc);
        hour = static_cast<int>(time_value / 10000);
        minute = static_cast<int>((time_value - hour * 10000) / 100);


        double sec_value =  time_value - hour * 10000 - minute * 100;
        second = static_cast<int>(sec_value);
        millisecond = static_cast<int>((sec_value - second) * 1000.0);


        year = utc_year_;
        month = utc_month_;
        day = utc_day_;
    }

    // 转换Unix时间

    std::tm tm = {};

    tm.tm_year = year - 1900;
    tm.tm_mon  = month - 1;
    tm.tm_mday = day;

    tm.tm_hour = hour;
    tm.tm_min  = minute;
    tm.tm_sec  = second;


    time_t unix_time = timegm(&tm);


    if (unix_time < 0)
    {
        MLOGGER_ERROR(
            "Invalid unix time: {}",
            unix_time);

        return false;
    }


    if (static_cast<uint64_t>(unix_time) >
        std::numeric_limits<uint32_t>::max())
    {
        MLOGGER_ERROR(
            "Unix time overflow: {}",
            unix_time);

        return false;
    }


    sec =
        static_cast<uint32_t>(unix_time);

    nsec =
        static_cast<uint32_t>(millisecond)
        * 1000000u;


    return true;
}

}  // namespace rtk_driver