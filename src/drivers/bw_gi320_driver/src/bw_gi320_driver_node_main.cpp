#include <memory>

#include <mlogger/mlogger.hpp>
#include <rclcpp/rclcpp.hpp>
#include "bw_gi320_driver_node.hpp"


int main(int argc, char ** argv)
{
    MLOGGER_MODULE_INIT("logs", "bw_gi320_driver", "bw_gi320_driver_node");
    MLOGGER_INFO("bw_gi320_driver_node 启动");

    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<bw_gi320_driver::RtkNode>();

        MLOGGER_INFO("bw_gi320_driver_node 开始 spin");

        rclcpp::spin(node);

        MLOGGER_INFO("bw_gi320_driver_node 正常退出");
    }
    catch (const std::exception & e)
    {
        MLOGGER_ERROR(
            "bw_gi320_driver_node 异常退出: {}",
            e.what());

        rclcpp::shutdown();
        return 1;
    }
    catch (...)
    {
        MLOGGER_ERROR(
            "bw_gi320_driver_node 未知异常退出");

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();

    return 0;
}