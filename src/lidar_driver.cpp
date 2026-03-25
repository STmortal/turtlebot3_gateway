#include "my_sensor_gateway/lidar_driver.hpp"
#include <chrono>

// 构造函数
LidarDriver::LidarDriver(const std::string & name, rclcpp::Node::SharedPtr node)
: SensorBase(name, "lidar", node), data_received_(false)
{
}

// 析构函数
LidarDriver::~LidarDriver()
{
    close();
}

// 初始化：对应真实激光雷达的参数配置、波特率设置
bool LidarDriver::init()
{
    RCLCPP_INFO(node_->get_logger(), "激光雷达 [%s] 初始化中...", get_name().c_str());
    // 真实硬件场景：这里会做串口初始化、寄存器配置、雷达参数设置
    // 仿真场景：只需要重置标志位，初始化完成
    data_received_ = false;
    RCLCPP_INFO(node_->get_logger(), "激光雷达 [%s] 初始化完成", get_name().c_str());
    return true;
}

// 打开设备：对应真实硬件的open()，打开串口设备文件，启动数据接收
bool LidarDriver::open()
{
    if (is_opened_) {
        RCLCPP_WARN(node_->get_logger(), "激光雷达 [%s] 已经打开", get_name().c_str());
        return true;
    }

    RCLCPP_INFO(node_->get_logger(), "激光雷达 [%s] 打开中...", get_name().c_str());
    // 真实硬件场景：这里会调用open()打开/dev/ttyUSB0串口设备
    // 仿真场景：订阅ROS2话题，模拟硬件数据接收
    lidar_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&LidarDriver::lidar_data_callback, this, std::placeholders::_1));

    is_opened_ = true;
    RCLCPP_INFO(node_->get_logger(), "激光雷达 [%s] 打开成功", get_name().c_str());
    return true;
}

// 读取数据：对应真实硬件的read()，从设备读取数据
bool LidarDriver::read(SensorData & data)
{
    if (!is_opened_) {
        RCLCPP_ERROR(node_->get_logger(), "激光雷达 [%s] 未打开，无法读取数据", get_name().c_str());
        return false;
    }

    if (!data_received_) {
        RCLCPP_WARN(node_->get_logger(), "激光雷达 [%s] 暂无数据", get_name().c_str());
        return false;
    }

    // 把最新的激光雷达数据拷贝到输出参数
    data = latest_data_;
    // 更新时间戳
    data.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    data.is_valid = true;

    RCLCPP_DEBUG(node_->get_logger(), "激光雷达 [%s] 数据读取成功", get_name().c_str());
    return true;
}

// 关闭设备：对应真实硬件的close()，关闭设备文件
bool LidarDriver::close()
{
    if (!is_opened_) {
        return true;
    }

    RCLCPP_INFO(node_->get_logger(), "激光雷达 [%s] 关闭中...", get_name().c_str());
    // 真实硬件场景：调用close()关闭串口设备
    // 仿真场景：重置订阅者，停止数据接收
    lidar_sub_.reset();
    is_opened_ = false;
    data_received_ = false;
    RCLCPP_INFO(node_->get_logger(), "激光雷达 [%s] 关闭成功", get_name().c_str());
    return true;
}

// 激光雷达数据回调函数：收到话题数据时，更新最新数据
void LidarDriver::lidar_data_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // 填充通用数据结构体
    latest_data_.sensor_name = get_name();
    latest_data_.sensor_type = get_type();
    latest_data_.range_min = msg->range_min;
    latest_data_.range_max = msg->range_max;
    latest_data_.ranges = msg->ranges;
    data_received_ = true;
}

