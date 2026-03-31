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
DriverError LidarDriver::init()
{
    if (state_ != DriverState::UNINITIALIZED) {
        RCLCPP_ERROR(node_->get_logger(), "激光雷达 [%s] 初始化失败：状态非法", get_name().c_str());
        state_ = DriverState::ERROR;
        return DriverError::ERROR_STATE_INVALID;
    }

    RCLCPP_INFO(node_->get_logger(), "激光雷达 [%s] 初始化中...", get_name().c_str());
    data_received_ = false;
    state_ = DriverState::INITIALIZED;
    RCLCPP_INFO(node_->get_logger(), "激光雷达 [%s] 初始化完成", get_name().c_str());
    return DriverError::SUCCESS;
}


// 打开设备：对应真实硬件的open()，打开串口设备文件，启动数据接收
DriverError LidarDriver::open()
{
    // 已打开 → 直接返回成功，不报错
    if (state_ == DriverState::OPENED) {
        RCLCPP_INFO(node_->get_logger(), "激光雷达 [%s] 已处于打开状态，无需重复操作", get_name().c_str());
        return DriverError::SUCCESS;
    }

    if (state_ != DriverState::INITIALIZED) {
        RCLCPP_ERROR(node_->get_logger(), "激光雷达 [%s] 打开失败：未初始化或已打开", get_name().c_str());
        state_ = DriverState::ERROR;
        return DriverError::ERROR_STATE_INVALID;
    }

    RCLCPP_INFO(node_->get_logger(), "激光雷达 [%s] 打开中...", get_name().c_str());
    // 订阅激光雷达话题（示例）
    lidar_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&LidarDriver::lidar_data_callback, this, std::placeholders::_1));

    state_ = DriverState::OPENED;
    RCLCPP_INFO(node_->get_logger(), "激光雷达 [%s] 打开成功", get_name().c_str());
    return DriverError::SUCCESS;
}




// 读取数据：对应真实硬件的read()，从设备读取数据
DriverError LidarDriver::read(SensorData & data)
{
    if (state_ != DriverState::OPENED) {
        RCLCPP_ERROR(node_->get_logger(), "激光雷达 [%s] 未打开，无法读取数据", get_name().c_str());
        return DriverError::ERROR_STATE_INVALID;
    }

    if (!data_received_) {
        RCLCPP_WARN(node_->get_logger(), "激光雷达 [%s] 暂无数据", get_name().c_str());
        return DriverError::ERROR_STATE_INVALID;
    }

    std::shared_lock<std::shared_mutex> lock(data_mutex_);

    // 修复：直接填充输出参数，不调用拷贝赋值，零拷贝优化完全保留
    data.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    data.sensor_name = get_name();
    data.sensor_type = get_type();
    data.is_valid = latest_data_.is_valid;
    data.range_min = latest_data_.range_min;
    data.range_max = latest_data_.range_max;
    data.ranges_count = latest_data_.ranges_count;
    
    // 直接拷贝数组数据，固定大小开销极低
    memcpy(data.ranges, latest_data_.ranges, sizeof(data.ranges));

    RCLCPP_DEBUG(node_->get_logger(), "激光雷达 [%s] 数据读取成功", get_name().c_str());
    return DriverError::SUCCESS;
}


// 关闭设备：对应真实硬件的close()，关闭设备文件
DriverError LidarDriver::close()
{
    // 已关闭/未初始化，直接返回
    if (state_ == DriverState::CLOSED || state_ == DriverState::UNINITIALIZED) {
        return DriverError::SUCCESS;
    }

    RCLCPP_INFO(node_->get_logger(), "激光雷达 [%s] 关闭中...", get_name().c_str());
    lidar_sub_.reset();
    data_received_ = false;

    // 关闭成功，更新状态
    state_ = DriverState::CLOSED;
    RCLCPP_INFO(node_->get_logger(), "激光雷达 [%s] 关闭成功", get_name().c_str());
    return DriverError::SUCCESS;
}


// 激光雷达数据回调函数：收到话题数据时，更新最新数据
void LidarDriver::lidar_data_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{   
    std::unique_lock<std::shared_mutex> lock(data_mutex_);

    if (msg->ranges.empty() || msg->range_min <= 0 || msg->range_max <= msg->range_min) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "激光雷达数据非法，丢弃");
        latest_data_.is_valid = false;
        return;
    }

    // 填充通用数据结构体，零动态分配
    latest_data_.sensor_name = get_name();
    latest_data_.sensor_type = get_type();
    latest_data_.range_min = msg->range_min;
    latest_data_.range_max = msg->range_max;
    latest_data_.ranges_count = std::min(msg->ranges.size(), (size_t)SensorData::MAX_LIDAR_POINTS);
    
    // 填充激光雷达数据，无动态内存分配
    for (size_t i = 0; i < latest_data_.ranges_count; i++) {
        latest_data_.ranges[i] = msg->ranges[i];
    }
    
    latest_data_.is_valid = true;
    data_received_ = true;
}


