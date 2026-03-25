#include "my_sensor_gateway/imu_driver.hpp"
#include <chrono>

ImuDriver::ImuDriver(const std::string & name, rclcpp::Node::SharedPtr node)
: SensorBase(name, "imu", node), data_received_(false)
{
}

ImuDriver::~ImuDriver()
{
    close();
}

bool ImuDriver::init()
{
    RCLCPP_INFO(node_->get_logger(), "IMU [%s] 初始化中...", get_name().c_str());
    data_received_ = false;
    RCLCPP_INFO(node_->get_logger(), "IMU [%s] 初始化完成", get_name().c_str());
    return true;
}

bool ImuDriver::open()
{
    if (is_opened_) {
        RCLCPP_WARN(node_->get_logger(), "IMU [%s] 已经打开", get_name().c_str());
        return true;
    }

    RCLCPP_INFO(node_->get_logger(), "IMU [%s] 打开中...", get_name().c_str());
    // 订阅IMU话题，模拟真实硬件数据接收
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10,
        std::bind(&ImuDriver::imu_data_callback, this, std::placeholders::_1));

    is_opened_ = true;
    RCLCPP_INFO(node_->get_logger(), "IMU [%s] 打开成功", get_name().c_str());
    return true;
}

bool ImuDriver::read(SensorData & data)
{
    if (!is_opened_) {
        RCLCPP_ERROR(node_->get_logger(), "IMU [%s] 未打开，无法读取数据", get_name().c_str());
        return false;
    }

    if (!data_received_) {
        RCLCPP_WARN(node_->get_logger(), "IMU [%s] 暂无数据", get_name().c_str());
        return false;
    }

    data = latest_data_;
    data.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    data.is_valid = true;
    return true;
}

bool ImuDriver::close()
{
    if (!is_opened_) {
        return true;
    }

    RCLCPP_INFO(node_->get_logger(), "IMU [%s] 关闭中...", get_name().c_str());
    imu_sub_.reset();
    is_opened_ = false;
    data_received_ = false;
    RCLCPP_INFO(node_->get_logger(), "IMU [%s] 关闭成功", get_name().c_str());
    return true;
}

void ImuDriver::imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    // 填充IMU专属数据
    latest_data_.sensor_name = get_name();
    latest_data_.sensor_type = get_type();
    // 加速度
    latest_data_.linear_acceleration[0] = msg->linear_acceleration.x;
    latest_data_.linear_acceleration[1] = msg->linear_acceleration.y;
    latest_data_.linear_acceleration[2] = msg->linear_acceleration.z;
    // 角速度
    latest_data_.angular_velocity[0] = msg->angular_velocity.x;
    latest_data_.angular_velocity[1] = msg->angular_velocity.y;
    latest_data_.angular_velocity[2] = msg->angular_velocity.z;
    // 姿态四元数
    latest_data_.orientation[0] = msg->orientation.x;
    latest_data_.orientation[1] = msg->orientation.y;
    latest_data_.orientation[2] = msg->orientation.z;
    latest_data_.orientation[3] = msg->orientation.w;

    data_received_ = true;
}

