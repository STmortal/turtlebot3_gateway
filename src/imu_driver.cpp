#include "my_sensor_gateway/imu_driver.hpp"
#include <chrono>
#include <cmath>

ImuDriver::ImuDriver(const std::string & name, rclcpp::Node::SharedPtr node)
: SensorBase(name, "imu", node), data_received_(false)
{
}

ImuDriver::~ImuDriver()
{
    close();
}

DriverError ImuDriver::init()
{
    if (state_ != DriverState::UNINITIALIZED) {
        RCLCPP_ERROR(node_->get_logger(), "IMU [%s] 初始化失败：状态非法", get_name().c_str());
        state_ = DriverState::ERROR;
        return DriverError::ERROR_STATE_INVALID;
    }

    RCLCPP_INFO(node_->get_logger(), "IMU [%s] 初始化中...", get_name().c_str());
    data_received_ = false;
    state_ = DriverState::INITIALIZED;
    RCLCPP_INFO(node_->get_logger(), "IMU [%s] 初始化完成", get_name().c_str());
    return DriverError::SUCCESS;
}

DriverError ImuDriver::open()
{
    if (state_ != DriverState::INITIALIZED) {
        RCLCPP_WARN(node_->get_logger(), "IMU [%s] 打开失败：未初始化或已打开", get_name().c_str());
        return DriverError::ERROR_STATE_INVALID;
    }

    RCLCPP_INFO(node_->get_logger(), "IMU [%s] 打开中...", get_name().c_str());

    rclcpp::SubscriptionOptions options;
    options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", rclcpp::QoS(10).reliable(),
        std::bind(&ImuDriver::imu_data_callback, this, std::placeholders::_1),
        options);

    state_ = DriverState::OPENED;
    RCLCPP_INFO(node_->get_logger(), "IMU [%s] 打开成功", get_name().c_str());
    return DriverError::SUCCESS;
}

DriverError ImuDriver::read(SensorData & data)
{
    if (state_ != DriverState::OPENED) {
        RCLCPP_ERROR(node_->get_logger(), "IMU [%s] 未打开，无法读取数据", get_name().c_str());
        return DriverError::ERROR_STATE_INVALID;
    }

    if (!data_received_) {
        RCLCPP_WARN(node_->get_logger(), "IMU [%s] 暂无数据", get_name().c_str());
        return DriverError::ERROR_DATA_TIMEOUT;
    }

    std::shared_lock<std::shared_mutex> lock(data_mutex_);

    data.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    data.sensor_name = get_name();
    data.sensor_type = get_type();
    data.is_valid = latest_data_.is_valid;

    memcpy(data.linear_acceleration, latest_data_.linear_acceleration, sizeof(data.linear_acceleration));
    memcpy(data.angular_velocity, latest_data_.angular_velocity, sizeof(data.angular_velocity));
    memcpy(data.orientation, latest_data_.orientation, sizeof(data.orientation));

    return DriverError::SUCCESS;
}

DriverError ImuDriver::close()
{
    if (state_ == DriverState::CLOSED || state_ == DriverState::UNINITIALIZED) {
        return DriverError::SUCCESS;
    }

    RCLCPP_INFO(node_->get_logger(), "IMU [%s] 关闭中...", get_name().c_str());
    imu_sub_.reset();
    data_received_ = false;
    state_ = DriverState::CLOSED;
    RCLCPP_INFO(node_->get_logger(), "IMU [%s] 关闭成功", get_name().c_str());
    return DriverError::SUCCESS;
}

void ImuDriver::imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(data_mutex_);

    const float MAX_ACCEL = 156.8f;
    const float MAX_GYRO = 34.9f;

    if (fabs(msg->linear_acceleration.z) > MAX_ACCEL || fabs(msg->angular_velocity.z) > MAX_GYRO) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "IMU数据超出范围，丢弃");
        latest_data_.is_valid = false;
        return;
    }

    latest_data_.sensor_name = get_name();
    latest_data_.sensor_type = get_type();

    latest_data_.linear_acceleration[0] = msg->linear_acceleration.x;
    latest_data_.linear_acceleration[1] = msg->linear_acceleration.y;
    latest_data_.linear_acceleration[2] = msg->linear_acceleration.z;

    latest_data_.angular_velocity[0] = msg->angular_velocity.x;
    latest_data_.angular_velocity[1] = msg->angular_velocity.y;
    latest_data_.angular_velocity[2] = msg->angular_velocity.z;

    latest_data_.orientation[0] = msg->orientation.x;
    latest_data_.orientation[1] = msg->orientation.y;
    latest_data_.orientation[2] = msg->orientation.z;
    latest_data_.orientation[3] = msg->orientation.w;

    latest_data_.is_valid = true;
    data_received_ = true;
}
