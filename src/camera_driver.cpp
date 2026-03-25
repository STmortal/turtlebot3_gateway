#include "my_sensor_gateway/camera_driver.hpp"
#include <chrono>

CameraDriver::CameraDriver(const std::string & name, rclcpp::Node::SharedPtr node)
: SensorBase(name, "camera", node), data_received_(false), exposure_time_(10000)
{
}

CameraDriver::~CameraDriver()
{
    close();
}

bool CameraDriver::init()
{
    RCLCPP_INFO(node_->get_logger(), "深度相机 [%s] 初始化中...", get_name().c_str());
    data_received_ = false;
    exposure_time_ = 10000;  // 默认曝光时间10ms
    RCLCPP_INFO(node_->get_logger(), "深度相机 [%s] 初始化完成", get_name().c_str());
    return true;
}

bool CameraDriver::open()
{
    if (is_opened_) {
        RCLCPP_WARN(node_->get_logger(), "深度相机 [%s] 已经打开", get_name().c_str());
        return true;
    }

    RCLCPP_INFO(node_->get_logger(), "深度相机 [%s] 打开中...", get_name().c_str());
    // 订阅虚拟深度相机话题
    camera_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/camera/depth/image_raw", 10,
        std::bind(&CameraDriver::camera_data_callback, this, std::placeholders::_1));

    is_opened_ = true;
    RCLCPP_INFO(node_->get_logger(), "深度相机 [%s] 打开成功", get_name().c_str());
    return true;
}

bool CameraDriver::read(SensorData & data)
{
    if (!is_opened_) {
        RCLCPP_ERROR(node_->get_logger(), "深度相机 [%s] 未打开，无法读取数据", get_name().c_str());
        return false;
    }

    if (!data_received_) {
        RCLCPP_WARN(node_->get_logger(), "深度相机 [%s] 暂无数据", get_name().c_str());
        return false;
    }

    data = latest_data_;
    data.timestamp = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    data.is_valid = true;
    return true;
}

bool CameraDriver::close()
{
    if (!is_opened_) {
        return true;
    }

    RCLCPP_INFO(node_->get_logger(), "深度相机 [%s] 关闭中...", get_name().c_str());
    camera_sub_.reset();
    is_opened_ = false;
    data_received_ = false;
    RCLCPP_INFO(node_->get_logger(), "深度相机 [%s] 关闭成功", get_name().c_str());
    return true;
}

bool CameraDriver::ioctl(SensorIoctlCmd cmd, void * arg)
{
    switch (cmd) {
        case SensorIoctlCmd::GET_PARAM:
            RCLCPP_INFO(node_->get_logger(), "深度相机 [%s] 获取参数：曝光时间 = %u us",
                get_name().c_str(), exposure_time_);
            if (arg) {
                *static_cast<uint32_t*>(arg) = exposure_time_;
            }
            return true;
        case SensorIoctlCmd::SET_PARAM:
            if (arg) {
                exposure_time_ = *static_cast<uint32_t*>(arg);
                RCLCPP_INFO(node_->get_logger(), "深度相机 [%s] 设置参数：曝光时间 = %u us",
                    get_name().c_str(), exposure_time_);
            }
            return true;
        case SensorIoctlCmd::RESET:
            RCLCPP_INFO(node_->get_logger(), "深度相机 [%s] 复位", get_name().c_str());
            exposure_time_ = 10000;
            return true;
        default:
            RCLCPP_WARN(node_->get_logger(), "深度相机 [%s] 未知ioctl命令", get_name().c_str());
            return false;
    }
}

void CameraDriver::camera_data_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    latest_data_.sensor_name = get_name();
    latest_data_.sensor_type = get_type();
    latest_data_.image_width = msg->width;
    latest_data_.image_height = msg->height;
    latest_data_.image_encoding = msg->encoding;
    latest_data_.image_data = msg->data;
    data_received_ = true;
}
