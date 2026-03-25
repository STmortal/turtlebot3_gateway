#ifndef MY_SENSOR_GATEWAY_CAMERA_DRIVER_HPP_
#define MY_SENSOR_GATEWAY_CAMERA_DRIVER_HPP_

#include "my_sensor_gateway/sensor_base.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraDriver : public SensorBase
{
public:
    explicit CameraDriver(
        const std::string & name,
        rclcpp::Node::SharedPtr node);
    ~CameraDriver() override;

    bool init() override;
    bool open() override;
    bool read(SensorData & data) override;
    bool close() override;

    // 重写ioctl接口，实现相机参数设置
    bool ioctl(SensorIoctlCmd cmd, void * arg) override;

private:
    void camera_data_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    bool data_received_;
    uint32_t exposure_time_;  // 模拟相机曝光时间参数
};

#endif  // MY_SENSOR_GATEWAY_CAMERA_DRIVER_HPP_
