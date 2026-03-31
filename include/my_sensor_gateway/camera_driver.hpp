#ifndef MY_SENSOR_GATEWAY_CAMERA_DRIVER_HPP
#define MY_SENSOR_GATEWAY_CAMERA_DRIVER_HPP

#include "my_sensor_gateway/sensor_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraDriver : public SensorBase
{
public:
  CameraDriver(const std::string & name, rclcpp::Node::SharedPtr node);
  ~CameraDriver() override;

  DriverError init() override;
  DriverError open() override;
  DriverError read(SensorData & data) override;
  DriverError close() override;

  bool ioctl(SensorIoctlCmd cmd, void * arg) override;

private:
  void camera_data_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  bool data_received_;
  uint32_t exposure_time_;
};

#endif
