#ifndef MY_SENSOR_GATEWAY_IMU_DRIVER_HPP
#define MY_SENSOR_GATEWAY_IMU_DRIVER_HPP

#include "my_sensor_gateway/sensor_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuDriver : public SensorBase
{
public:
  ImuDriver(const std::string & name, rclcpp::Node::SharedPtr node);
  ~ImuDriver() override;

  // 关键修复：返回值从 bool → DriverError
  DriverError init() override;
  DriverError open() override;
  DriverError read(SensorData & data) override;
  DriverError close() override;

private:
  void imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  bool data_received_;
};

#endif
