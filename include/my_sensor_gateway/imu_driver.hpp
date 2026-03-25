#ifndef MY_SENSOR_GATEWAY_IMU_DRIVER_HPP_
#define MY_SENSOR_GATEWAY_IMU_DRIVER_HPP_

#include "my_sensor_gateway/sensor_base.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuDriver : public SensorBase
{
public:
    explicit ImuDriver(
        const std::string & name,
        rclcpp::Node::SharedPtr node);
    ~ImuDriver() override;

    bool init() override;
    bool open() override;
    bool read(SensorData & data) override;
    bool close() override;

private:
    void imu_data_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    bool data_received_;
};

#endif  // MY_SENSOR_GATEWAY_IMU_DRIVER_HPP_

