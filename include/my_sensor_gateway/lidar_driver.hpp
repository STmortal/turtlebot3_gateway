#ifndef MY_SENSOR_GATEWAY_LIDAR_DRIVER_HPP_
#define MY_SENSOR_GATEWAY_LIDAR_DRIVER_HPP_

#include "my_sensor_gateway/sensor_base.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarDriver : public SensorBase
{
public:
    // 构造函数
    explicit LidarDriver(
        const std::string & name,
        rclcpp::Node::SharedPtr node);
    ~LidarDriver() override;

    // 重写基类的纯虚函数
    bool init() override;
    bool open() override;
    bool read(SensorData & data) override;
    bool close() override;

private:
    // 激光雷达数据回调函数：收到/scan话题数据时调用
    void lidar_data_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    // ROS2订阅者
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    // 数据是否收到的标志位
    bool data_received_;
};

#endif  // MY_SENSOR_GATEWAY_LIDAR_DRIVER_HPP_

