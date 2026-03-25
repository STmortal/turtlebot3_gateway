#include "rclcpp/rclcpp.hpp"
#include "my_sensor_gateway/sensor_base.hpp"
#include "my_sensor_gateway/lidar_driver.hpp"
#include "my_sensor_gateway/imu_driver.hpp"
#include <memory>
#include <chrono>

class SensorDriverTestNode : public rclcpp::Node
{
public:
    SensorDriverTestNode() : Node("sensor_driver_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "===== 传感器驱动测试节点启动 =====");
    }

    // 初始化所有驱动（必须在节点被 shared_ptr 管理后调用）
    void init_drivers()
    {
        // 现在 shared_from_this() 是安全的
        lidar_driver_ = std::make_unique<LidarDriver>("front_lidar", shared_from_this());
        imu_driver_ = std::make_unique<ImuDriver>("body_imu", shared_from_this());

        init_all_sensors();
        open_all_sensors();

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SensorDriverTestNode::read_all_sensors, this));
    }

    ~SensorDriverTestNode()
    {
        close_all_sensors();
        RCLCPP_INFO(this->get_logger(), "===== 传感器驱动测试节点关闭 =====");
    }

private:
    void init_all_sensors()
    {
        RCLCPP_INFO(this->get_logger(), "----- 开始初始化传感器 -----");
        lidar_driver_->init();
        imu_driver_->init();
        RCLCPP_INFO(this->get_logger(), "----- 所有传感器初始化完成 -----");
    }

    void open_all_sensors()
    {
        RCLCPP_INFO(this->get_logger(), "----- 开始打开传感器 -----");
        lidar_driver_->open();
        imu_driver_->open();
        RCLCPP_INFO(this->get_logger(), "----- 所有传感器打开完成 -----");
    }

    void read_all_sensors()
    {
        SensorData data;

        RCLCPP_INFO(this->get_logger(), "----- 读取传感器数据 -----");
        if (lidar_driver_->read(data)) {
            RCLCPP_INFO(this->get_logger(), "激光雷达 [%s] 数据有效：测距范围 [%.2f, %.2f] m",
                data.sensor_name.c_str(), data.range_min, data.range_max);
        }

        if (imu_driver_->read(data)) {
            RCLCPP_INFO(this->get_logger(), "IMU [%s] 数据有效：加速度 X=%.2f Y=%.2f Z=%.2f m/s²",
                data.sensor_name.c_str(),
                data.linear_acceleration[0],
                data.linear_acceleration[1],
                data.linear_acceleration[2]);
        }
    }

    void close_all_sensors()
    {
        RCLCPP_INFO(this->get_logger(), "----- 开始关闭传感器 -----");
        if (lidar_driver_) lidar_driver_->close();
        if (imu_driver_) imu_driver_->close();
        RCLCPP_INFO(this->get_logger(), "----- 所有传感器关闭完成 -----");
    }

    std::unique_ptr<SensorBase> lidar_driver_;
    std::unique_ptr<SensorBase> imu_driver_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorDriverTestNode>();
    node->init_drivers();  // 关键：节点完全构造后调用初始化
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
