#include "rclcpp/rclcpp.hpp"
#include "my_sensor_gateway/sensor_manager.hpp"
#include "my_sensor_gateway/lidar_driver.hpp"
#include "my_sensor_gateway/imu_driver.hpp"
#include "my_sensor_gateway/camera_driver.hpp"
#include <memory>
#include <chrono>

class MultithreadCollectTestNode : public rclcpp::Node
{
public:
    MultithreadCollectTestNode() : Node("multithread_collect_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "===== 多线程数据采集测试节点启动 =====");
    }

    void init_system()
    {
        // 1. 创建传感器管理类
        sensor_manager_ = std::make_unique<SensorManager>(shared_from_this());

        // 2. 注册所有传感器
        sensor_manager_->register_sensor(
            std::make_shared<LidarDriver>("front_lidar", shared_from_this()), 100);
        sensor_manager_->register_sensor(
            std::make_shared<ImuDriver>("body_imu", shared_from_this()), 1000); // IMU缓冲区更大
        sensor_manager_->register_sensor(
            std::make_shared<CameraDriver>("depth_camera", shared_from_this()), 50);

        // 3. 初始化所有传感器
        sensor_manager_->init_all_sensors();

        // 4. 启动采集线程
        sensor_manager_->start_all_sensors();

        // 5. 创建定时器，100ms读取一次所有传感器数据（消费者）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&MultithreadCollectTestNode::read_all_data, this));

        RCLCPP_INFO(this->get_logger(), "===== 多线程采集系统初始化完成 =====");
    }

    ~MultithreadCollectTestNode()
    {
        if (sensor_manager_) {
            sensor_manager_->stop_all_sensors();
        }
        RCLCPP_INFO(this->get_logger(), "===== 多线程数据采集测试节点关闭 =====");
    }

private:
    void read_all_data()
    {
        SensorData data;
        static uint64_t count = 0;
        count++;

        // 每10次打印一次日志，避免刷屏
        if (count % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "----- 第%lu次读取传感器数据 -----", count);
        }

        // 读取激光雷达数据
        if (sensor_manager_->read_sensor_data("front_lidar", data, 10)) {
            if (count % 10 == 0) {
                RCLCPP_INFO(this->get_logger(), "激光雷达数据有效，测距范围 [%.2f, %.2f] m",
                    data.range_min, data.range_max);
            }
        }

        // 读取IMU数据
        if (sensor_manager_->read_sensor_data("body_imu", data, 10)) {
            if (count % 10 == 0) {
                RCLCPP_INFO(this->get_logger(), "IMU数据有效，加速度 Z=%.2f m/s²",
                    data.linear_acceleration[2]);
            }
        }

        // 读取相机数据
        if (sensor_manager_->read_sensor_data("depth_camera", data, 10)) {
            if (count % 10 == 0) {
                RCLCPP_INFO(this->get_logger(), "深度相机数据有效，分辨率 %ux%u",
                    data.image_width, data.image_height);
            }
        }
    }

    std::unique_ptr<SensorManager> sensor_manager_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultithreadCollectTestNode>();
    node->init_system(); // 节点完全构造后初始化，避免bad_weak_ptr
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

