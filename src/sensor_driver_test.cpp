#include "rclcpp/rclcpp.hpp"
#include "my_sensor_gateway/sensor_base.hpp"
#include "my_sensor_gateway/lidar_driver.hpp"
#include "my_sensor_gateway/imu_driver.hpp"
#include "my_sensor_gateway/camera_driver.hpp"
#include "my_sensor_gateway/fake_peripheral.hpp"
#include <memory>
#include <chrono>

class SensorDriverTestNode : public rclcpp::Node
{
public:
    SensorDriverTestNode() : Node("sensor_driver_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "===== 传感器驱动测试节点启动 =====");
    }

    void init_drivers()
    {
        // 1. 初始化传感器驱动
        lidar_driver_ = std::make_unique<LidarDriver>("front_lidar", shared_from_this());
        imu_driver_ = std::make_unique<ImuDriver>("body_imu", shared_from_this());
        camera_driver_ = std::make_unique<CameraDriver>("depth_camera", shared_from_this());

        // 2. 初始化仿真外设接口
        fake_gpio_ = std::make_unique<FakeGpio>(shared_from_this());
        fake_i2c_ = std::make_unique<FakeI2c>(shared_from_this());
        fake_spi_ = std::make_unique<FakeSpi>(shared_from_this());

        // 3. 测试外设接口
        test_peripherals();

        // 4. 初始化并打开传感器
        init_all_sensors();
        open_all_sensors();

        // 5. 测试相机ioctl接口
        test_camera_ioctl();

        // 6. 创建定时器，1秒读取一次传感器数据
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
    // 测试仿真外设接口
    void test_peripherals()
    {
        RCLCPP_INFO(this->get_logger(), "----- 开始测试仿真外设接口 -----");

        // 测试GPIO
        RCLCPP_INFO(this->get_logger(), "--- 测试GPIO接口 ---");
        fake_gpio_->set_direction(0, GpioInterface::Direction::OUTPUT);
        fake_gpio_->write(0, GpioInterface::Level::HIGH);
        fake_gpio_->set_direction(1, GpioInterface::Direction::INPUT);
        GpioInterface::Level level;
        fake_gpio_->read(1, level);

        // 测试I2C
        RCLCPP_INFO(this->get_logger(), "--- 测试I2C接口 ---");
        fake_i2c_->open(0x50);
        std::vector<uint8_t> write_data = {0x01, 0x02, 0x03};
        fake_i2c_->write(0x10, write_data);
        std::vector<uint8_t> read_data;
        fake_i2c_->read(0x10, read_data, 3);
        fake_i2c_->close();

        // 测试SPI
        RCLCPP_INFO(this->get_logger(), "--- 测试SPI接口 ---");
        fake_spi_->open(0, 0, SpiInterface::Mode::MODE0, 10000000);
        std::vector<uint8_t> tx_data = {0xAA, 0xBB, 0xCC};
        std::vector<uint8_t> rx_data;
        fake_spi_->transfer(tx_data, rx_data);
        fake_spi_->close();

        RCLCPP_INFO(this->get_logger(), "----- 仿真外设接口测试完成 -----");
    }

    // 测试相机ioctl接口
    void test_camera_ioctl()
    {
        RCLCPP_INFO(this->get_logger(), "----- 测试相机ioctl接口 -----");
        uint32_t exposure = 20000;
        camera_driver_->ioctl(SensorIoctlCmd::SET_PARAM, &exposure);
        camera_driver_->ioctl(SensorIoctlCmd::GET_PARAM, &exposure);
        RCLCPP_INFO(this->get_logger(), "----- 相机ioctl接口测试完成 -----");
    }

    void init_all_sensors()
    {
        RCLCPP_INFO(this->get_logger(), "----- 开始初始化传感器 -----");
        lidar_driver_->init();
        imu_driver_->init();
        camera_driver_->init();
        RCLCPP_INFO(this->get_logger(), "----- 所有传感器初始化完成 -----");
    }

    void open_all_sensors()
    {
        RCLCPP_INFO(this->get_logger(), "----- 开始打开传感器 -----");
        lidar_driver_->open();
        imu_driver_->open();
        camera_driver_->open();
        RCLCPP_INFO(this->get_logger(), "----- 所有传感器打开完成 -----");
    }

    void read_all_sensors()
    {
        SensorData data;

        RCLCPP_INFO(this->get_logger(), "----- 读取传感器数据 -----");
        if (lidar_driver_->read(data) == DriverError::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "激光雷达 [%s] 数据有效：测距范围 [%.2f, %.2f] m",
                data.sensor_name.c_str(), data.range_min, data.range_max);
        }

        if (imu_driver_->read(data) == DriverError::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "IMU [%s] 数据有效：加速度 X=%.2f Y=%.2f Z=%.2f m/s²",
                data.sensor_name.c_str(),
                data.linear_acceleration[0],
                data.linear_acceleration[1],
                data.linear_acceleration[2]);
        }

        if (camera_driver_->read(data) == DriverError::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "深度相机 [%s] 数据有效：分辨率 %ux%u, 编码 %s",
                data.sensor_name.c_str(),
                data.image_width, data.image_height,
                data.image_encoding.c_str());
        }
    }

    void close_all_sensors()
    {
        RCLCPP_INFO(this->get_logger(), "----- 开始关闭传感器 -----");
        if (lidar_driver_) lidar_driver_->close();
        if (imu_driver_) imu_driver_->close();
        if (camera_driver_) camera_driver_->close();
        RCLCPP_INFO(this->get_logger(), "----- 所有传感器关闭完成 -----");
    }

    std::unique_ptr<SensorBase> lidar_driver_;
    std::unique_ptr<SensorBase> imu_driver_;
    std::unique_ptr<SensorBase> camera_driver_;
    std::unique_ptr<FakeGpio> fake_gpio_;
    std::unique_ptr<FakeI2c> fake_i2c_;
    std::unique_ptr<FakeSpi> fake_spi_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SensorDriverTestNode>();
    node->init_drivers();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
