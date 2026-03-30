#include <gtest/gtest.h>
#include "my_sensor_gateway/lidar_driver.hpp"
#include "my_sensor_gateway/imu_driver.hpp"
#include "my_sensor_gateway/camera_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <thread>

// ====================== 测试夹具：为每个测试创建ROS2节点 ======================
class SensorDriverTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // 每个测试开始前，初始化ROS2并创建节点
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<rclcpp::Node>("test_sensor_node");
    }

    void TearDown() override
    {
        // 每个测试结束后，关闭ROS2
        node_.reset();
        rclcpp::shutdown();
    }

    std::shared_ptr<rclcpp::Node> node_;
};

// ====================== 测试用例1：激光雷达驱动生命周期 ======================
TEST_F(SensorDriverTest, LidarDriverLifecycle)
{
    auto lidar = std::make_unique<LidarDriver>("test_lidar", node_);

    // 测试初始化
    EXPECT_TRUE(lidar->init());
    EXPECT_FALSE(lidar->is_opened());

    // 测试打开
    EXPECT_TRUE(lidar->open());
    EXPECT_TRUE(lidar->is_opened());

    // 测试重复打开（应该返回true，不报错）
    EXPECT_TRUE(lidar->open());

    // 测试关闭
    EXPECT_TRUE(lidar->close());
    EXPECT_FALSE(lidar->is_opened());

    // 测试重复关闭（应该返回true，不报错）
    EXPECT_TRUE(lidar->close());
}

// ====================== 测试用例2：IMU驱动生命周期 ======================
TEST_F(SensorDriverTest, ImuDriverLifecycle)
{
    auto imu = std::make_unique<ImuDriver>("test_imu", node_);

    EXPECT_TRUE(imu->init());
    EXPECT_TRUE(imu->open());
    EXPECT_TRUE(imu->is_opened());
    EXPECT_TRUE(imu->close());
    EXPECT_FALSE(imu->is_opened());
}

// ====================== 测试用例3：相机驱动ioctl接口 ======================
TEST_F(SensorDriverTest, CameraDriverIoctl)
{
    auto camera = std::make_unique<CameraDriver>("test_camera", node_);
    camera->init();
    camera->open();

    // 测试设置参数
    uint32_t exposure = 20000;
    EXPECT_TRUE(camera->ioctl(SensorIoctlCmd::SET_PARAM, &exposure));

    // 测试获取参数
    uint32_t get_exposure = 0;
    EXPECT_TRUE(camera->ioctl(SensorIoctlCmd::GET_PARAM, &get_exposure));
    EXPECT_EQ(get_exposure, 20000);

    // 测试复位
    EXPECT_TRUE(camera->ioctl(SensorIoctlCmd::RESET, nullptr));

    camera->close();
}

// ====================== 测试用例4：未打开时读取数据（异常处理） ======================
TEST_F(SensorDriverTest, ReadWithoutOpen)
{
    auto lidar = std::make_unique<LidarDriver>("test_lidar", node_);
    lidar->init(); // 只初始化，不打开

    SensorData data;
    // 未打开时读取，应该返回false
    EXPECT_FALSE(lidar->read(data));
}

// ====================== main函数：运行所有测试 ======================
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

