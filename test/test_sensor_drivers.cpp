#include <gtest/gtest.h>
#include "my_sensor_gateway/lidar_driver.hpp"
#include "my_sensor_gateway/imu_driver.hpp"
#include "my_sensor_gateway/camera_driver.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <thread>

// ====================== 测试夹具：全局只初始化一次ROS2 ======================
class SensorDriverTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite()
    {
        rclcpp::shutdown();
    }

    void SetUp() override
    {
        node_ = std::make_shared<rclcpp::Node>("test_sensor_node");
    }

    void TearDown() override
    {
        node_.reset();
    }

    std::shared_ptr<rclcpp::Node> node_;
};

// ====================== 测试用例1：激光雷达驱动生命周期 ======================
TEST_F(SensorDriverTest, LidarDriverLifecycle)
{
    auto lidar = std::make_unique<LidarDriver>("test_lidar", node_);

    // 测试初始化
    EXPECT_EQ(lidar->init(), DriverError::SUCCESS);
    EXPECT_FALSE(lidar->is_opened());

    // 测试打开
    EXPECT_EQ(lidar->open(), DriverError::SUCCESS);
    EXPECT_TRUE(lidar->is_opened());

    // 测试重复打开
    EXPECT_EQ(lidar->open(), DriverError::SUCCESS);

    // 测试关闭
    EXPECT_EQ(lidar->close(), DriverError::SUCCESS);
    EXPECT_FALSE(lidar->is_opened());

    // 测试重复关闭
    EXPECT_EQ(lidar->close(), DriverError::SUCCESS);
}

// ====================== 测试用例2：IMU驱动生命周期 ======================
TEST_F(SensorDriverTest, ImuDriverLifecycle)
{
    auto imu = std::make_unique<ImuDriver>("test_imu", node_);

    EXPECT_EQ(imu->init(), DriverError::SUCCESS);
    EXPECT_EQ(imu->open(), DriverError::SUCCESS);
    EXPECT_TRUE(imu->is_opened());
    EXPECT_EQ(imu->close(), DriverError::SUCCESS);
    EXPECT_FALSE(imu->is_opened());
}

// ====================== 测试用例3：相机驱动ioctl接口 ======================
TEST_F(SensorDriverTest, CameraDriverIoctl)
{
    auto camera = std::make_unique<CameraDriver>("test_camera", node_);
    EXPECT_EQ(camera->init(), DriverError::SUCCESS);
    EXPECT_EQ(camera->open(), DriverError::SUCCESS);

    // 测试设置参数
    uint32_t exposure = 20000;
    EXPECT_TRUE(camera->ioctl(SensorIoctlCmd::SET_PARAM, &exposure));

    // 测试获取参数
    uint32_t get_exposure = 0;
    EXPECT_TRUE(camera->ioctl(SensorIoctlCmd::GET_PARAM, &get_exposure));
    EXPECT_EQ(get_exposure, 20000);

    // 测试复位
    EXPECT_TRUE(camera->ioctl(SensorIoctlCmd::RESET, nullptr));

    EXPECT_EQ(camera->close(), DriverError::SUCCESS);
}

// ====================== 测试用例4：未打开时读取数据（异常处理） ======================
TEST_F(SensorDriverTest, ReadWithoutOpen)
{
    auto lidar = std::make_unique<LidarDriver>("test_lidar", node_);
    EXPECT_EQ(lidar->init(), DriverError::SUCCESS); // 只初始化，不打开

    SensorData data;
    // ✅ 修复：返回 DriverError，不能用 EXPECT_FALSE
    EXPECT_NE(lidar->read(data), DriverError::SUCCESS);
}

// ====================== main函数：运行所有测试 ======================
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
