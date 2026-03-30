#include <gtest/gtest.h>
#include "my_sensor_gateway/ring_buffer.hpp"
#include "my_sensor_gateway/sensor_base.hpp"
#include <thread>
#include <vector>
#include <atomic>

// ====================== 测试用例1：基础功能测试 ======================
TEST(RingBufferTest, BasicFunctionality)
{
    // 创建一个容量为10的环形缓冲区，存储int
    RingBuffer<int> buffer(10);

    // 测试初始状态
    EXPECT_TRUE(buffer.empty());
    EXPECT_EQ(buffer.size(), 0);

    // 测试push数据
    for (int i = 0; i < 5; i++) {
        EXPECT_TRUE(buffer.push(i));
    }
    EXPECT_EQ(buffer.size(), 5);
    EXPECT_FALSE(buffer.empty());

    // 测试pop数据
    int data;
    for (int i = 0; i < 5; i++) {
        EXPECT_TRUE(buffer.pop(data));
        EXPECT_EQ(data, i);
    }
    EXPECT_TRUE(buffer.empty());
    EXPECT_EQ(buffer.size(), 0);
}

// ====================== 测试用例2：缓冲区满边界条件 ======================
TEST(RingBufferTest, BufferFull)
{
    RingBuffer<int> buffer(5);

    // 填满缓冲区
    for (int i = 0; i < 5; i++) {
        EXPECT_TRUE(buffer.push(i));
    }
    EXPECT_EQ(buffer.size(), 5);

    // 缓冲区满时，push应该返回false（非阻塞模式）
    EXPECT_FALSE(buffer.push(100, 10)); // 10ms超时
    EXPECT_EQ(buffer.size(), 5);
}

// ====================== 测试用例3：缓冲区空边界条件 ======================
TEST(RingBufferTest, BufferEmpty)
{
    RingBuffer<int> buffer(5);

    // 缓冲区空时，pop应该返回false（非阻塞模式）
    int data;
    EXPECT_FALSE(buffer.pop(data, 10)); // 10ms超时
}

// ====================== 测试用例4：清空缓冲区 ======================
TEST(RingBufferTest, ClearBuffer)
{
    RingBuffer<int> buffer(10);

    for (int i = 0; i < 5; i++) {
        buffer.push(i);
    }
    EXPECT_EQ(buffer.size(), 5);

    buffer.clear();
    EXPECT_TRUE(buffer.empty());
    EXPECT_EQ(buffer.size(), 0);
}

// ====================== 测试用例5：停止缓冲区 ======================
TEST(RingBufferTest, StopBuffer)
{
    RingBuffer<int> buffer(10);
    buffer.push(1);

    buffer.stop();

    // 停止后，push和pop都应该返回false
    int data;
    EXPECT_FALSE(buffer.push(2));
    EXPECT_FALSE(buffer.pop(data));
}

// ====================== 测试用例6：SensorData移动语义测试 ======================
TEST(RingBufferTest, SensorDataMoveSemantics)
{
    RingBuffer<SensorData> buffer(5);

    // 创建一个SensorData对象
    SensorData data1;
    data1.sensor_name = "test_lidar";
    data1.sensor_type = "lidar";
    data1.is_valid = true;
    data1.ranges_count = 10;
    for (size_t i = 0; i < 10; i++) {
        data1.ranges[i] = static_cast<float>(i);
    }

    // 用移动语义push
    EXPECT_TRUE(buffer.push(std::move(data1)));
    EXPECT_EQ(buffer.size(), 1);

    // pop出来，验证数据正确
    SensorData data2;
    EXPECT_TRUE(buffer.pop(data2));
    EXPECT_EQ(data2.sensor_name, "test_lidar");
    EXPECT_EQ(data2.sensor_type, "lidar");
    EXPECT_TRUE(data2.is_valid);
    EXPECT_EQ(data2.ranges_count, 10);
    for (size_t i = 0; i < 10; i++) {
        EXPECT_FLOAT_EQ(data2.ranges[i], static_cast<float>(i));
    }
}

// ====================== 测试用例7：多线程并发测试（核心重点） ======================
TEST(RingBufferTest, MultithreadedConcurrentAccess)
{
    RingBuffer<int> buffer(100);
    std::atomic<int> produced_count(0);
    std::atomic<int> consumed_count(0);
    const int total_items = 1000;

    // 生产者线程：生产1000个数据
    auto producer = [&]() {
        for (int i = 0; i < total_items; i++) {
            while (!buffer.push(i, 100)) {
                // 缓冲区满，等待一下
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
            produced_count++;
        }
    };

    // 消费者线程：消费1000个数据
    auto consumer = [&]() {
        int data;
        while (consumed_count < total_items) {
            if (buffer.pop(data, 100)) {
                consumed_count++;
            }
        }
    };

    // 启动生产者和消费者线程
    std::thread producer_thread(producer);
    std::thread consumer_thread(consumer);

    // 等待线程结束
    producer_thread.join();
    consumer_thread.join();

    // 验证：生产和消费的数量一致
    EXPECT_EQ(produced_count, total_items);
    EXPECT_EQ(consumed_count, total_items);
    EXPECT_TRUE(buffer.empty());
}

// ====================== main函数：运行所有测试 ======================
int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

