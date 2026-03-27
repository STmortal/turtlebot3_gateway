#ifndef MY_SENSOR_GATEWAY_SENSOR_MANAGER_HPP_
#define MY_SENSOR_GATEWAY_SENSOR_MANAGER_HPP_

#include "my_sensor_gateway/sensor_base.hpp"
#include "my_sensor_gateway/ring_buffer.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <vector>
#include <thread>
#include <unordered_map>
#include <string>

class SensorManager
{
public:
    /**
     * @brief 构造函数
     * @param node ROS2节点指针，用于日志和传感器初始化
     */
    explicit SensorManager(rclcpp::Node::SharedPtr node);
    ~SensorManager();

    /**
     * @brief 注册传感器到管理类
     * @param sensor 传感器驱动对象
     * @param buffer_capacity 对应环形缓冲区的容量
     * @return 成功返回true，失败返回false
     */
    bool register_sensor(std::shared_ptr<SensorBase> sensor, size_t buffer_capacity = 100);

    /**
     * @brief 初始化所有已注册的传感器
     * @return 全部成功返回true，任意一个失败返回false
     */
    bool init_all_sensors();

    /**
     * @brief 打开所有传感器，启动采集线程
     * @return 全部成功返回true，任意一个失败返回false
     */
    bool start_all_sensors();

    /**
     * @brief 停止所有采集线程，关闭所有传感器
     */
    void stop_all_sensors();

    /**
     * @brief 读取指定传感器的最新数据
     * @param sensor_name 传感器名称
     * @param data 输出参数，读取到的传感器数据
     * @param timeout_ms 超时时间
     * @return 成功返回true，失败返回false
     */
    bool read_sensor_data(const std::string & sensor_name, SensorData & data, uint32_t timeout_ms = 100);

    /**
     * @brief 获取所有已注册的传感器名称
     */
    std::vector<std::string> get_all_sensor_names() const;

private:
    /**
     * @brief 单个传感器的采集线程函数
     * @param sensor 传感器对象
     */
    void sensor_collect_thread(std::shared_ptr<SensorBase> sensor);

    // ROS2节点指针
    rclcpp::Node::SharedPtr node_;
    // 已注册的传感器：key=传感器名称，value=传感器对象
    std::unordered_map<std::string, std::shared_ptr<SensorBase>> sensors_;
    // 每个传感器对应的环形缓冲区
    std::unordered_map<std::string, std::shared_ptr<RingBuffer<SensorData>>> buffers_;
    // 每个传感器对应的采集线程
    std::unordered_map<std::string, std::shared_ptr<std::thread>> collect_threads_;
    // 运行状态标志
    std::atomic<bool> is_running_;
};

#endif  // MY_SENSOR_GATEWAY_SENSOR_MANAGER_HPP_

