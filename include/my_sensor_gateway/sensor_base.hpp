#ifndef MY_SENSOR_GATEWAY_SENSOR_BASE_HPP_
#define MY_SENSOR_GATEWAY_SENSOR_BASE_HPP_

#include <string>
#include <memory>
#include <cstdint>
#include <vector>
#include "rclcpp/rclcpp.hpp"

// ====================== 通用传感器数据结构体 ======================
// 统一封装所有传感器的输出数据，上层代码只需要处理这一种数据结构
struct SensorData
{
    uint64_t timestamp;          // 时间戳（微秒），工业驱动必备，用于数据同步
    std::string sensor_name;     // 传感器名称
    std::string sensor_type;     // 传感器类型："lidar" "imu" "camera"
    std::vector<uint8_t> raw_data;  // 原始数据字节流，模拟真实硬件的原始数据
    bool is_valid;               // 数据是否有效

    // 不同传感器的扩展字段（按需填充）
    // 激光雷达专属
    std::vector<float> ranges;       // 测距数据
    float range_min;
    float range_max;
    // IMU专属
    float linear_acceleration[3];    // 加速度
    float angular_velocity[3];       // 角速度
    float orientation[4];            // 四元数姿态
    // 相机专属
    uint32_t image_width;
    uint32_t image_height;
    std::string image_encoding;
};

// ====================== 传感器驱动抽象基类 ======================
// 定义所有传感器驱动的通用接口与生命周期，纯虚函数必须由子类实现
class SensorBase
{
public:
    // 构造函数：传入传感器名称、类型、ROS2节点指针
    explicit SensorBase(
        const std::string & name,
        const std::string & type,
        rclcpp::Node::SharedPtr node)
        : name_(name), type_(type), node_(node), is_opened_(false)
    {
        RCLCPP_INFO(node_->get_logger(), "传感器 [%s] 基类初始化完成", name_.c_str());
    }

    // 虚析构函数：保证基类指针能正确释放子类资源，C++面向对象必备
    virtual ~SensorBase() = default;

    // ====================== 驱动核心生命周期接口（纯虚函数，子类必须实现） ======================
    /**
     * @brief 传感器初始化：对应真实硬件的寄存器配置、参数初始化
     * @return 成功返回true，失败返回false
     */
    virtual bool init() = 0;

    /**
     * @brief 打开传感器设备：对应真实硬件的open()系统调用，打开设备文件
     * @return 成功返回true，失败返回false
     */
    virtual bool open() = 0;

    /**
     * @brief 读取传感器数据：对应真实硬件的read()系统调用，从硬件读取数据
     * @param data 输出参数，读取到的传感器数据
     * @return 成功返回true，失败返回false
     */
    virtual bool read(SensorData & data) = 0;

    /**
     * @brief 关闭传感器设备：对应真实硬件的close()系统调用，关闭设备文件
     * @return 成功返回true，失败返回false
     */
    virtual bool close() = 0;

    // ====================== 通用工具接口 ======================
    std::string get_name() const { return name_; }
    std::string get_type() const { return type_; }
    bool is_opened() const { return is_opened_; }

protected:
    std::string name_;          // 传感器名称
    std::string type_;          // 传感器类型
    rclcpp::Node::SharedPtr node_;  // ROS2节点指针，用于话题订阅/日志打印
    bool is_opened_;            // 设备是否打开
    SensorData latest_data_;    // 最新的传感器数据
};

#endif  // MY_SENSOR_GATEWAY_SENSOR_BASE_HPP_

