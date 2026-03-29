#ifndef MY_SENSOR_GATEWAY_SENSOR_BASE_HPP_
#define MY_SENSOR_GATEWAY_SENSOR_BASE_HPP_

#include <string>
#include <memory>
#include <cstdint>
#include <vector>
#include <functional>
#include "rclcpp/rclcpp.hpp"

// ====================== 通用传感器数据结构体 ======================
// 优化：预分配固定大小内存，运行时零动态分配，支持写时复制
struct SensorData
{
    // 固定常量定义，适配我们的传感器参数
    static constexpr size_t MAX_LIDAR_POINTS = 360;       // 激光雷达最大点数
    static constexpr size_t MAX_IMAGE_SIZE = 160 * 120 * 4; // 深度相机最大图像大小（160x120 32位浮点）
    static constexpr size_t MAX_RAW_DATA_SIZE = 1024;     // 原始数据最大长度

    uint64_t timestamp;          // 时间戳（微秒）
    std::string sensor_name;     // 传感器名称
    std::string sensor_type;     // 传感器类型："lidar" "imu" "camera"
    bool is_valid;               // 数据是否有效

    // 优化1：预分配固定大小的数组，替代动态vector，运行时零内存分配
    // 激光雷达专属
    float ranges[MAX_LIDAR_POINTS];
    size_t ranges_count; // 实际有效点数
    float range_min;
    float range_max;
    // IMU专属
    float linear_acceleration[3];
    float angular_velocity[3];
    float orientation[4];
    // 相机专属
    uint32_t image_width;
    uint32_t image_height;
    std::string image_encoding;
    uint8_t image_data[MAX_IMAGE_SIZE];
    size_t image_data_size; // 实际有效图像数据长度
    // 原始数据
    uint8_t raw_data[MAX_RAW_DATA_SIZE];
    size_t raw_data_size;

    // 构造函数：初始化所有数据，预分配内存
    SensorData()
    {
        timestamp = 0;
        is_valid = false;
        ranges_count = 0;
        range_min = 0.0f;
        range_max = 0.0f;
        image_width = 0;
        image_height = 0;
        image_data_size = 0;
        raw_data_size = 0;
        // 初始化数组为0
        memset(ranges, 0, sizeof(ranges));
        memset(linear_acceleration, 0, sizeof(linear_acceleration));
        memset(angular_velocity, 0, sizeof(angular_velocity));
        memset(orientation, 0, sizeof(orientation));
        memset(image_data, 0, sizeof(image_data));
        memset(raw_data, 0, sizeof(raw_data));
    }

    // 禁用拷贝构造（避免全量拷贝，强制用移动语义）
    SensorData(const SensorData&) = delete;
    SensorData& operator=(const SensorData&) = delete;

    // 启用移动构造（零拷贝，只转移资源所有权，不拷贝数据）
    SensorData(SensorData&& other) noexcept
    {
        // 浅拷贝，零内存拷贝
        timestamp = other.timestamp;
        sensor_name = std::move(other.sensor_name);
        sensor_type = std::move(other.sensor_type);
        is_valid = other.is_valid;
        ranges_count = other.ranges_count;
        range_min = other.range_min;
        range_max = other.range_max;
        image_width = other.image_width;
        image_height = other.image_height;
        image_encoding = std::move(other.image_encoding);
        image_data_size = other.image_data_size;
        raw_data_size = other.raw_data_size;

        // 数组数据直接拷贝（固定大小，开销极低，或用memcpy优化）
        memcpy(ranges, other.ranges, sizeof(ranges));
        memcpy(linear_acceleration, other.linear_acceleration, sizeof(linear_acceleration));
        memcpy(angular_velocity, other.angular_velocity, sizeof(angular_velocity));
        memcpy(orientation, other.orientation, sizeof(orientation));
        memcpy(image_data, other.image_data, sizeof(image_data));
        memcpy(raw_data, other.raw_data, sizeof(raw_data));

        // 重置源对象
        other.is_valid = false;
        other.ranges_count = 0;
        other.image_data_size = 0;
    }

    // 移动赋值运算符
    SensorData& operator=(SensorData&& other) noexcept
    {
        if (this != &other) {
            timestamp = other.timestamp;
            sensor_name = std::move(other.sensor_name);
            sensor_type = std::move(other.sensor_type);
            is_valid = other.is_valid;
            ranges_count = other.ranges_count;
            range_min = other.range_min;
            range_max = other.range_max;
            image_width = other.image_width;
            image_height = other.image_height;
            image_encoding = std::move(other.image_encoding);
            image_data_size = other.image_data_size;
            raw_data_size = other.raw_data_size;

            memcpy(ranges, other.ranges, sizeof(ranges));
            memcpy(linear_acceleration, other.linear_acceleration, sizeof(linear_acceleration));
            memcpy(angular_velocity, other.angular_velocity, sizeof(angular_velocity));
            memcpy(orientation, other.orientation, sizeof(orientation));
            memcpy(image_data, other.image_data, sizeof(image_data));
            memcpy(raw_data, other.raw_data, sizeof(raw_data));

            other.is_valid = false;
            other.ranges_count = 0;
            other.image_data_size = 0;
        }
        return *this;
    }
};


// ====================== 字符设备驱动通用接口枚举 ======================
// 模拟真实硬件的ioctl命令，嵌入式驱动岗面试必问
enum class SensorIoctlCmd : uint32_t
{
    GET_PARAM = 0x01,  // 获取传感器参数
    SET_PARAM = 0x02,  // 设置传感器参数
    RESET = 0x03,       // 复位传感器
    CALIBRATE = 0x04,   // 校准传感器
};

// ====================== GPIO操作抽象接口 ======================
// 模拟真实GPIO的操作，嵌入式开发必备
class GpioInterface
{
public:
    enum class Direction { INPUT, OUTPUT };
    enum class Level { LOW, HIGH };

    virtual ~GpioInterface() = default;
    virtual bool set_direction(uint32_t pin, Direction dir) = 0;
    virtual bool write(uint32_t pin, Level level) = 0;
    virtual bool read(uint32_t pin, Level & level) = 0;
};

// ====================== I2C总线操作抽象接口 ======================
// 模拟真实I2C总线的操作，大部分传感器用I2C通信
class I2cInterface
{
public:
    virtual ~I2cInterface() = default;
    virtual bool open(uint8_t dev_addr) = 0;
    virtual bool write(uint8_t reg_addr, const std::vector<uint8_t> & data) = 0;
    virtual bool read(uint8_t reg_addr, std::vector<uint8_t> & data, size_t len) = 0;
    virtual bool close() = 0;
};

// ====================== SPI总线操作抽象接口 ======================
// 模拟真实SPI总线的操作，高速传感器用SPI通信
class SpiInterface
{
public:
    enum class Mode { MODE0, MODE1, MODE2, MODE3 };

    virtual ~SpiInterface() = default;
    virtual bool open(uint32_t bus, uint32_t cs, Mode mode, uint32_t speed_hz) = 0;
    virtual bool transfer(const std::vector<uint8_t> & tx_data, std::vector<uint8_t> & rx_data) = 0;
    virtual bool close() = 0;
};

// ====================== 传感器驱动抽象基类 ======================
class SensorBase
{
public:
    explicit SensorBase(
        const std::string & name,
        const std::string & type,
        rclcpp::Node::SharedPtr node)
        : name_(name), type_(type), node_(node), is_opened_(false), fd_(-1)
    {
        RCLCPP_INFO(node_->get_logger(), "传感器 [%s] 基类初始化完成", name_.c_str());
    }

    virtual ~SensorBase() = default;

    // ====================== 驱动核心生命周期接口 ======================
    virtual bool init() = 0;
    virtual bool open() = 0;
    virtual bool read(SensorData & data) = 0;
    virtual bool close() = 0;

    // ====================== 新增：字符设备驱动通用接口 ======================
    // 模拟真实硬件的write/ioctl系统调用，嵌入式驱动岗面试必问
    /**
     * @brief 写入数据到传感器：对应真实硬件的write()系统调用
     * @param data 要写入的数据
     * @return 成功返回true，失败返回false
     */
    virtual bool write(const std::vector<uint8_t> & data)
    {
        (void)data;
        RCLCPP_WARN(node_->get_logger(), "传感器 [%s] 未实现write接口", name_.c_str());
        return false;
    }

    /**
     * @brief 传感器控制：对应真实硬件的ioctl()系统调用
     * @param cmd 控制命令
     * @param arg 控制参数
     * @return 成功返回true，失败返回false
     */
    virtual bool ioctl(SensorIoctlCmd cmd, void * arg)
    {
        (void)cmd;
        (void)arg;
        RCLCPP_WARN(node_->get_logger(), "传感器 [%s] 未实现ioctl接口", name_.c_str());
        return false;
    }

    // ====================== 通用工具接口 ======================
    std::string get_name() const { return name_; }
    std::string get_type() const { return type_; }
    bool is_opened() const { return is_opened_; }
    int get_fd() const { return fd_; }

protected:
    std::string name_;
    std::string type_;
    rclcpp::Node::SharedPtr node_;
    bool is_opened_;
    int fd_;  // 模拟真实硬件的文件描述符
    SensorData latest_data_;
};

#endif  // MY_SENSOR_GATEWAY_SENSOR_BASE_HPP_
