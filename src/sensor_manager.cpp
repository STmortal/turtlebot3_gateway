#include "my_sensor_gateway/sensor_manager.hpp"
#include <chrono>

SensorManager::SensorManager(rclcpp::Node::SharedPtr node)
: node_(node), is_running_(false)
{
    RCLCPP_INFO(node_->get_logger(), "传感器管理类初始化完成");
}

SensorManager::~SensorManager()
{
    stop_all_sensors();
    RCLCPP_INFO(node_->get_logger(), "传感器管理类销毁");
}

bool SensorManager::register_sensor(std::shared_ptr<SensorBase> sensor, size_t buffer_capacity)
{
    if (!sensor) {
        RCLCPP_ERROR(node_->get_logger(), "注册传感器失败：传感器对象为空");
        return false;
    }

    std::string sensor_name = sensor->get_name();
    if (sensors_.count(sensor_name) > 0) {
        RCLCPP_WARN(node_->get_logger(), "传感器 [%s] 已注册，无需重复注册", sensor_name.c_str());
        return true;
    }

    // 注册传感器，创建对应的环形缓冲区
    sensors_[sensor_name] = sensor;
    buffers_[sensor_name] = std::make_shared<RingBuffer<SensorData>>(buffer_capacity);
    RCLCPP_INFO(node_->get_logger(), "传感器 [%s] 注册成功，缓冲区容量：%zu",
        sensor_name.c_str(), buffer_capacity);
    return true;
}

bool SensorManager::init_all_sensors()
{
    RCLCPP_INFO(node_->get_logger(), "===== 开始初始化所有传感器 =====");
    for (auto & [name, sensor] : sensors_) {
        if (!sensor->init()) {
            RCLCPP_ERROR(node_->get_logger(), "传感器 [%s] 初始化失败", name.c_str());
            return false;
        }
    }
    RCLCPP_INFO(node_->get_logger(), "===== 所有传感器初始化完成 =====");
    return true;
}

bool SensorManager::start_all_sensors()
{
    if (is_running_) {
        RCLCPP_WARN(node_->get_logger(), "传感器采集已在运行中，无需重复启动");
        return true;
    }

    RCLCPP_INFO(node_->get_logger(), "===== 启动所有传感器采集线程 =====");
    is_running_ = true;

    // 先打开所有传感器
    for (auto & [name, sensor] : sensors_) {
        if (!sensor->open()) {
            RCLCPP_ERROR(node_->get_logger(), "传感器 [%s] 打开失败", name.c_str());
            is_running_ = false;
            return false;
        }
    }

    // 为每个传感器创建独立的采集线程
    for (auto & [name, sensor] : sensors_) {
        collect_threads_[name] = std::make_shared<std::thread>(
            &SensorManager::sensor_collect_thread, this, sensor);
        RCLCPP_INFO(node_->get_logger(), "传感器 [%s] 采集线程已启动", name.c_str());
    }

    RCLCPP_INFO(node_->get_logger(), "===== 所有传感器采集线程启动完成 =====");
    return true;
}

void SensorManager::stop_all_sensors()
{
    if (!is_running_) {
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "===== 停止所有传感器采集线程 =====");
    is_running_ = false;

    // 停止所有环形缓冲区，唤醒所有等待的线程
    for (auto & [name, buffer] : buffers_) {
        buffer->stop();
    }

    // 等待所有采集线程退出
    for (auto & [name, thread] : collect_threads_) {
        if (thread && thread->joinable()) {
            thread->join();
            RCLCPP_INFO(node_->get_logger(), "传感器 [%s] 采集线程已退出", name.c_str());
        }
    }

    // 关闭所有传感器
    for (auto & [name, sensor] : sensors_) {
        sensor->close();
    }

    // 清空所有容器
    collect_threads_.clear();
    buffers_.clear();
    sensors_.clear();

    RCLCPP_INFO(node_->get_logger(), "===== 所有传感器已停止 =====");
}

bool SensorManager::read_sensor_data(const std::string & sensor_name, SensorData & data, uint32_t timeout_ms)
{
    if (buffers_.count(sensor_name) == 0) {
        RCLCPP_ERROR(node_->get_logger(), "读取数据失败：传感器 [%s] 未注册", sensor_name.c_str());
        return false;
    }

    return buffers_[sensor_name]->pop(data, timeout_ms);
}

std::vector<std::string> SensorManager::get_all_sensor_names() const
{
    std::vector<std::string> names;
    for (auto & [name, sensor] : sensors_) {
        names.push_back(name);
    }
    return names;
}

// 采集线程核心函数：无限循环读取传感器数据，写入环形缓冲区
 // 采集线程核心函数：无限循环读取传感器数据，写入环形缓冲区
void SensorManager::sensor_collect_thread(std::shared_ptr<SensorBase> sensor)
{
    std::string sensor_name = sensor->get_name();
    RCLCPP_INFO(node_->get_logger(), "传感器 [%s] 采集线程开始运行", sensor_name.c_str());

    pthread_setname_np(pthread_self(), sensor_name.c_str());

    SensorData data;
    while (is_running_) {
        // 读取传感器数据
        if (sensor->read(data)) {
            // 优化：用std::move移动语义，零拷贝写入缓冲区
            if (!buffers_[sensor_name]->push(std::move(data), 100)) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(),
                    *node_->get_clock(), 1000,
                    "传感器 [%s] 缓冲区已满，数据丢失", sensor_name.c_str());
            }
        }
        // 按传感器类型适配采集频率
        if (sensor->get_type() == "imu") {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    RCLCPP_INFO(node_->get_logger(), "传感器 [%s] 采集线程结束", sensor_name.c_str());
}


