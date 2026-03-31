#include "my_sensor_gateway/sensor_manager.hpp"
#include <chrono>
#include <pthread.h>
#include <cstring>

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
        // 修复：判断 DriverError == SUCCESS
        if (sensor->init() != DriverError::SUCCESS) {
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

    for (auto & [name, sensor] : sensors_) {
        // 修复：判断 DriverError
        if (sensor->open() != DriverError::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "传感器 [%s] 打开失败", name.c_str());
            is_running_ = false;
            return false;
        }
    }

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

    for (auto & [name, buffer] : buffers_) {
        buffer->stop();
    }

    for (auto & [name, thread] : collect_threads_) {
        if (thread && thread->joinable()) {
            thread->join();
            RCLCPP_INFO(node_->get_logger(), "传感器 [%s] 采集线程已退出", name.c_str());
        }
    }

    for (auto & [name, sensor] : sensors_) {
        sensor->close();
    }

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

void SensorManager::sensor_collect_thread(std::shared_ptr<SensorBase> sensor)
{
    std::string sensor_name = sensor->get_name();
    RCLCPP_INFO(node_->get_logger(), "传感器 [%s] 采集线程开始运行", sensor_name.c_str());

    char thread_name[16];
    strncpy(thread_name, sensor_name.c_str(), 15);
    thread_name[15] = '\0';
    pthread_setname_np(pthread_self(), thread_name);

    SensorData data;
    while (is_running_) {
        // 修复：判断 DriverError == SUCCESS
        if (sensor->read(data) == DriverError::SUCCESS) {
            if (!buffers_[sensor_name]->push(std::move(data), 100)) {
                RCLCPP_WARN_THROTTLE(node_->get_logger(),
                    *node_->get_clock(), 1000,
                    "传感器 [%s] 缓冲区已满，数据丢失", sensor_name.c_str());
            }
        }

        if (sensor->get_type() == "imu") {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    RCLCPP_INFO(node_->get_logger(), "传感器 [%s] 采集线程结束", sensor_name.c_str());
}
