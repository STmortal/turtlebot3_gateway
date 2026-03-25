#ifndef MY_SENSOR_GATEWAY_FAKE_PERIPHERAL_HPP_
#define MY_SENSOR_GATEWAY_FAKE_PERIPHERAL_HPP_

#include "my_sensor_gateway/sensor_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include <unordered_map>
#include <random>

// ====================== 仿真版GPIO接口 ======================
class FakeGpio : public GpioInterface
{
public:
    explicit FakeGpio(rclcpp::Node::SharedPtr node) : node_(node) {}

    bool set_direction(uint32_t pin, Direction dir) override
    {
        RCLCPP_INFO(node_->get_logger(), "仿真GPIO: 设置引脚 %d 方向为 %s",
            pin, (dir == Direction::INPUT) ? "输入" : "输出");
        pin_directions_[pin] = dir;
        return true;
    }

    bool write(uint32_t pin, Level level) override
    {
        if (pin_directions_.count(pin) && pin_directions_[pin] != Direction::OUTPUT) {
            RCLCPP_ERROR(node_->get_logger(), "仿真GPIO: 引脚 %d 不是输出模式", pin);
            return false;
        }
        RCLCPP_DEBUG(node_->get_logger(), "仿真GPIO: 写入引脚 %d 电平 %s",
            pin, (level == Level::LOW) ? "低" : "高");
        pin_levels_[pin] = level;
        return true;
    }

    bool read(uint32_t pin, Level & level) override
    {
        if (pin_directions_.count(pin) && pin_directions_[pin] != Direction::INPUT) {
            RCLCPP_ERROR(node_->get_logger(), "仿真GPIO: 引脚 %d 不是输入模式", pin);
            return false;
        }
        // 仿真：随机返回高低电平，模拟真实传感器触发
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(0, 1);
        level = (dis(gen) == 0) ? Level::LOW : Level::HIGH;
        RCLCPP_DEBUG(node_->get_logger(), "仿真GPIO: 读取引脚 %d 电平 %s",
            pin, (level == Level::LOW) ? "低" : "高");
        return true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::unordered_map<uint32_t, Direction> pin_directions_;
    std::unordered_map<uint32_t, Level> pin_levels_;
};

// ====================== 仿真版I2C接口 ======================
class FakeI2c : public I2cInterface
{
public:
    explicit FakeI2c(rclcpp::Node::SharedPtr node) : node_(node), dev_addr_(0) {}

    bool open(uint8_t dev_addr) override
    {
        RCLCPP_INFO(node_->get_logger(), "仿真I2C: 打开设备地址 0x%02X", dev_addr);
        dev_addr_ = dev_addr;
        return true;
    }

    bool write(uint8_t reg_addr, const std::vector<uint8_t> & data) override
    {
        RCLCPP_DEBUG(node_->get_logger(), "仿真I2C: 写入寄存器 0x%02X, 数据长度 %zu",
            reg_addr, data.size());
        i2c_registers_[reg_addr] = data;
        return true;
    }

    bool read(uint8_t reg_addr, std::vector<uint8_t> & data, size_t len) override
    {
        RCLCPP_DEBUG(node_->get_logger(), "仿真I2C: 读取寄存器 0x%02X, 数据长度 %zu",
            reg_addr, len);
        // 仿真：返回模拟的传感器数据
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<> dis(0, 255);
        data.resize(len);
        for (size_t i = 0; i < len; i++) {
            data[i] = static_cast<uint8_t>(dis(gen));
        }
        return true;
    }

    bool close() override
    {
        RCLCPP_INFO(node_->get_logger(), "仿真I2C: 关闭设备");
        dev_addr_ = 0;
        return true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    uint8_t dev_addr_;
    std::unordered_map<uint8_t, std::vector<uint8_t>> i2c_registers_;
};

// ====================== 仿真版SPI接口 ======================
class FakeSpi : public SpiInterface
{
public:
    explicit FakeSpi(rclcpp::Node::SharedPtr node) : node_(node), is_opened_(false) {}

    bool open(uint32_t bus, uint32_t cs, Mode mode, uint32_t speed_hz) override
    {
        RCLCPP_INFO(node_->get_logger(), "仿真SPI: 打开总线 %d, 片选 %d, 模式 %d, 速度 %u Hz",
            bus, cs, static_cast<int>(mode), speed_hz);
        is_opened_ = true;
        return true;
    }

    bool transfer(const std::vector<uint8_t> & tx_data, std::vector<uint8_t> & rx_data) override
    {
        if (!is_opened_) {
            RCLCPP_ERROR(node_->get_logger(), "仿真SPI: 设备未打开");
            return false;
        }
        RCLCPP_DEBUG(node_->get_logger(), "仿真SPI: 传输数据, 发送长度 %zu", tx_data.size());
        // 仿真：返回和发送数据相同的回环数据，模拟真实SPI传输
        rx_data = tx_data;
        return true;
    }

    bool close() override
    {
        RCLCPP_INFO(node_->get_logger(), "仿真SPI: 关闭设备");
        is_opened_ = false;
        return true;
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool is_opened_;
};

#endif  // MY_SENSOR_GATEWAY_FAKE_PERIPHERAL_HPP_
