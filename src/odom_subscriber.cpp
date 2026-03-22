// 1. 包含必要的头文件
#include "rclcpp/rclcpp.hpp"          // ROS2 C++核心头文件
#include "nav_msgs/msg/odometry.hpp"  // 里程计消息类型头文件

// 2. 定义一个订阅者类，继承自rclcpp::Node
class OdomSubscriber : public rclcpp::Node
{
public:
    // 构造函数：初始化节点，创建订阅者
    OdomSubscriber() : Node("odom_subscriber")
    {
        // 3. 创建订阅者
        // 参数1：话题名称（TurtleBot3的里程计话题固定为/odom）
        // 参数2：队列大小（缓存10条消息）
        // 参数3：回调函数（收到消息后调用的函数）
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&OdomSubscriber::odom_callback, this, std::placeholders::_1));
        
        // 打印日志，告诉用户节点启动了
        RCLCPP_INFO(this->get_logger(), "里程计订阅者节点已启动！正在监听 /odom 话题...");
    }

private:
    // 4. 定义回调函数：收到里程计消息后执行
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
        // 从消息中提取机器人的位置信息（x, y, z）
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double z = msg->pose.pose.position.z;
        
        // 从消息中提取机器人的速度信息（线速度、角速度）
        double vx = msg->twist.twist.linear.x;
        double wz = msg->twist.twist.angular.z;
        
        // 打印日志，显示机器人状态
        RCLCPP_INFO(this->get_logger(), 
            "机器人位置: (x=%.2f, y=%.2f, z=%.2f) | 速度: vx=%.2f, wz=%.2f",
            x, y, z, vx, wz);
    }

    // 声明订阅者成员变量
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
};

// 5. main函数：程序入口
int main(int argc, char *argv[])
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建节点对象并运行（spin会让节点一直运行，直到按Ctrl+C停止）
    rclcpp::spin(std::make_shared<OdomSubscriber>());
    
    // 关闭ROS2
    rclcpp::shutdown();
    return 0;
}

