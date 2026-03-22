cmake_minimum_required(VERSION 3.8)
project(turtlebot3_gateway)

# 1. 编译选项：开启警告，让代码更规范
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. 找到依赖包（和创建功能包时的--dependencies对应）
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  # 新增：里程计消息需要这个依赖

# 3. 添加头文件目录（后续我们会在include下放头文件）
include_directories(
  include
)

# 4. 添加可执行文件：odom_subscriber
# 参数1：可执行文件的名字（运行时用的名字）
# 参数2：源文件的路径
add_executable(odom_subscriber src/odom_subscriber.cpp)

# 5. 链接依赖库：把我们的代码和ROS2的库链接起来
ament_target_dependencies(odom_subscriber
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# 6. 安装可执行文件：把编译好的文件放到install目录，让ROS2能找到
install(TARGETS
  odom_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# 7. 结束（必须有这一行）
ament_package()
