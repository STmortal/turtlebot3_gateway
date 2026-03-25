cmake_minimum_required(VERSION 3.8)
project(turtlebot3_gateway)

# 编译选项：开启警告，C++17标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 找到依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# 添加头文件目录
include_directories(
  include
)

# ====================== 第二天的里程计订阅者节点（保留） ======================
add_executable(odom_subscriber src/odom_subscriber.cpp)
ament_target_dependencies(odom_subscriber
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)
install(TARGETS
  odom_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# ====================== 第四天新增：传感器驱动库 ======================
# 把驱动代码编译成静态库，方便后续复用
add_library(sensor_driver_lib STATIC
  src/lidar_driver.cpp
  src/imu_driver.cpp
)
ament_target_dependencies(sensor_driver_lib
  rclcpp
  sensor_msgs
)
# 让其他节点能找到头文件
target_include_directories(sensor_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ====================== 第四天新增：驱动测试节点 ======================
add_executable(sensor_driver_test src/sensor_driver_test.cpp)
# 链接我们编译的驱动库
target_link_libraries(sensor_driver_test sensor_driver_lib)
ament_target_dependencies(sensor_driver_test
  rclcpp
  sensor_msgs
)
install(TARGETS
  sensor_driver_test
  DESTINATION lib/${PROJECT_NAME}
)

# ====================== 安装头文件 ======================
install(
  DIRECTORY include/
  DESTINATION include
)

# ====================== 测试与安装 ======================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
