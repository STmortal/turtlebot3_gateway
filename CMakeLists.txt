cmake_minimum_required(VERSION 3.8)
project(turtlebot3_gateway)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pthread -g -O0)
endif()

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
include_directories(include)

# ====================== 核心驱动库 ======================
add_library(sensor_driver_lib STATIC
  src/lidar_driver.cpp
  src/imu_driver.cpp
  src/camera_driver.cpp
  src/sensor_manager.cpp
)
ament_target_dependencies(sensor_driver_lib
  rclcpp
  sensor_msgs
)
target_include_directories(sensor_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sensor_driver_lib pthread)

# ====================== 基础节点（保留） ======================
# 第二天的里程计订阅者节点
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

# 第四天的单传感器驱动测试节点
add_executable(sensor_driver_test src/sensor_driver_test.cpp)
target_link_libraries(sensor_driver_test sensor_driver_lib)
ament_target_dependencies(sensor_driver_test
  rclcpp
  sensor_msgs
)
install(TARGETS
  sensor_driver_test
  DESTINATION lib/${PROJECT_NAME}
)

# 第六天的多线程采集测试节点
add_executable(multithread_collect_test src/multithread_collect_test.cpp)
target_link_libraries(multithread_collect_test sensor_driver_lib)
ament_target_dependencies(multithread_collect_test
  rclcpp
  sensor_msgs
)
install(TARGETS
  multithread_collect_test
  DESTINATION lib/${PROJECT_NAME}
)

# ====================== 第八天新增：GTest单元测试 ======================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # 1. 环形缓冲区测试
  ament_add_gtest(test_ring_buffer test/test_ring_buffer.cpp)
  target_link_libraries(test_ring_buffer sensor_driver_lib)
  ament_target_dependencies(test_ring_buffer rclcpp sensor_msgs)

  # 2. 传感器驱动测试
  ament_add_gtest(test_sensor_drivers test/test_sensor_drivers.cpp)
  target_link_libraries(test_sensor_drivers sensor_driver_lib)
  ament_target_dependencies(test_sensor_drivers rclcpp sensor_msgs)

  # ========== 【关键】取消下面4行的注释，开启覆盖率 ==========
  target_compile_options(test_ring_buffer PRIVATE --coverage)
  target_compile_options(test_sensor_drivers PRIVATE --coverage)
  target_link_options(test_ring_buffer PRIVATE --coverage)
  target_link_options(test_sensor_drivers PRIVATE --coverage)
endif()


# ====================== 安装头文件 ======================
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
