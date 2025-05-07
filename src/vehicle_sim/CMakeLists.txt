cmake_minimum_required(VERSION 3.8)
project(vehicle_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# vehicle_sim_node
add_executable(
        vehicle_sim_node src/vehicle_sim_node.cpp
        src/include/Vehicle2DData.cpp
        src/include/TireConfig.cpp
        src/include/VehicleInput.cpp
        src/include/Vehicle2DConfig.cpp
        src/include/Vehicle2D.cpp
)
ament_target_dependencies(
        vehicle_sim_node
        rclcpp
        sensor_msgs
        nav_msgs
        tf2
        tf2_ros
        ament_index_cpp
)
target_include_directories(vehicle_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/include>
  $<INSTALL_INTERFACE:include>
  )
target_compile_features(vehicle_sim_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS
  vehicle_sim_node
  DESTINATION lib/${PROJECT_NAME}
)

# static_transform_publisher
add_executable(static_transform_publisher src/static_transform_publisher.cpp)
ament_target_dependencies(
        static_transform_publisher
        geometry_msgs
        rclcpp
        tf2
        tf2_ros
)
target_include_directories(static_transform_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/include>
  $<INSTALL_INTERFACE:include>
  )
target_compile_features(static_transform_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS
  static_transform_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# common
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
