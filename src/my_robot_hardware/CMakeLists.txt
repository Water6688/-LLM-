cmake_minimum_required(VERSION 3.8)
project(my_robot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(fmt REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../feetech_ros2_driver/feetech_driver/include
)

set(FEETECH_DRIVER_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../build/feetech_ros2_driver/feetech_driver)
set(FEETECH_COMM_LIB ${FEETECH_DRIVER_LIB_DIR}/libcommunication_protocol.so)
set(FEETECH_SERIAL_LIB ${FEETECH_DRIVER_LIB_DIR}/libserial_port.so)

add_library(${PROJECT_NAME} SHARED src/my_robot_hardware.cpp)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

target_link_libraries(${PROJECT_NAME} ${FEETECH_COMM_LIB} ${FEETECH_SERIAL_LIB})

add_executable(sts3215_example src/sts3215_example.cpp)
target_include_directories(
  sts3215_example
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(sts3215_example ${FEETECH_COMM_LIB} ${FEETECH_SERIAL_LIB} fmt::fmt)

add_executable(set_position_example src/set_position_example.cpp)
target_include_directories(
  set_position_example
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(set_position_example ${FEETECH_COMM_LIB} ${FEETECH_SERIAL_LIB} fmt::fmt)

add_executable(set_arm_center src/set_arm_center.cpp)
target_include_directories(
  set_arm_center
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(set_arm_center ${FEETECH_COMM_LIB} ${FEETECH_SERIAL_LIB} fmt::fmt)

pluginlib_export_plugin_description_file(hardware_interface my_robot_hardware_plugin_description.xml)

install(
  TARGETS ${PROJECT_NAME} sts3215_example set_position_example set_arm_center
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES my_robot_hardware_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_lifecycle hardware_interface pluginlib feetech_driver)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
