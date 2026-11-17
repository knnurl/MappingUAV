# filepath: src/map_interface/CMakeLists.txt
cmake_minimum_required(VERSION 3.8)
project(map_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(dynamicEDT3D REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/QueryMap.srv"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}
  rosidl_typesupport_cpp)

add_executable(map_server_node src/map_server_node.cpp)
target_include_directories(map_server_node PRIVATE
  include
  ${OCTOMAP_INCLUDE_DIRS}
  ${DYNAMICEDT3D_INCLUDE_DIRS})
target_link_libraries(map_server_node
  ${OCTOMAP_LIBRARIES}
  ${DYNAMICEDT3D_LIBRARIES}
  "${cpp_typesupport_target}")
ament_target_dependencies(map_server_node
  rclcpp sensor_msgs nav_msgs geometry_msgs octomap_msgs)

add_executable(map_saver_node src/map_saver_node.cpp)
target_include_directories(map_saver_node PRIVATE include)
ament_target_dependencies(map_saver_node rclcpp sensor_msgs nav_msgs)
install(TARGETS map_server_node map_saver_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_map_core test/test_map_core.cpp)
  target_include_directories(test_map_core PRIVATE
    include
    ${OCTOMAP_INCLUDE_DIRS}
    ${DYNAMICEDT3D_INCLUDE_DIRS})
  target_link_libraries(test_map_core
    ${OCTOMAP_LIBRARIES}
    ${DYNAMICEDT3D_LIBRARIES})
  ament_add_gtest(test_keyframes test/test_keyframes.cpp)
  target_include_directories(test_keyframes PRIVATE include)
  ament_add_gtest(test_voxel_accumulator test/test_voxel_accumulator.cpp)
  target_include_directories(test_voxel_accumulator PRIVATE include)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
