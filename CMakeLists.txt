cmake_minimum_required(VERSION 3.16)
project(fotonic_3dcamera)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

message(STATUS "Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
if( ${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86_64" )
  set(FZ_API ${PROJECT_SOURCE_DIR}/lib/x86_64/libfz_api.so.1)
else()
  set(FZ_API ${PROJECT_SOURCE_DIR}/lib/i386/libfz_api.so.1)
endif()

add_library(fotonic_3dcamera_node SHARED 
  src/fotonic_3dcamera.cpp
  src/camera.cpp)

target_include_directories(
  fotonic_3dcamera_node
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(
  fotonic_3dcamera_node
  rclcpp::rclcpp
  rcutils::rcutils
  camera_info_manager::camera_info_manager
  cv_bridge::cv_bridge
  image_transport::image_transport
  tf2::tf2
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${FZ_API}
)
  
add_library(fotonic_3dcamera SHARED src/component.cpp)
target_include_directories(
  fotonic_3dcamera
  PRIVATE
  include
)
target_link_libraries(
  fotonic_3dcamera
  fotonic_3dcamera_node
  rclcpp_components::component
)
  
rclcpp_components_register_node(
  fotonic_3dcamera
  PLUGIN "fotonic::Camera3D" # edit namespace if needed
  EXECUTABLE fotonic_3dcamera_executable
)
  
install(
  TARGETS fotonic_3dcamera fotonic_3dcamera_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(DIRECTORY lib DESTINATION share/${PROJECT_NAME})

install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
  
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
  
ament_export_dependencies(
  rclcpp
  rclcpp_components
  rcutils
  camera_info_manager
  cv_bridge
  image_transport
  geometry_msgs
  std_msgs
  sensor_msgs
  tf2)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
  
ament_package()
