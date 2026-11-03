cmake_minimum_required(VERSION 3.8)
project(rov_vision)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc ximgproc calib3d videoio)
find_package(rosidl_default_generators REQUIRED)
include_directories(include)

add_executable(tag_velocity_estimator src/tag_velocity_estimator.cpp)
ament_target_dependencies(tag_velocity_estimator
  rclcpp
  geometry_msgs
  visualization_msgs
  Eigen3
)
target_include_directories(
  tag_velocity_estimator PUBLIC include)

add_executable(camera_info_publisher src/camera_info_publisher.cpp)
ament_target_dependencies(camera_info_publisher
  rclcpp
  sensor_msgs
  camera_info_manager
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(stereo_triangulator src/stereo_triangulator.cpp)
ament_target_dependencies(stereo_triangulator
  rclcpp
  geometry_msgs
  Eigen3
  apriltag_msgs
  sensor_msgs
  message_filters
)
target_include_directories(
  stereo_triangulator PUBLIC include)


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetStreamMode.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(depth_estimation
  src/depth_estimation.cpp
  src/depth_estimation_main.cpp
)
ament_target_dependencies(depth_estimation rclcpp sensor_msgs std_msgs cv_bridge)
target_include_directories(depth_estimation PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_estimation ${OpenCV_LIBS})

add_executable(streamer
  src/streamer.cpp
  src/streamer_main.cpp
)
ament_target_dependencies(streamer rclcpp sensor_msgs std_msgs cv_bridge)
target_include_directories(streamer PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(streamer ${cpp_typesupport_target} ${OpenCV_LIBS})

install(TARGETS camera_info_publisher stereo_triangulator tag_velocity_estimator
  depth_estimation streamer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config include
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
