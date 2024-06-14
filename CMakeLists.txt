cmake_minimum_required(VERSION 3.8)
project(gst_test)

# Set the C and C++ standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ffmpeg_image_transport_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Find GStreamer
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

# Include directories
include_directories(
  include
  ${GST_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Build
add_executable(gst_test src/gst_test.cpp)
ament_target_dependencies(gst_test
  rclcpp
  std_msgs
  sensor_msgs
  ffmpeg_image_transport_msgs
  cv_bridge
  image_transport
)

# Link GStreamer and OpenCV libraries
target_link_libraries(gst_test
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Install
install(TARGETS
  gst_test
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
