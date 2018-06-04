---
layout: post
title: OpenCV 라이브러리를 사용하는 ROS 2.0 빌드용 CMakeFileList.txt
category: ROS2
tag: [ROS, OpenCV]
---
# ROS 2.0 with OpenCV

## CMakeLists.txt

<pre class="prettyprint">
cmake_minimum_required(VERSION 3.5)

project(usb_camera_publisher)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(${PROJECT_NAME} 
  "rclcpp" 
  "sensor_msgs"
  "OpenCV")

install(TARGETS ${PROJECT_NAME}
  DESTINATION bin/${PROJECT_NAME})

ament_package()
</pre>

위에서 `sensor_msgs` 라이브러리는 ROS 2.0에서 사용하는 이미지의 데이터 타입 `Image`를 사용하기 위해서 추가했습니다.