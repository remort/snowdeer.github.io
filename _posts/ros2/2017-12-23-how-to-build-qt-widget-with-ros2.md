---
layout: post
title: ROS 2.0 기반 QT Widget 빌드용 CMakeFileList.txt
category: ROS2
tag: [ROS, QT]
---
# ROS 2.0 with QT

ROS 2.0에서 QT Widget 프로그램을 빌드하는 방법에 대해 포스팅합니다.

먼저 QT Widget 샘플 코드입니다. QTCreator를 이용해서 생성한 프로젝트라서 `mainwindow.h`, `mainwindow.cpp`, `mainwindow.ui` 등의 파일도 생성이 되는데 여기서는 `CMakeLists.txt` 파일에 대해서만 포스팅합니다.

## CMakeLists.txt

<pre class="prettyprint">
cmake_minimum_required(VERSION 3.5)
project(qt_widget)

set(REQUIRED_QT_VERSION 5.9.0)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(${PROJECT_NAME} main.cpp mainwindow.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp Qt5Core Qt5Widgets)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
</pre>