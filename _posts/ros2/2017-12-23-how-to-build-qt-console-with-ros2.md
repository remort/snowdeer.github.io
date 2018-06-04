---
layout: post
title: QT Console 프로그램을 ROS 2.0으로 빌드하기
category: ROS2
tag: [ROS, QT]
---
# ROS 2.0 with QT

ROS 2.0에서 QT 콘솔 프로그램을 빌드하는 방법에 대해 포스팅합니다.

먼저 QT 콘솔 샘플 코드입니다.

## main.cpp

<pre class="prettyprint">
#include &lt;QCoreApplication&gt;
#include &lt;QDebug&gt;

int main(int argc, char *argv[]) {
  QCoreApplication a(argc, argv);

  qDebug() << "snowdeer QT Console";

  return a.exec();
}
</pre>

<br>

## CMakeLists.txt

<pre class="prettyprint">
cmake_minimum_required(VERSION 3.5)
project(qt_console)

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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp Qt5Core)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
</pre>