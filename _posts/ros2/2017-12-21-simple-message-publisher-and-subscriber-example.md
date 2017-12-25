---
layout: post
title: Simple Message Publisher 및 Subscriber 예제
category: ROS2
tag: [ROS]
---

간단하게 메시지를 발행/수신하는 예제 코드는 다음과 같습니다.

# snowdeer_publisher

## package.xml

<pre class="prettyprint">
&lt;?xml version="1.0"?&gt;
&lt;?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?&gt;
&lt;package format="2"&gt;
  &lt;name&gt;snow_publisher&lt;/name&gt;
  &lt;version&gt;0.4.0&lt;/version&gt;
  &lt;description&gt;Snow ROS Message Publisher&lt;/description&gt;
  &lt;maintainer email="snowdeer0314@gmail.com"&gt;snowdeer&lt;/maintainer&gt;
  &lt;license&gt;Apache License 2.0&lt;/license&gt;

  &lt;buildtool_depend&gt;ament_cmake&lt;/buildtool_depend&gt;

  &lt;build_depend&gt;rclcpp&lt;/build_depend&gt;
  &lt;build_depend&gt;std_msgs&lt;/build_depend&gt;

  &lt;exec_depend&gt;rclcpp&lt;/exec_depend&gt;
  &lt;exec_depend&gt;std_msgs&lt;/exec_depend&gt;
  
  &lt;export&gt;
    &lt;build_type&gt;ament_cmake&lt;/build_type&gt;
  &lt;/export&gt;
&lt;/package&gt;
</pre>

<br>

## CMakeLists.txt

<pre class="prettyprint">
cmake_minimum_required(VERSION 3.5)
project(snow_publisher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
</pre>

<br>

## main.cpp

<pre class="prettyprint">
#include &lt;cstdio&gt;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
  
  printf("This is SnowDeer's ROS Message Publisher.\n");

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("snowdeer_msg_publisher");
  auto publisher = node->create_publisher&lt;std_msgs::msg::String&gt;("snowdeer_topic");
  auto message = std::make_shared&lt;std_msgs::msg::String&gt;();
  auto publish_count = 0;

  rclcpp::WallRate loop_rate(1000ms);

  while (rclcpp::ok()) {
    message->data = "Hello, SnowDeer! " + std::to_string(publish_count++);

    RCLCPP_INFO(node->get_logger(), "Publishing: '%s'", message->data.c_str())
    publisher->publish(message);
    
    rclcpp::spin_some(node);

    loop_rate.sleep();
  }

  rclcpp::shutdown();
  
  return 0;
}
</pre>

<br>

# snowdeer_subscriber

## package.xml

<pre class="prettyprint">
&lt;?xml version="1.0"?&gt;
&lt;?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?&gt;
&lt;package format="2"&gt;
  &lt;name&gt;snow_subscriber&lt;/name&gt;
  &lt;version&gt;0.4.0&lt;/version&gt;
  &lt;description&gt;Snow ROS Message Subscriber&lt;/description&gt;
  &lt;maintainer email="snowdeer0314@gmail.com"&gt;snowdeer&lt;/maintainer&gt;
  &lt;license&gt;Apache License 2.0&lt;/license&gt;

  &lt;buildtool_depend&gt;ament_cmake&lt;/buildtool_depend&gt;

  &lt;build_depend&gt;rclcpp&lt;/build_depend&gt;
  &lt;build_depend&gt;std_msgs&lt;/build_depend&gt;

  &lt;exec_depend&gt;rclcpp&lt;/exec_depend&gt;
  &lt;exec_depend&gt;std_msgs&lt;/exec_depend&gt;
  
  &lt;export&gt;
    &lt;build_type&gt;ament_cmake&lt;/build_type&gt;
  &lt;/export&gt;
&lt;/package&gt;
</pre>

<br>

## CMakeLists.txt

<pre class="prettyprint">
cmake_minimum_required(VERSION 3.5)
project(snow_subscriber)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
</pre>

<br>

## main.cpp

<pre class="prettyprint">
#include &lt;cstdio&gt;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;

rclcpp::Node::SharedPtr node = nullptr;

void callback_from_topic(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(node->get_logger(), "Received Message: '%s'", msg->data.c_str())
}

int main(int argc, char * argv[]) {
  
  printf("This is SnowDeer's ROS Message Subscriber.\n");

  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("snowdeer_msg_subscriber");

  auto subscription = node->create_subscription&ltstd_msgs::msg::String&gt;
      ("snowdeer_topic", callback_from_topic);

  rclcpp::spin(node);
  rclcpp::shutdown();

  node = nullptr;

  return 0;
}
</pre>