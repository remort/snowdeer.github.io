---
layout: post
title: Simple Message Publisher 및 Subscriber 예제 (Node 클래스 상속)
category: ROS2
tag: [ROS]
---

간단하게 메시지를 발행/수신하는 예제 코드는 다음과 같습니다.

# snow_publisher_using_class

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
project(snow_publisher_using_class)

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

class SnowPublisher : public rclcpp::Node {
public:
  SnowPublisher()
  : Node("minimal_publisher") {
    mCount = 0;
    mPublisher = this->create_publisher&lt;std_msgs::msg::String&gt;("snowdeer_channel");
    mTimer = this->create_wall_timer(
      1000ms, std::bind(&SnowPublisher::publish_message, this));
  }

private:
  int mCount;
  rclcpp::TimerBase::SharedPtr mTimer;
  rclcpp::Publisher&lt;std_msgs::msg::String&gt;::SharedPtr mPublisher;
  
  void publish_message();
 
};

void SnowPublisher::publish_message() {
  mCount++;

  auto message = std_msgs::msg::String();
  message.data = "Hello, SnowDeer! " + std::to_string(mCount);

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str())
  mPublisher->publish(message);
}

int main(int argc, char * argv[]) {
  
  printf("This is SnowDeer's ROS Message Publisher.\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared&lt;SnowPublisher&gt;());
  rclcpp::shutdown();

  return 0;
}
</pre>

<br>

# snow_subscriber_using_class

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
project(snow_subscriber_using_class)

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
using std::placeholders::_1;

class SnowSubscriber : public rclcpp::Node {
public:
  SnowSubscriber()
  : Node("snowdeer_msg_subscriber") {
    mSubscriber = this->create_subscription&lt;std_msgs::msg::String&gt;(
      "snowdeer_channel", std::bind(&SnowSubscriber::receive_message, this, _1));
  }

private:
  rclcpp::Subscription&lt;std_msgs::msg::String&gt;::SharedPtr mSubscriber;

  void receive_message(const std_msgs::msg::String::SharedPtr msg);
  
};

void SnowSubscriber::receive_message(const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received Message: '%s'", msg->data.c_str())
}

int main(int argc, char * argv[]) {
  
  printf("This is SnowDeer's ROS Message Subscriber.\n");

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared&lt;SnowSubscriber&gt;());
  rclcpp::shutdown();

  return 0;
}
</pre>