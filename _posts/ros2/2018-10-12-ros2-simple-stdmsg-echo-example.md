---
layout: post
title: 문자열(std_msg)을 수신(Subscribe)하고 전송(Publish)하는 예제 코드(C++)
category: ROS2
tag: [ROS]
---

# 문자열(std_msg)을 수신(Subscribe)하고 전송(Publish)하는 예제 코드(C++)

하나의 프로그램내에서 ROS 2.0 문자열 메시지를 수신하고, 바로 그 내용을 다른 토픽(Topic)으로 전송하는 예제 코드입니다. 메시지 수신부는 람다 함수로 되어 있습니다. 

<pre class="prettyprint">
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include &lt;iostream&gt;

using namespace std;
using std::placeholders::_1;

const string NODE_NAME = "snowdeer_msg_echo_example";

int main(int argc, char *argv[]) {
  cout << "Hello, ROS2 ECHO Example" << std::endl;

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(NODE_NAME);
  auto publisher = node->create_publisher&lt;std_msgs::msg::String&gt;("snowdeer_output");

  auto subscription = node->create_subscription&lt;std_msgs::msg::String&gt;
      ("snowdeer_input", [publisher](std_msgs::msg::String::SharedPtr inMessage) {
        auto outMessage = std::make_shared&lt;std_msgs::msg::String&gt;();
        outMessage->data = inMessage->data;
        publisher->publish(outMessage);
      });

  auto message = std::make_shared&lt;std_msgs::msg::String&gt;();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
</pre>