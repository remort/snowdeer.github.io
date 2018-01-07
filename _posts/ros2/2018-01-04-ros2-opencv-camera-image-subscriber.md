---
layout: post
title: OpenCV 카메라 이미지를 Subscribe 하는 예제
category: ROS2
tag: [ROS, OpenCV]
---
# 카메라 이미지 Subscriber

원본 소스 참고는 [여기](https://github.com/ros2/demos/blob/master/image_tools/src/showimage.cpp)를 했습니다.

## main.cpp

<pre class="prettyprint">
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include &lt;cstdio&gt;
#include &lt;memory&gt;
#include &lt;string&gt;

using namespace std;
using namespace cv;

#define ROS_NODE_NAME "usb_camera_subscriber"
#define ROS_CAMERA_TOPIC_NAME "snowdeer_camera_topic"


void (*breakCapture)(int);
void signalingHandler(int signo) {
  printf("'Ctrl + C'(%d) processing...\n", signo);

  exit(1);
}

int encoding2mat_type(const string & encoding) {
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  } else if (encoding == "bgra8") {
    return CV_8UC4;
  } else if (encoding == "32FC1") {
    return CV_32FC1;
  } else if (encoding == "rgb8") {
    return CV_8UC3;
  } else {
    throw std::runtime_error("Unsupported encoding type");
  }
}

string mat_type2encoding(int mat_type) {
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

void show_image(const sensor_msgs::msg::Image::SharedPtr msg) {
  printf("Received image #%s\n", msg->header.frame_id.c_str());

  cv::Mat frame(msg->height, msg->width, encoding2mat_type(msg->encoding),
      const_cast&lt;unsigned char *&gt;(msg->data.data()), msg->step);

  CvMat cvframe;
  if (msg->encoding == "rgb8") {
    cv::Mat frame2;
    cv::cvtColor(frame, frame2, cv::COLOR_RGB2BGR);
    cvframe = frame2;
  } else {
    cvframe = frame;
  }

  cvShowImage("Received Image", &cvframe);
  if( waitKey(10) == 27 ) exit(1);
}

void callback_from_image_topic(const sensor_msgs::msg::Image::SharedPtr msg) {
  show_image(msg);
}

int main(int argc, char** argv) {
  breakCapture = signal(SIGINT, signalingHandler);

  rclcpp::init(argc, argv);

  rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_HISTORY_KEEP_ALL;

  size_t depth = 10;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  auto node = rclcpp::Node::make_shared(ROS_NODE_NAME);
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = depth;
  custom_qos_profile.reliability = reliability_policy;
  custom_qos_profile.history = history_policy;

  auto sub = node->create_subscription&lt;sensor_msgs::msg::Image&gt;(
      ROS_CAMERA_TOPIC_NAME, callback_from_image_topic, custom_qos_profile);

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
</pre>