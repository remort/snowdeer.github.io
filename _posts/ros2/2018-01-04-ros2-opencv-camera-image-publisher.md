---
layout: post
title: OpenCV 카메라 이미지를 Publish 하는 예제
category: ROS2
tag: [ROS, OpenCV]
---
# 카메라 이미지 Publisher

원본 소스 참고는 [여기](https://github.com/ros2/demos/blob/master/image_tools/src/cam2image.cpp)를 했습니다.

## main.cpp

<pre class="prettyprint">
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

#include &lt;cstdio&gt;
#include &lt;memory&gt;
#include &lt;string&gt;

using namespace std;
using namespace cv;

#define ROS_NODE_NAME "usb_camera_publisher"
#define ROS_CAMERA_TOPIC_NAME "snowdeer_camera_topic"

void (*breakCapture)(int);
void signalingHandler(int signo) {
  printf("'Ctrl + C'(%d) processing...\n", signo);

  exit(1);
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

void convert_frame_to_message(const cv::Mat & frame, size_t frame_id, 
  sensor_msgs::msg::Image::SharedPtr msg) {
  
  msg->height = frame.rows;
  msg->width = frame.cols;
  msg->encoding = mat_type2encoding(frame.type());
  msg->step = static_cast&lt;sensor_msgs::msg::Image::_step_type&gt;(frame.step);
  size_t size = frame.step * frame.rows;
  msg->data.resize(size);
  memcpy(&msg->data[0], frame.data, size);
  msg->header.frame_id = std::to_string(frame_id);
}

int main(int argc, char** argv) {
  breakCapture = signal(SIGINT, signalingHandler);

  rclcpp::init(argc, argv);

  rmw_qos_reliability_policy_t reliability_policy = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  rmw_qos_history_policy_t history_policy = RMW_QOS_POLICY_HISTORY_KEEP_ALL;

  size_t depth = 10;
  double freq = 30.0;
  size_t width = 320;
  size_t height = 240;
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  
  auto node = rclcpp::Node::make_shared(ROS_NODE_NAME);
  rmw_qos_profile_t custom_camera_qos_profile = rmw_qos_profile_default;
  custom_camera_qos_profile.depth = depth;
  custom_camera_qos_profile.reliability = reliability_policy;
  custom_camera_qos_profile.history = history_policy;

  auto publisher1 = node->create_publisher&lt;sensor_msgs::msg::Image&gt;(
    ROS_CAMERA_TOPIC_NAME, custom_camera_qos_profile);

  rclcpp::WallRate loop_rate(freq);

  VideoCapture cap;
  if(!cap.open(0)) return 0;
  
  cap.set(CV_CAP_PROP_FRAME_WIDTH, static_cast&lt;double&gt;(width));
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, static_cast&lt;double&gt;(height));

  if (!cap.isOpened()) {
    printf("Could not open video stream!!\n");
    return 1;
  }
  
  Mat frame;
  auto msg = std::make_shared&lt;sensor_msgs::msg::Image&gt;();
  msg->is_bigendian = false;

  size_t i = 1;

  while (rclcpp::ok()) {
    cap >> frame;

    if (!frame.empty()) {
      convert_frame_to_message(frame, i, msg);
    }

    if (!frame.empty()) {
      CvMat cvframe = frame;
      cvShowImage("Camera", &cvframe);
      if( waitKey(10) == 27 ) break;

      printf("Publishing image #%zd\n", i);
      publisher->publish(msg);
      ++i;

      if(i > 10000000) i = 0;
    }

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
</pre>