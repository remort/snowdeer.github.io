---
layout: post
title: ROS 2.0 Service-Client 샘플 코드
category: ROS2
tag: [ROS]
---

# Client

<pre class="prettyprint">
#include "rclcpp/rclcpp.hpp"
#include "snowdeer_msgs/srv/add.hpp"
#include &lt;string&gt;

using namespace rclcpp;
using namespace snowdeer_msgs::srv;
using namespace std;

int main(int argc, char **argv) {
  cout << "App1" << endl;
  rclcpp::init(argc, argv);

  auto node = Node::make_shared("snowdeer_app1");
  auto client = node->create_client&lt;Add&gt;("snowdeer_sample_channel");

  thread t1([client]() {
    while (true) {
      auto a = 0;
      auto b = 0;

      cout << "Input two number: " << endl;
      cin >> a >> b;

      auto req = make_shared&lt;Add::Request&gt;();
      req->a = a;
      req->b = b;

      auto tp = std::chrono::system_clock::now() + std::chrono::seconds(3);
      auto request = client->async_send_request(req);
      auto status = request.wait_until(tp);

      if (status == future_status::ready) {
        auto resp = request.get();
        cout << "Sum : " << resp->sum << endl;
      } else {
        cout << "Timeout !!!" << endl;
      }
    }
  });

  rclcpp::WallRate loop_rate(100ms);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    loop_rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}
</pre>

<br>

# Service

<pre class="prettyprint">
#include "rclcpp/rclcpp.hpp"
#include "snowdeer_msgs/srv/add.hpp"
#include &lt;string&gt;

using namespace rclcpp;
using namespace snowdeer_msgs::srv;
using namespace std;

int main(int argc, char **argv) {
  cout << "App3" << endl;
  rclcpp::init(argc, argv);

  auto node = Node::make_shared("snowdeer_app2");
  auto service = node->create_service&lt;Add&gt;("snowdeer_sample_channel", [](const shared_ptr&lt;rmw_request_id_t&gt; request_header,
                                                                 const shared_ptr&lt;Add::Request&gt; request,
                                                                 const shared_ptr&lt;Add::Response&gt; response) {
    cout << "Request(" << request->a << ", " << request->b << ")" << endl;
    response->sum = request->a + request->b;
    cout << "Return : " << response->sum << endl;
  });

  rclcpp::WallRate loop_rate(100ms);
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    loop_rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}
</pre>