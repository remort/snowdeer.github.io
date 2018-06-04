layout: post
title: ROS2 Simple Service Receiver on Python
category: ROS2
tag: [ROS, Python]
---
# ROS2 Simple Service Receiver on Python

간단한 ROS2 Service를 수신하고, 커스텀 메시지 배열을 Publish 하는 예제입니다.

<br>

<pre class="prettyprint">
from simple_service.srv import SimpleService
from simple_service.msg import Position
from simple_service.msg import RoutingData

import rclpy

global g_publisher


def add_position(positions, x, y, theta):
    pos = Position()
    pos.x = x;
    pos.y = y;
    pos.theta = theta
    positions.append(pos)


def callback(request, response):
    request_id = request.request_id
    print("Simple Service Request: {0} ".format(request_id))

    if request_id == Navigation.Request.REQUEST_DEMO_MOVE_PATH_1:
        msg = RoutingData()
        msg.command = RoutingData.COMMAND_MOVE_ALONG_PATH

        add_position(msg.positions, 100, 100, 10)
        add_position(msg.positions, 500, 200, 10)
        add_position(msg.positions, 1000, 1000, 1)

        g_publisher.publish(msg)

    elif request_id == Navigation.Request.REQUEST_DEMO_MOVE_PATH_2:
        msg = RoutingData()
        msg.command = RoutingData.COMMAND_MOVE_ALONG_PATH

        add_position(msg.positions, -100, -100, 10)
        add_position(msg.positions, -500, -200, 10)
        add_position(msg.positions, -1000, -1000, 1)

        g_publisher.publish(msg)

    response.response = 200

    return response


def main(args=None):
    global g_publisher

    rclpy.init(args=args)

    node = rclpy.create_node('simple_service_receiver')
    g_publisher = node.create_publisher(RoutingData, RoutingData.NAME)

    srv = node.create_service(SimpleService, SimpleService.Request.NAME, callback)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
</pre>