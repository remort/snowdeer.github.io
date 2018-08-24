---
layout: post
title: 키보드를 이용해서 Service Request를 날리고 이벤트를 Subscription하는 예제(Python)
category: ROS2
tag: [ROS]
---

# 키보드를 이용해서 Service Request를 날리고 이벤트를 Subscription하는 예제

<pre class="prettyprint">
import threading
import rclpy, os
import datetime

from tts_service.srv._tts_request import TtsRequest
from tts_service.msg import TtsEvent

NODE_NAME = "tts_service_controller"

client = None;


def callback_on_tts_event_arrived(msg):
    curTime = datetime.datetime.now()
    if msg.event == TtsEvent.EVENT_ON_TTS_STARTED:
        print("[{0}] EVENT_TTS_STARTED".format(curTime))
    elif msg.event == TtsEvent.EVENT_ON_TTS_FINISHED:
        print("[{0}] EVENT_TTS_FINISHED".format(curTime))


def handle_keyboard():
    global client

    while True:
        print("\n<TTS Service Control Menu>")
        print("   1. Request to Speech Text ('Hello')")
        print("   99. Exit")

        menu = input('Input the menu: ')

        if menu == '1':
            req = TtsRequest.Request()
            req.text = "Hello"
            client.call_async(req)

        elif menu == '99':
            rclpy.shutdown()
            os._exit(1)


def main(args=None):
    global client

    rclpy.init(args=args)

    node = rclpy.create_node(NODE_NAME)
    client = node.create_client(TtsRequest, TtsRequest.Request.NAME)
    subscription = node.create_subscription(TtsEvent, TtsEvent.NAME, callback_on_tts_event_arrived)

    th = threading.Thread(target=handle_keyboard)
    th.start()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
</pre>