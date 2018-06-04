---
layout: post
title: ROS2 Simple Keyboard Publisher on Python
category: ROS2
tag: [ROS, Python]
---
# ROS2 Simple Keyboard Publisher on Python

<br>

## package.xml

<pre class="prettyprint">
&lt;?xml version="1.0"?&gt;
&lt;?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?&gt;
&lt;package format="2"&gt;
  &lt;name&gt;simple_publisher&lt;/name&gt;
  &lt;version&gt;0.4.0&lt;/version&gt;
  &lt;description&gt;Simple Publisher on Python&lt;/description&gt;
  &lt;maintainer email="snowdeer0314@gmail.com"&gt;snowdeer&lt;/maintainer&gt;
  &lt;license&gt;Apache License 2.0&lt;/license&gt;

  &lt;exec_depend&gt;rclpy&lt;/exec_depend&gt;
  &lt;exec_depend&gt;std_msgs&lt;/exec_depend&gt;
  &lt;exec_depend&gt;simple_message&lt;/exec_depend&gt;

  &lt;export&gt;
    &lt;build_type&gt;ament_python&lt;/build_type&gt;
  &lt;/export&gt;
&lt;/package&gt;
</pre>

<br>

## setup.cfg

<pre class="prettyprint">
[develop]
script-dir=$base/bin/simple_publisher
[install]
install-scripts=$base/bin/simple_publisher
</pre>

<br>

## setup.py

<pre class="prettyprint">
from setuptools import setup

package_name = 'simple_publisher'

setup(
    name=package_name,
    version='0.4.0',
    packages=[],
    py_modules=[
        'simple_publisher'],
    install_requires=['setuptools'],
    author='snowdeer ',
    author_email='snowdeer0314@gmail.com',
    maintainer='snowdeer',
    maintainer_email='snowdeer0314@gmail.com',
    keywords=['ROS'],
    description='Simple Publisher on Python',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = simple_publisher:main'
        ],
    },
)
</pre>

<br>

## simple_publisher.py

<pre class="prettyprint">
import threading
import rclpy
import os

from simple_message.msg import SimpleMessage

NODE_NAME = "simple_publisher"


def handle_keyboard(publisher):
    while True:
        print('\n- Simple Publisher Menu -')
        print('   1. Command (Move along path 1)')
        print('   2. Command (Move along path 2)')
        print('   99. Exit')

        menu = input('Input the menu: ')

        if menu == '1':
            msg = SimpleMessage()
            msg.command_id = SimpleMessage.COMMAND_FOR_DEMO_1
            publisher.publish(msg)
            print(" \n>>> command is published : {0}".format(msg.command_id))

        elif menu == '2':
            msg = SimpleMessage()
            msg.command_id = SimpleMessage.COMMAND_FOR_DEMO_2
            publisher.publish(msg)
            print(" \n>>> command is published : {0}".format(msg.command_id))

        elif menu == '99':
            rclpy.shutdown()
            os._exit(1)


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(NODE_NAME)
    publisher = node.create_publisher(SimpleMessage, SimpleMessage.NAME)

    th = threading.Thread(target=handle_keyboard, args=(publisher,))
    th.start()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
</pre>