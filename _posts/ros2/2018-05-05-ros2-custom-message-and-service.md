layout: post
title: ROS2 커스텀 Message 및 Service
category: ROS2
tag: [ROS, Python]
---
# ROS2 커스텀 Message 및 Service

ROS2에서 사용자 정의 Message와 Service를 정의하는 방법입니다.

<br>

## package.xml

<pre class="prettyprint">
&lt;?xml version="1.0"?&gt;
&lt;?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?&gt;
&lt;package format="3"&gt;
  &lt;name&gt;tree_service&lt;/name&gt;
  &lt;version&gt;0.4.0&lt;/version&gt;
  &lt;description&gt;Message and Service of Tree-Service&lt;/description&gt;
  &lt;maintainer email="snowdeer0314@gmail.com"&gt;snowdeer&lt;/maintainer&gt;
  &lt;license&gt;Apache License 2.0&lt;/license&gt;

  &lt;buildtool_depend&gt;ament_cmake&lt;/buildtool_depend&gt;
  
  &lt;build_depend&gt;builtin_interfaces&lt;/build_depend&gt;
  &lt;build_depend&gt;rosidl_default_generators&lt;/build_depend&gt;
  
  &lt;exec_depend&gt;builtin_interfaces&lt;/exec_depend&gt;
  &lt;exec_depend&gt;rosidl_default_runtime&lt;/exec_depend&gt;

  &lt;member_of_group&gt;rosidl_interface_packages&lt;/member_of_group&gt;

  &lt;export&gt;
    &lt;build_type&gt;ament_cmake&lt;/build_type&gt;
  &lt;/export&gt;
&lt;/package&gt;
</pre>

<br>

## CMakeLists.txt

<pre class="prettyprint">
cmake_minimum_required(VERSION 3.5)

project(tree_service)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
    "msg/TreeEvent.msg"
    "msg/TreeData.msg"
    "msg/TreeNode.msg"
)

set(srv_files
    "srv/Tree.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces
)

ament_package()
</pre>

<br>

Message는 'msg'라는 서브 디렉토리 밑에, Service는 'srv'라는 서브 디렉토리 밑에 위치시킵니다. (위 `CMakeLists.txt` 파일 참고)

각각의 예제 파일들은 다음과 같습니다. 상단에 상수값을 정의하면 나중에 C++이나 Python에서 해당 메시지나 서비스를 사용할 때 해당 상수값을 이용할 수 있습니다. `TOPIC_NAME`이라던지 이벤트 ID 등을 상수로 정의하면 편리합니다.

<br>

## msg/TreeNode.msg

<pre class="prettyprint">
string NAME = "TreeNode"

string id
string parent_id
string type
string name
</pre>

<br>

## srv/TreeData

<pre class="prettyprint">
string NAME = "TreeData"
int64 REQUEST_TO_PUBLISH_TREE_DATA = 1

int64 request_id
---
int64 request_id
int64 response
</pre>