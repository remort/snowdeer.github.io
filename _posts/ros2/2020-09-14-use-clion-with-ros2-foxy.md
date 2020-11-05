---
layout: post
title: Clion에서 ROS 2.0 Foxy 사용하기
category: ROS2
tag: [ROS]
---

Ubuntu 20.04와 ROS2 Foxy를 같이 사용할 경우 `Clion`에서 빌드를 돌리면 다음과 같은 오류가 발생할 수 있습니다.

~~~
Could NOT find FastRTPS (missing: FastRTPS_INCLUDE_DIR FastRTPS_LIBRARIES)
~~~

이 경우 `CMakeLists.txt` 파일에 아래 내용을 넣어주면 됩니다.

<pre class="prettyprint">
set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)
</pre>