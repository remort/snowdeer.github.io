---
layout: post
title: ROS 2.0 소스로 설치할 때 중간에 Timeout 오류나는 경우 해결법
category: ROS2
tag: [ROS]
---
# Timeout 오류 문제 해결법

ROS 2.0 설치를 소스로 다운받아서 빌드하는 경우, 중간에 소스를 다운받다가 Timeout이 나는 경우가 종종 있습니다. 

예를 들면, `rviz_ogre_vendor`라는 패키지를 600초 안에 다운 받아야 하는데, 만약 다운 속도가 느려서 600초가 넘어가면 다운로드 실패가 되며 다시 처음부터 다운받아야하는 불상사가 생기는 경우가 있습니다. 응답 없음에 대한 Timeout이 아니라, 전체 다운로드하는데 걸리는 시간이 600초 이하라야 하더군요. 

해결법은 다음과 같습니다.

`/ros2_ws/src/ros2/rviz/rviz_ogre_vendor` 폴더를 찾아갑니다. `rviz_ogre_vendor` 패키지가 아닌 다른 패키지에서 문제가 발생하면 해당 패키지 폴더를 찾아가면 됩니다. 해당 폴더에서 `CMakeList.txt` 파일을 열면 그 안에 다운로드 경로와 Timeout 설정값이 있습니다. Timeout 값을 20~30배로 늘려주면 됩니다.
