---
layout: post
title: CMake 업그레이드 방법 
category: Linux
tag: [Linux]
---
# CMake 업그레이드 방법 

다음 명령어를 이용해서 `cmake`를 업그레이드 할 수 있습니다.

~~~
sudo -E add-apt-repository -y ppa:george-edison55/cmake-3.x
sudo -E apt-get update
sudo apt-get install cmake
~~~

다만, 위 방법으로는 3.5만 받아집니다. 2018년 4월 10일 기준으로 `cmake` 최신 버전은 3.11 입니다.
최신 버전을 받기 위해서는 다음 명령어를 이용합니다.

~~~
sudo apt remove cmake

https://cmake.org/files/v3.11/cmake-3.11.0.tar.gz
tar -zxvf cmake-3.11.0.tar.gz

cd cmake-3.11.0
./bootstrap
make
sudo make install

cmake --version
~~~