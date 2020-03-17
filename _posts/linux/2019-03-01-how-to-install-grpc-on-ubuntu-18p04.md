---
layout: post
title: gRPC(C++) 설치 방법 

category: Linux
permalink: /mac-os/:year/:month/:day/:title/
tag: [리눅스]
---
# gRPC 설치 방법

Ubuntu 18.04 기준 C++용 gRPC 설치 방법입니다. 다른 언어에 비해 설치 방법이 조금 더 까다롭습니다.

<br>

## 필요 패키지 설치

<pre class="prettyprtint">
sudo apt-get install build-essential autoconf libtool pkg-config
sudo apt-get install libgflags-dev libgtest-dev
sudo apt-get install clang-5.0 libc++-dev
sudo apt-get install libunwind-dev
sudo apt-get install golang
</pre>

<br>

## CMake 업데이트

오래된 버전의 `cmake` 명령어로는 gRPC 설치가 완벽하게 되지 않을 수 있습니다. 특히, `/usr/local/lib/cmake/grpc/gRPCTargets.cmake` 파일이 생성되지 않아서 혼이 난 적이 있습니다.

[여기](https://cmake.org/download/)에서 `cmake`를 최신 버전으로 설치합시다.

<pre class="prettyprtint">
tar -zxvf cmake-3.16.5.tar.gz

cd cmake-3.16.5
./bootstrap
make -j 8
sudo make install
</pre>

<br>

## gRPC 소스 다운로드

<pre class="prettyprtint">
git clone -b $(curl -L http://grpc.io/release) https://github.com/grpc/grpc

cd grpc/
git submodule update --init

mkdir -p cmake/build
cd cmake/build
cmake ../..
make -j 8
sudo make install
</pre>

<br>

## 참고

* https://github.com/grpc/grpc/blob/master/src/cpp/README.md#make
* https://github.com/grpc/grpc/blob/master/BUILDING.md
