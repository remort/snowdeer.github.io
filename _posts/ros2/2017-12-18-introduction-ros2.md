---
layout: post
title: ROS 2.0 소개
category: ROS2
tag: [ROS]
---

ROS 2.0 Ardent Apalone 버전이 2017년 12월 8일에 공개되었습니다. 그동안 알파 버전부터 베타3 버전까지 배포되었으나, 이번에 공개한 Ardent Apalone은 최초의 non-beta 버전입니다.

<br>

# ROS 2.0이 나오게 된 배경

ROS는 로봇 계열에서 가장 많이 사용되던 운영체제였습니다. 하지만, ROS는 아래와 같은 문제점 또는 특징을 갖고 있었습니다.

## ROS 1.0의 특징

* 단일 로봇(Single Robot) 지원
* 워크스테이션 급의 컴퓨팅 리소스 필요
* 리얼타임(Real-time) 요구사항 미지원
* 아주 훌륭한 네트워크 연결 상태 필요(유선이거나 아주 높은 대역폭의 무선 필요)
* 주로 연구 기관, 아카데미 등에서 사용

<br>

## 새로운 요구 사항 등장

* 멀티 로봇 지원 : 기존 ROS의 경우 일부분을 수정하여 복수의 로봇을 제어할 수는 있었지만 일반적인 접근 방식은 아니었습니다. 오늘날에는 복수의 로봇을 동시에 제어하는 시나리오가 많이 생겼습니다.
* Small Embeded Platform : OS가 탑재되지 않은 마이크로 컨트롤러(Micro Controller)와 같은 작은 플랫폼과도 ROS와 연동시키고 싶은 경우가 생겼습니다.
* 리얼타임(Real-Time) 시스템 : 프로세스간, 기계간 통신을 ROS를 통해 실시간으로 제어할 수 있어야 합니다.
* 네트워크 조건 완화 : 기존에는 너무 이상적인 네트워크 환경을 요구했지만 앞으로는 상태가 나쁜 WiFi 상황도 고려해야 합니다.

<br>

## 새로운 기술 적용

아래와 같은 새로운 기술들이 적용되었습니다.

* Zeroconf
* Protocol Buffers
* ZeroMQ (and the other MQs)
* Redis
* WebSockets
* DDS (Data Distribution Service)

<br>

# OS 지원

ROS 2.0은 다음과 같은 OS를 지원합니다.

* Ubuntu 16.04 LTS (Xenial)
* Mac OS X 10.12 (Sierra)
* Windows 10