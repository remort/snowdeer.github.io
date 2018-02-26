---
layout: post
title: Docker Architecture
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---
# Docker Architecture

![Image](/assets/docker/005.png)

Docker 구조는 위 그림과 같습니다.

가운데의 도커 호스트(Docker Host)안에 도커 데몬(Docker Daemon)이 존재하며 컨테이너의 생성, 실행, 모니터링 및 이미지 관리 등의 역할을 합니다.

도커 클라이언트(Docker Client)는 도커 데몬에게 명령을 내릴 수 있는 인터페이스 역할을 하며, 도커 데몬과는 HTTP 통신을 합니다. 기본적으로 유닉스 도메인 소켓(Unix Domain Socket)을 통해 통신을 하지만, 원격 클라이언트 또는 `systemd`가 관리하는 소켓의 파일 디스크립터(File Descriptor)를 통해 TCP 통신을 하기도 합니다.

도커 레지스트리(Docker Registry)는 이미지들을 저장하고 배포하는 역할을 합니다. 보통 도커 허브(Docker Hub)를 통해 이미지를 배포하며 독자적인 사설 레지스트리를 생성해서 운영할 수도 있습니다.