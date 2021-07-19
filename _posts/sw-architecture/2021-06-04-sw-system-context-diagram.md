---
layout: post
title: System Context Diagram
category: S/W Architecture
permalink: /sw-architecture/:year/:month/:day/:title/

tag: [설계, ContextDiagram]
---

# Context Diagram

System Context Diagram의 목적은 시스템의 범위를 표현하는 것이다. 레벨이 존재하긴 하지만 대부분의 Context Diagram은 최상위 수준(TLCD, Top level context diagram)으로 표현한다.

Context는 시스템이 상호작용하는 환경을 의미하며 다음과 같은 것들이 될 수 있다.

* 사람
* 다른 시스템
* 센서, 제어 장치 등의 H/W

Context Diagram은 개발해야 하는 범위를 알 수 있고, 시스템의 범위를 벗어나있는 프레임워크, 라이브러리, 외부 서비스, 다른 시스템, 다른 접점의 S/W 등을 명확하게 보여준다.

<br>

## Context Diagram 내용

순수한 Context Diagram은 시스템에 대한 아키텍처 세부사항을 보여주지 않는다. (하지만 같은 Context내의 시스템의 일부 내부 구조를 보여주는 경우가 종종 있다.) 또한 인터랙션 정보나 데이터 흐름(데이터가 전송되고, 자극이 발생하고, 메시지가 전송되는 등)을 보여주지도 않는다. 

<br>

