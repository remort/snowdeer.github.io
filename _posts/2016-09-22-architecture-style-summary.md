---
layout: post
title: Architecture Style 장단점들
category: 소프트웨어 설계
tag: [architecture style]
---

각 Architecture Style의 장단점들만 간단히 정리해보았습니다.

<br>

## Batch Sequential Architecture Style

장점

* Simple divisions on subsystems
* Each subsystem can be a stand-alone progrma working on input data and producing output data

단점

* Implementation requires external control
* It does not provide interactive interface
* Concurrency is not supported and hence throughput remains low
* High latency

<br>

## 