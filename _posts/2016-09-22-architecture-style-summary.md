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

## Pipe and Filter Architecture Style

장점

* Concurrency : It provides high overall throughput for excessive data processing.
* Reusability : Encapsulation of filters makes it easy to plug and play, and to substitute
* Modifiability : It features low coupling between filters, less impact from adding new filters, and modifying the implementation of any existing filters as long as the I/O interfaces are unchanged.
* Simplicity : It offers clear division between any two filters connected by a pipe.
* Flexibility : It supports both sequential and parallel execution.

단점

* Dynamic Interaction 용으로는 적합하지 않음
* Filter마다 서로 다른 데이터 포맷을 사용할 수 있기 때문에 데이터 전송에 있어 common한 공통 분모가 존재해야 함
* Filter간 데이터 전송 및 Parsing에 대한 오버헤드


<br>

## Repository Architecture Style

장점

* 백업과 복원이 쉽다.
* 새로운 S/W 컴포넌트 추가가 쉽다. (컴포넌트간 직접 통신이 없음)
* S/W 컴포넌트간 데이터 전송 오버헤드가 낮다.

단점

* 데이터 저장소의 안정성과 가용성이 아주 중요하다.
* 데이터 저장소 및 각 Agent 들간 데이터 구조체 의존도가 높다.
* 만약 데이터가 분산되었을 때, 데이터 전송 비용이 높다.

<br>

## Blackboard Architecture Style

장점

* Scalability : Knowledge Source의 추가 및 업데이트가 쉽다.
* Concurrency : 모든 Knowledge Source는 독립적이기 떄문에 병렬적으로 처리 가능하다.
* 가설이나 추측에 대한 실험이 가능하다.
* Knowledge Source Agent의 재사용이 가능하다.

단점

* Blackboard와 Knowledge Source간 Dependency가 높다. Blackboard의 대이터 구조체 변경은 각 Agent들에 큰 영향을 미칠 수 있다.
* 추리/추론의 종료 시점을 결정하기 어려워서, 최종 Solution은 부분적이거나 Approximated 되었을 가능성이 크다.
* 각 Agent들은 Blackboard내 shared data를 사용하기 떄문에 동기화 문제가 발생할 수 있다.
* 디버깅 및 테스트가 어렵다.

