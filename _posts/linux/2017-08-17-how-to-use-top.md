---
layout: post
title: Top 사용법
category: Linux
tag: [Linux 명령어]
---
# 시스템 모니터링

`top`는 시스템 모니터링을 하는 명령어입니다. 시스템의 부하 관련 정보를 수초마다 체크하여 다음과 같이 화면에 갱신해줍니다.

 ![image](/assets/2017-08-17-how-to-use-top/01.png)

* load average : CPU가 처리하는 걸 기다리는 작업 개수. 1 분당 평균으로 몇 개의 일이 쌓이는지 나타냄
* TIME+ : 해당 프로세스가 실제로 CPU를 사용하는 시간
* COMMAND : 프로세스가 실행되었을 때 실행한 명령어 커맨드. <kbd>C</kbd>를 눌러 상세 표시 전환 가능

<br>

# 단축키

단축키 | 설명
--- | ---
<kbd>Shift</kbd> + <kbd>M</kbd> | 메모리 소비량 순으로 정렬
<kbd>Shift</kbd> + <kbd>T</kbd> | CPU 실행 시간 순으로 정렬
<kbd>Shift</kbd> + <kbd>P</kbd> | CPU 점유량 순으로 정렬