---
layout: post
title: top 사용법
category: Linux
tag: [리눅스 명령어]
---
# 시스템 모니터링

`top`는 시스템 모니터링을 하는 명령어입니다. 시스템의 부하 관련 정보를 수초마다 체크하여 다음과 같이 화면에 갱신해줍니다.

 ![image](/assets/2017-08-17-how-to-use-top/01.png)

* load average : CPU가 처리하는 걸 기다리는 작업 개수. 1 분당 평균으로 몇 개의 일이 쌓이는지 나타냄
* TIME+ : 해당 프로세스가 실제로 CPU를 사용하는 시간
* COMMAND : 프로세스가 실행되었을 때 실행한 명령어 커맨드. <kbd>C</kbd>를 눌러 상세 표시 전환 가능

<br>

# 프로세스에 대한 내용

항목 | 내용
---|---
PID | 프로세스 ID
USER | 프로세스를 실행한 사용자 ID
PR | 프로세스 우선 순위
NI | 작업 수행의 Nice Value 값으로 마이너스를 갖는 값이 우선 순위가 높음
VIRT | 가상 메모리 사용량(SWAP + RES)
RES | 현재 페이지의 상주 크기(Resident Size)
SHR | 분할된 페이지로 프로세스에 의해 사용된 메모리를 나눈 메모리의 총합
S | 프로세스의 상태. S(Sleeping), R(Running), W(Swapped out process), Z(Zombies) 등의 상태를 가짐
%CPU | CPU 사용률
%MEM | 메모리 사용률

<br>

# 단축키

단축키 | 설명
--- | ---
<kbd>Shift</kbd> + <kbd>M</kbd> | 메모리 소비량 순으로 정렬
<kbd>Shift</kbd> + <kbd>T</kbd> | CPU 실행 시간 순으로 정렬
<kbd>Shift</kbd> + <kbd>P</kbd> | CPU 점유량 순으로 정렬
<kbd>Space</kbd> | 화면 갱신
