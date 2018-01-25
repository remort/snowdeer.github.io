---
layout: post
title: 우분투(Ubuntu) 16.04에 아나콘다(Anaconda) 설치하는 방법
category: Linux
tag: [리눅스 설정, Ubuntu]
---
# Ubuntu 16.04에 Anaconda 설치

Ubuntu에 아나콘다를 설치하는 방법은 다음과 같습니다.

먼저 [아나콘다 홈페이지](https://www.anaconda.com/download/#linux)에 가서 설치 스크립트 파일을 다운로드 합니다. 현재 Python 3.6 버전에 64비트 버전 기준으로 약 500MB가 조금 넘습니다.

그리고 터미널로 가서 다음 명령어를 입력해줍니다.

~~~
bash Anaconda3-5.0.1-Linux-x86_64.sh
~~~

이 때부터 <kbd>Enter</kbd> 키와 `yes` 타이핑 등을 차례로 요구하기 때문에 천천히 설치를 진행합니다.

설치 경로는 보통 `/home/[userid]/anaconda3`로 지정이 될 것이며, 그 후 `.bashrc`에 패스 등록까지 해줍니다.

그 후 

~~~
source ~/.bashrc
~~~

명령어를 통해 `.bashrc`를 한 번 더 수행해주면 됩니다.

그 이후 `conda list` 등의 명령어가 정상적으로 동작하는지 확인하면 됩니다.