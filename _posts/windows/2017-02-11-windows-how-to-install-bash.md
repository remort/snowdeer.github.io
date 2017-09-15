---
layout: post
title: Windows 10 - Bash 설치

category: Windows
permalink: /windows/:year/:month/:day/:title/
tag: [Windows]
---
# Windows 10에서 Bash를 설치하는 방법

2016년 봄부터 윈도우 10에서 Bash를 사용할 수 있도록 제공하고 있습니다. 하지만, 아직 정식 지원은 아니며 개발자 모드를 활성화해야만 사용할 수 있습니다. 더 자세한 내용은 [여기에서 확인](https://blogs.msdn.microsoft.com/eva/?p=7633)할 수 있습니다.

<br>

## Bash를 사용하기 위한 최소 사양

* Build 14316 이후
* 64 bit 운영체제
 

시스템 정보에서 내 PC가 요구 사양을 만족하는지 확인 할 수 있습니다.

![image](/assets/tips-windows/006.png)

<br>

## 설치 방법

먼저 개발자 모드를 설정합니다.

~~~
모든 설정 → 업데이트 및 복구 → 개발자용
~~~

![image](/assets/tips-windows/007.png)

Windows 기능 켜기/끄기에서 ‘Linux용 Windows 하위 시스템(베타)’를 설치합니다.

~~~
제어판 → 프로그램 제거 및 변경
~~~

![image](/assets/tips-windows/008.png)

![image](/assets/tips-windows/009.png)


명령 프롬프트 창에서 bash를 실행합니다.

![image](/assets/tips-windows/010.png)

이제 bash가 설치 완료되었습니다.