---
layout: post
title: Windows 10 Bash 설치
category: Windows10
tag: [windows, bash]
---

윈도우 10에서 Bash를 설치하는 방법입니다.
2016년 봄부터 윈도우 10에서 Bash를 사용할 수 있도록 제공하고 있습니다.
하지만, 아직 정식 지원은 아니며 개발자 모드를 활성화해야만 사용할 수 있습니다.
더 자세한 내용은 [여기](https://blogs.msdn.microsoft.com/eva/?p=7633)를 참고하세요.

<br>

## 최소 요구 사항

* Build 14316 이후
* 64 bit 운영체제

시스템 정보에서 다음과 같은 정보를 확인 할 수 있습니다.

![Image]({{ site.baseurl }}/assets/2016-10-31-windows10-bash-install/1.png)

<br>

## 설치 방법

* 개발자 모드를 설정합니다.
'모든 설정 → 업데이트 및 복구 → 개발자용'에서 옵션을 다음과 같이 선택합니다.

![Image]({{ site.baseurl }}/assets/2016-10-31-windows10-bash-install/2.png)

* Windows 기능 켜기/끄기에서 'Linux용 Windows 하위 시스템(베타)'를 설치합니다.
'제어판 → 프로그램 제거 및 변경'에서 찾을 수 있습니다.

![Image]({{ site.baseurl }}/assets/2016-10-31-windows10-bash-install/3.png)

![Image]({{ site.baseurl }}/assets/2016-10-31-windows10-bash-install/4.png)

* 명령 프롬프트 창에서 bash를 실행합니다.

![Image]({{ site.baseurl }}/assets/2016-10-31-windows10-bash-install/5.png)

이제 bash가 설치되고 사용을 할 수 있게 됩니다.