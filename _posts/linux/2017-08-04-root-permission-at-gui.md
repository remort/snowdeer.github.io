---
layout: post
title: gksu 사용법 - root 권한으로 GUI 어플 실행하기
category: Linux

tag: [리눅스 명령어]
---

라즈베리파이 뿐만 아니라 우분투(Ubuntu)에서도 동일하게 적용되는 방법입니다.

터미널에서 GUI에 Root 권한을 주는 명령어는 `gksu`입니다.

예를 들어 파일 탐색기 GUI를 Root 권한으로 실행하는 방법은 터미널에서

~~~
gksu pcmanfm
~~~

으로 실행하면 됩니다. (`pcmanfm`이 라즈베리파이용 기본 파일 탐색기입니다.)

만약 텍스트에디터를 열고 싶으시면

~~~
gksu leafpad
~~~

라고 실행하시면 됩니다. (`leafpad`가 라즈베리파이용 기본 텍스트에디터기입니다.)