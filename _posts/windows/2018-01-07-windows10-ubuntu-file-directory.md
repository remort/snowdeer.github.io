---
layout: post
title: Windows 10 - Ubuntu on Windows 10 서로간 파일 시스템 접근하기

category: Windows
permalink: /windows/:year/:month/:day/:title/
tag: [Windows]
---
# Ubuntu on Windows 10

Windows 10에서는 Ubnutu가 Native로 동작할 수 있습니다. 스토어에서 Ubuntu를 내려받아 설치하면 됩니다. 물론, 제어판에서 아래와 같은 작업 후 '재부팅'을 해주어야 Ubuntu가 정상동작합니다.

~~~
제어판 → 프로그램 제거 및 변경
~~~

![image](/assets/tips-windows/008.png)

![image](/assets/tips-windows/009.png)

<br>

# Windows에서 Ubuntu 파일 시스템에 접근

Windows 상에서 Ubuntu가 설치되었을 때 해당 파일 시스템은 아래와 같은 폴더에 저장됩니다.

~~~
C:\Users\<username>\AppData\Local\Packages\CanonicalGroupLimited.UbuntuonWindows_79rhkp1fndgsc\LocalState
~~~

가상화가 아닌 Native로 동작하는 리눅스이다보니 압축된 파일이 아니라, 파일 시스템 자체에 그대로 접근이 가능합니다. 물론, Ubuntu 상에서 작성한 파일을 Windows에서 수정할 수도 있고, 반대로 Windows에 있는 파일을 Ubuntu로 밀어넣을 수도 있습니다. (단, 이 때는 `chmod` 명령어로 권한 변경을 해주어야 매끄럽게 동작합니다.)

<br>

# Ubuntu에서 Windows 파일 시스템에 접근

반대로 Ubuntu에서 Windows의 파일 시스템에 접근할 수 있습니다. 아래의 경로를 이용하면 Ubuntu에서 Windows 파일 시스템이 보입니다.

~~~
/mnt/c
~~~