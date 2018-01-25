---
layout: post
title: 우분투(Ubuntu)에서 Snap Package 사용하기
category: Linux
tag: [리눅스 설정, Ubuntu]
---
# Snap Package
 
우분투에 기본 내장된 `Ubuntu Software`를 이용해서 프로그램들을 설치하려고 했을 때 아래와 같은 오류가 발생하는 경우가 있습니다.

![image](/assets/linux/002.png)

~~~
Detailed errors from the package manager follow:

snapd returned status code 400: Bad Request
~~~

Linux가 Ubuntu, Fedora, Debian 등 다양한 형태로 파편화가 되어 있고, 프로그램 설치 파일 형태도 제각각이기 때문에 Universal Package로 만들려는 시도가 있었습니다. 그러다가 우분투에서는 'Snap'이라는 이름의 Universal Package를 런칭했습니다. 당연히 'Snap'은 다른 리눅스에서도 설치가 가능했습니다. 작년 초에 [Snap 2.20](https://insights.ubuntu.com/2017/01/09/how-to-snap-introducing-classic-confinement/)을 릴리즈했고, 기존에 사용하던 App인 'Software Center'에서는 아직 지원이 안되고 있습니다. 그래서 현재는 Snap을 사용하려면 터미널에서 커맨드 입력으로 사용해야 합니다.

<br>

## Snap 사용법

다음과 같은 형태로 사용하면 됩니다.

~~~
$ snap find "visual studio code"
~~~

위 명령어를 입력하면 'Visual Studio Code'에 대한 정보가 나오고 패키지명과 Class Snap 여부를 확인할 수 있습니다.

![image](/assets/linux/003.png)

따라서 위와 같은 경우는 아래 명령어를 이용해서 프로그램을 설치할 수 있습니다.

~~~
$ sudo snap install vscode --classic
~~~