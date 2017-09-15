---
layout: post
title: Homebrew 설치하기

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# Homebrew

Mac에서 홈브류(Homebrew)는 패키지 매니저(Package Manager)로 생각하면 됩니다. ruby로 개발되어졌으며, 애플에서 제공하지는 않지만 유용한 프로그램들을 손쉽게 다운로드하고 설치할 수 있게 해주고 있습니다.

![image](/assets/tips-mac/009.png)

<br>

## 설치 방법

Homebrew를 설치하기 위해서는 먼저 `CLD(Command Line Developer Tool)`를 설치해야 합니다. 터미널에서 다음 명령어를 입력하면 됩니다.

~~~
xcode-select –install
~~~

그 다음에는 다음 명령어를 이용하여 Homebrew를 설치합니다.

~~~
ruby -e “$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)”
~~~

최신 설치 명령어는 [여기에서 확인](https://brew.sh/)할 수 있습니다.

설치를 시작하면, ‘Press RETURN to continue or any other key to abort’ 라는 메세지가 나오고 <kbd>Return</kbd> 키 를 누르면 설치가 시작됩니다. 