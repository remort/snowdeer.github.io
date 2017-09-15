---
layout: post
title: 터미널에서 tree 명령어 사용하기

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# tree 명령어

`tree`는 폴더 구조를 시각적으로 알아보기 쉽게 트리 형태로 보여주는 명령어입니다. 불행히도 Mac에는 이 명령어를 제공하지 않습니다. 그래서 Homebrew를 이용해서 `tree`를 설치해보도록 하겠습니다.

먼저 [Homebrew를 설치](/mac-os/2017/04/01/install-homebrew-on-mac)해야 합니다.

`tree` 설치는 다음 명령어를 이용해서 진행할 수 있습니다.

~~~
brew install tree
~~~

설치 후 `tree` 명령어를 수행하면 다음과 같이 폴더 구조를 트리 형태로 표현해줍니다.

![image](/assets/tips-mac/010.png)