---
layout: post
title: 우분투(Ubuntu) 16.04 크롬 브라우저 설치
category: Linux
tag: [Ubuntu]
---
# 크롬(Chrome) 브라우저 설치

Ubuntu 16.04 기준으로 크롬 브라우저 설치하는 방법입니다.

<br>

## wget으로 설치하기

~~~
$ sudo apt-get install libxss1 libgconf2-4 libappindicator1 libindicator7

$ wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb 

$ sudo dpkg -i google-chrome-stable_current_amd64.deb
~~~

<br>

## apt-get으로 설치하기

~~~
$ wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add - 

$ sudo sh -c 'echo "deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google.list'

$ sudo apt-get update

$ sudo apt-get install google-chrome-stable
~~~

만약 설치 이후에 `apt-get update` 명령을 이용해서 패키지 목록을 업데이트 할 때, 다음과 같은 오류 메세지가 나오면

~~~
W: Target Packages (main/binary-amd64/Packages) is configured multiple times in /etc/apt/sources.list.d/google-chrome.list:3 and /etc/apt/sources.list.d/google.list:1
~~~

아래 명령을 수행해줍니다.

~~~ 
$ sudo rm -rf /etc/apt/sources.list.d/google.list 
~~~
