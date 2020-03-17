---
layout: post
title: Ubuntu 설치 후 초기 설정(18.04 기준)
category: Linux
tag: [리눅스 설정, Ubuntu]
---

우분투 18.04 설치 이후 할 일들입니다.

<br>

# 업데이트

 `Software & Updates`를 실행해서 필요한 항목들을 업데이트 합니다. 

터미널에서

<pre class="prettyprint">
sudo apt update
sudo apt upgrade
</pre>

명령어를 통해 업데이트를 해줍니다.

<br>

## 한글 키보드 설치

한글 키보드 설치 방법은 [여기](/linux/2018/07/11/ubuntu-18p04-install-korean-keyboard/)에서 확인할 수 있습니다.

<br>

## 키보드 단축키 변경

`Setting`으로 들어가서 `Devices` > `Keyboard`에서 키보드 단축키를 변경할 수 있습니다.

* `Home folder`는 <kbd>Windows</kbd> + <kbd>E</kbd>
* `Copy a screenshot of a window to clipboard`는 <kbd>Shift</kbd> + <kbd>Ctrl</kbd> + <kbd>1</kbd>
* `Copy a screenshot of an area to clipboard`는 <kbd>Shift</kbd> + <kbd>Ctrl</kbd> + <kbd>2</kbd>
* `Save a screenshot of a window to Pictures`는 <kbd>Shift</kbd> + <kbd>Ctrl</kbd> + <kbd>3</kbd>
* `Save a screenshot of an area to Pictures`는 <kbd>Shift</kbd> + <kbd>Ctrl</kbd> + <kbd>4</kbd>

<br>

## gnome tweak tool 설치

~~~
sudo apt install gnome-tweak-tool
~~~

<br>

## Dock 설정

`Setting`에서 `Dock` 선택. 아이콘 크기는 `32` 정도로, Dock 위치는 `Bottom`으로 설정

<br>

## .bashrc 수정

<pre class="prettyprint">
vscode ~/.bashrc
</pre>

실행해서 아래라인의 숫자값 늘립니다. 파일 실행 이력 히스토리 크기를 늘리는 방법입니다.

<pre class="prettyprint">
HISTSIZE=99999
HISTFILESIZE=99999
</pre>

전방 검색(<kbd>Ctrl</kbd> + <kbd>S</kbd>)을 위한 설정도 해줍니다.

<pre class="prettyprint">
# for (i-search)
stty stop undef
</pre>

를 추가해줍니다.

<br>

## Sublime Text 3 설치

<pre class="prettyprint">
wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
sudo apt-get install apt-transport-https
echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
sudo apt-get update
sudo apt-get install sublime-text
</pre>

<br>

## D2Coding Font 설치

http://snowdeer.github.io/linux/2017/12/02/ubuntu-install-font-d2coding/

<br>

## Terminator 설치

<pre class="prettyprint">
sudo apt install terminator
</pre>