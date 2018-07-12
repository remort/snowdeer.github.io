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

~~~
sudo apt update
sudo apt upgrade
~~~

명령어를 통해 업데이트를 해줍니다.

<br>

## 한글 키보드 설치

한글 키보드 설치 방법은 [여기](/linux/2018/07/12/ubuntu-18p04-install-korean-keyboard/)에서 확인할 수 있습니다.

<br>

## 키보드 단축키 변경

`Setting`으로 들어가서 `Devices` > `Keyboard`에서 키보드 단축키를 변경할 수 있습니다.

* `Home Folder` 단축키를 <kbd>Windows</kbd> + <kbd>E</kbd>로 변경합니다.
* `Screenshots` 항목 내의 `Save a screenshot of an area to Pictures` 항목을 <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>4</kbd>로 변경 (Ubuntu 18.04에서는 클립보드 복사도 따로 있어서 다른 키로 할당)

<br>

## gnome tweak tool 설치

~~~
sudo apt install gnome-tweak-tool
~~~


## Dock 설정

`Setting`에서 `Dock` 선택. 아이콘 크기는 `32` 정도로, Dock 위치는 `Bottom`으로 설정