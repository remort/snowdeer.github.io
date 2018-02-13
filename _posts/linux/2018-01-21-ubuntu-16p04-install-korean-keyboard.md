---
layout: post
title: 우분투(Ubuntu) 16.04 한글 키보드 설치
category: Linux
tag: [리눅스 설정, Ubuntu]
---
# 한글 키보드 설치

Ubuntu 16.04 LTS 버전 기준으로 한글 키보드를 설치하는 방법입니다.

먼저 아래의 명령어를 수행해서 `fcitx-hangul` 패키지를 설치합니다.

~~~
sudo apt-get install fcitx-hangul
~~~

그리고 아래의 절차를 진행합니다.

* `System Settings` 실행
* `Language Support` 아이콘 실행
* 언어팩을 설치하라는 팝업창이 뜨면 '설치' 선택
* 'Keyboard input method system' 항목을 `fcitx`로 변경
* 재부팅

<br>

## 오른쪽 한/영키(Alt 키)를 이용한 한/영 전환

Unbuntu에서는 기본적으로 오른쪽 <kbd>Alt</kbd> 키가 커맨드 실행 기능으로 맵핑이 되어 있습니다. 한/영 전환 키로 활용하고 싶으면 다음과 같이 세팅하시면 됩니다.

* `System Settings`에서 `Keyboard` 실행
* `Shortcuts` 탭 선택한 후 `Typing` 항목 선택
* 모든 항목(`Switch to next source`, `Switch to previous source`, `Alternative Characters Key`)을 `Disabled`로 설정(<kbd>Back</kbd> 키를 누르면 `Disabled`가 됨)
* `Compose Key` 항목을 `Right Alt`로 변경
* `Switch to next source`를 선택한 다음 오른쪽 <kbd>Alt</kbd> 키(<kbd>한/영</kbd> 키)를 누르면 `Multikey`라는 항목으로 값이 설정됨

<br>

## fcitx 설정

* 오른쪽 상단 상태바에서 `fcitx` 아이콘 선택 → `Configure Current Input Method` 선택
* `+` 버튼을 눌러 `Hangul` 항목 추가(`+` 버튼 누른 창에서 `Only Show Current Language` 체크 버튼 해제해야 보임)
* `Global Config` 탭으로 변경하여 `Trigger Input Method` 항목을 <kbd>한/영</kbd> 키로 설정(`Multikey`라고 표현됨)
* `Extra key for trigger input method`는 `Disabled`