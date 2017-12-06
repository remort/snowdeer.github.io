---
layout: post
title: 우분투(Ubuntu) 14.04 한글 키보드 설치
category: Linux
tag: [리눅스 설정, Ubuntu]
---
# 한글 키보드 설치

Ubuntu 14.04 LTS 버전 기준으로 한글 키보드를 설치하는 방법입니다.

* `System Setting`을 실행
* `Language Support` 아이콘을 실행
* 언어팩을 설치하라는 팝업창이 뜨면 '설치' 선택
* 시스템 재부팅
* 터미널에서 `sudo ibus-setup` 실행
* `Input Method` 탭 선택하고, `Customize active input methods` 체크 박스 활성화
* 아래의 `Select an input method` 리스트 박스에서 `Korean` > `Hangul` 선택한 후 `Add` 버튼 클릭
* 다시 `System Setting`에서 `Text Entry` 선택
* `+` 버튼을 눌러 `Korean (Hangul)` 항목 선택

이제 기본적으로 <kbd>Ctrl</kbd> + <kbd>Space</kbd>를 눌러서 한/영 전환을 할 수 있습니다.

<br>

## 오른쪽 한/영키(Alt 키)를 이용한 한/영 전환

Unbuntu에서는 기본적으로 오른쪽 <kbd>Alt</kbd> 키가 커맨드 실행 기능으로 맵핑이 되어 있습니다. 한/영 전환 키로 활용하고 싶으면 다음과 같이 세팅하시면 됩니다.

* `System Setting`에서 `Keyboard` 실행
* `Shortcuts` 탭 선택한 후 `Typing` 항목 선택
* `Compose Key` 항목을 `Right Alt`로 변경
* `Switch to next source`를 선택한 다음 오른쪽 <kbd>Alt</kbd> 키를 누르면 `Multikey`라는 항목으로 값이 설정됨