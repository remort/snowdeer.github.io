---
layout: post
title: Ubuntu 설치 후 초기 설정(16.04 기준)
category: Linux
tag: [Linux]
---
# Ubuntu 설치 후 초기 설정

## 드라이버 및 소프트웨어 업데이트 

### Software & Updates 설정

`System Settings → Software & Updates`로 들어가서 

* `Ubuntu Software` 탭의 `Download from` 항목을 국내 서버(ex. neowiz)로 변경
*  `Additional Drivers` 탭으로 이동 및 필요한 드라이버 설치(특히 그래픽 카드)

터미널에서

<pre class="prettyprint">
sudo apt-get update

sudo apt-get upgrade
</pre>

명령어를 통해 최신 파일들로 업데이트

그 외 `Ubuntu Software` 프로그램을 수행한 후 `Updates` 탭으로 이동. 필요한 항목들 업데이트

<br> 

## 마우스 감도 및 키보드 감도 세팅

`System Settings → Keyboard` 및 `System Settings → Mouse & Touchpad`에서 감도 조절.

* 노트북의 터치 패드를 사용하는 경우 `Natural scrolling` 항목 선택을 해서 터치 패드의 스크롤 방향 변경
* 일반 키보드를 사용할 경우, <kbd>Shift</kbd> + 숫자 키패드 동작이 Windows와 동일하게 동작하도록 설정

<br>

## 한글 입력 설정

16.04 LTS 기준으로 [여기](/linux/2018/01/21/ubuntu-16p04-install-korean-keyboard/)를 참고
14.04 LTS 기준으로는 [여기](/linux/2017/12/01/ubuntu-14p04-install-korean-keyboard/)를 참고

<br>

## 프로그램 설치

* [크롬 설치](/linux/2018/02/02/ubuntu-16p04-install-chrome/)
* [JDK 설치](/linux/2017/08/28/install-jdk-on-ubuntu/)