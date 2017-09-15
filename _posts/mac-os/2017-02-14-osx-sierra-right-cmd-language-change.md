---
layout: post
title: 우측 cmd 키로 한/영 전환하기

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# 우측 cmd 키로 한/영 전환하기

최신 Mac OS(시에라, Sierra)에서도 통하는 방법입니다. MAC OS에서는 기본적으로 한/영 전환을 <kbd>Control</kbd> + <kbd>Space</kbd>로 하고 있습니다. 그러다가 OS 버전이 Sierra가 되면서 <kbd>Caps Lock</kbd> 키로도 가능하도록 개선되었습니다.

어느 방법이든 Windows에서 일반적으로 사용하던 단축키와 달라 불편함을 겪어서 결국, Mac 에서의 한/영 전환을 Windows에 최대한 가깝게 만들기로 결심했습니다. 그래서 오른쪽 <kbd>Command</kbd> 로 한/영 전환을 할 수 있는 방법을 찾아보았습니다.

이 방법은 시에라 및 이전 버전의 OS에서도 작동하는 방법입니다.

<br>

## Karabiner-Elements 설치

먼저 [Karabiner-Elements](https://github.com/tekezo/Karabiner-Elements)를 설치합니다. 간편하게 [여기에서 설치 파일을 다운](https://pqrs.org/latest/karabiner-elements-latest.dmg)받을 수 있습니다.

<br>

## 단축키 매핑

그런 다음 Karabiner-Elements를 실행합니다. 그리고 다음 그림과 같이 오른쪽 <kbd>Command</kbd> 키에 특수한 키를 매핑시키도록 합시다.

![image](/assets/tips-mac/001.png)

맥북 키보드엔 <kbd>F1</kbd> ~ <kbd>F12</kbd> 까지 밖에 없으니, 그 이후 버튼(<kbd>F13</kbd> ~ <kbd>F20</kbd> 등)을 매핑시켜주면 됩니다.

그런 다음 `시스템 환경 설정`으로 가서 `키보드 설정`을 선택합니다.

![image](/assets/tips-mac/002.png)

그리고는 입력 소스 단축키를 다음과 같이 설정합니다.

![image](/assets/tips-mac/003.png)