---
layout: post
title: 우측 cmd 키로 한/영 전환
category: OSX
tag: [osx, right command, language change]
---

최신 MAC OS(시에라, Sierra)에서도 통하는 방법입니다.  
(지금 이 맥북에도 이 방법을 적용해서 사용하고 있습니다.)

MAC OS에서는 기본적으로 한/영 전환을 <kbd>control</kbd> + <kbd>space</kbd>로 하고 있습니다.
그러다가 OS 버전이 Sierra가 되면서 <kbd>caps lock</kbd> 키로 변경이 되었습니다.

어느 방법이든 기존에 사용해오던 방식과는 많이 달라서 불편하더군요.
그래서 최대한 보편적인 방식에 가깝게 우측 <kbd>command</kbd> 키로 한/영 전환을 할 수 있도록 했습니다.

이 방법은 시에라 이전 버전의 OS에서도 작동합니다.

<br>

## Karabiner-Elements 설치

먼저 [Karabiner-Elements](https://github.com/tekezo/Karabiner-Elements)을 설치합니다.
간편하게 [여기](https://pqrs.org/latest/karabiner-elements-latest.dmg)를 클릭하면 바로 
설치 파일을 다운받을 수 있습니다.


<br>

## 단축키 매핑

위에서 설치한 Karabiner-Elements를 실행해줍니다. 
그리고 다음 그림과 같이 오른쪽 Command 키에 특수한 키를 매핑시켜줍니다.

맥북 키보드엔 F1~F12 까지 밖에 없으니, 그 이후 버튼을 매핑시켜주면 됩니다.

![image]({{ site.baseurl }}/assets/2016-11-14-osx-sierra-right-cmd-language-change/1.png)

그 다음엔 시스템 환경 설정으로 가서 키보드 설정을 선택합니다.

![image]({{ site.baseurl }}/assets/2016-11-14-osx-sierra-right-cmd-language-change/2.png)

그리고는 입력 소스 단축키를 다음과 같이 설정합니다.

![image]({{ site.baseurl }}/assets/2016-11-14-osx-sierra-right-cmd-language-change/3.png)

끝입니다.

이제 오른쪽 Command 키로 한/영 전환을 쉽게 할 수 있습니다.

