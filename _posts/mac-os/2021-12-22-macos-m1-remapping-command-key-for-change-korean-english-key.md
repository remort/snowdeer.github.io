---
layout: post
title: M1 오른쪽 Command 키를 한/영 전환키로 맵핑하는 방법

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# 오른쪽 Command 키를 한/영 전환키로 맵핑하는 방법
`Karabiner-Elements`를 이용해서 키맵핑하는 방법입니다. 초반에는 Karabiner가 M1과 충돌이 나서 
배터리 광탈 또는 커널 패닉 등의 문제가 있었지만, 지금은 해결된지 오래되었습니다.

`Karabiner-Elements`는 [여기](https://karabiner-elements.pqrs.org)에서 다운로드 할 수 있습니다.

설치 파일을 다운로드한 다음 설치까지 진행합니다.

## Karabiner-Elements 권한 부여

`Karabiner-Elements`를 실행합니다. 
만약 권한이 필요하면 화면과 같이 `karabiner-grabber`와 `karabiner-observer`에 `Input Monitoring` 권한을 부여해줍니다.

![image](/assets/tips-mac/014.png)

위 두 항목이 모두 보이지 않을 경우에는 화면에 보이는 항목 하나에서 팝업 메뉴를 실행하면 `Show in Finder` 메뉴가 출력됩니다.

![image](/assets/tips-mac/015.png)

그래도 못 찾으면 아래 경로에서 해당 파일들을 찾을 수 있습니다.

```
/Library/Application Support/org.pqrs/Karabiner-Elements/bin
```

또한 아래와 같은 권한도 부여를 해줘야 합니다. `System Preferences`의 `Security & Privacy`를 실행하면 
`General` 탭에서 확인할 수 있습니다.

![image](/assets/tips-mac/016.png)

## Karabiner-Elements 설정

암튼 권한 부여 후 `Karabiner-Elements`를 실행합니다. 그리고 다음 그림과 같이 오른쪽 <kbd>Command</kbd> 키에 특수한 키를 매핑시킵니다.

![image](/assets/tips-mac/017.png)

그런 다음 `System Preferences` 설정으로 가서 `Keyboard` 설정을 선택합니다.

![image](/assets/tips-mac/018.png)