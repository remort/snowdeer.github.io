---
layout: post
title: 안드로이드 SDK의 Path 설정

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# 안드로이드 SDK의 Path 설정

Mac에서 터미널을 통해 ‘adb’ 명령어를 실행하고 싶을 때가 종종 있습니다. Windows에서는 Android SDK가 설치된 경로를 환경변수로 등록해놓으면 되는데, Mac에서도 방법은 비슷합니다. 다만, Mac에서 Android SDK가 설치되어 있는 폴더 위치를 몰라서 Path 등록을 못하는 경우가 종종 있습니다.

<br>

## Mac에서 Android SDK가 설치되는 폴더

사용자가 임의로 폴더 위치를 수정하지 않았다면, 기본적으로 Android SDK는 다음 위치에 설치됩니다.

~~~
/Users/snowdeer/Library/Android
~~~

## .bash_profile 수정

환경 변수를 등록하기 위해서 `.bash_profile`을 수정하면 됩니다. 터미널에서 `nano`를 이용해서 수정을 해도 되고, 편하게 GUI 상의 ‘텍스트편집기’를 이용해서 수정을 해도 됩니다. 기본적으로 `.bash_profile`은 속성이 ‘숨김(Hidden)’으로 되어있습니다. Finder에서 숨김 파일을 보기 위해서는 Finder 창 안에서 단축키 <kbd>Shift</kbd> + <kbd>Command</kbd> + <kbd>.</kbd> 을 누르면 됩니다. `.bash_profile`은 `/User/[사용자계정]` 아래에 위치하고 있습니다.

그리고 `.bash_profile` 내에 다음과 같은 라인을 추가합니다.

~~~
export ANDROID_PATH=/Users/snowdeer/Library/Android
export PATH=$PATH:$ANDROID_PATH/sdk/platform-tools
~~~

이제 터미널을 다시 시작한 후 터미널 콘솔 창에서 `adb` 명령어를 입력해보면 잘 실행되는 것을 확인할 수 있습니다.
