---
layout: post
title: iTerm2에서 한글 깨지는 경우

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# iTerm2

Linux에는 [terminator](https://terminator-gtk3.readthedocs.io/en/latest/#)가 있다면, MacOS에서는 [iTerm2](https://iterm2.com)라는 막강한 터미널 프로그램이 있습니다.

그런데, `3.3.10` 버전 이후로 아래 사진과 같이 한글이 깨져서 보여주는 현상이 발생하고 있습니다. 

![image](/assets/tips-mac/011.png)

위 이미지에서 1 번째, 2 번째 디렉토리의 한글이 깨져서 보이는 것을 확인할 수 있습니다. 그런데, 3 번째 디렉토리는 정상으로 보이네요?

차이는 1 번째, 2 번째 디렉토리는 `Finder`에서 새로 생성한 디렉토리이고, 3 번째 디렉토리는 `iTerm2`에서 `mkdir` 명령어를 이용해서 만들었다는 차이가 있습니다.

즉, 단순히 폰트의 문제가 아닌 글자 자체, 유니코드 쪽의 설정이 다르다는 것을 의심할 수 있습니다.

해결책은 `iTerm2`의 `Preference -> Text`로 가서

![image](/assets/tips-mac/012.png)

`Unicode normalization form`을 `None`에서 `NFC`로 변경합니다.

![image](/assets/tips-mac/013.png)

그 이후 다시 출력해보면 한글이 잘 출력되는 것을 알 수 있습니다.