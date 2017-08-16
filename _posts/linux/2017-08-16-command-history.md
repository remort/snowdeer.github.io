---
layout: post
title: 명령어 이력 좀 더 효율적으로 사용하기
category: Linux
tag: [Linux 명령어]
---
# 명령어 이력

Linux나 Windows, MacOS 등 대부분의 터미널에서 공통적이지만, 명령 프롬프트 상태에서 <kbd>↑</kbd> 또는 <kbd>↓</kbd>를 연속해서 누르게 되면 그동안 실행했던 명령어 이력을 볼 수 있습니다.

<br>

# 후방 검색

여기서 조금 더 효율적으로 명령어 이력 기능을 사용하려면, <kbd>↑</kbd> 대신 <kbd>Ctrl</kbd> + <kbd>R</kbd>을 눌러봅니다. 그러면 프롬프트 앞에 `(reverse-i-search)`라는 문구가 나오게 됩니다. 이 기능은 명령어 이력을 검색하는 기능입니다. 'Reverse'라고 적혀 있는만큼 거꾸로 추척합니다. 

프롬프트가 `(reverse-i-search)`로 바뀌면 검색을 원하는 문구를 입력해 봅시다. 만약 여기서 더 이전 이력을 검색하려면 <kbd>Ctrl</kbd> + <kbd>R</kbd>을 계속해서 누르면 됩니다.

<br>

# 전방 검색

전방 검색의 단축키는 <kbd>Ctrl</kbd> + <kbd>S</kbd>입니다. 하지만 이 단축키는 기본적으로 다른 명령어로 할당이 되어 있습니다. 그래서 이 단축키를 활성화시키려면 다음과 같은 작업을 해주어야 합니다.

다음과 같이 `.bashrc` 파일을 엽니다.

~~~
sudo nano .bashrc
~~~

그리고 파일 맨 끝에 다음 문구를 추가해줍니다.

~~~
stty stop undef
~~~

그런 다음 터미널을 종료한 뒤 다시 시작을 합니다.

이제 <kbd>Ctrl</kbd> + <kbd>S</kbd>가 활성화된 것을 확인할 수 있을겁니다.

<br>

# 명령어 이력 개수 확장

명령어 이력의 개수는 제한이 있습니다. 개수를 늘릴려면 역시 `.bashrc` 파일을 수정하면 됩니다.

~~~
sudo nano .bashrc
~~~

그리고 파일 맨 끝에 다음 문구를 추가해줍니다.

~~~
export HISTSIZE 10000
export HISTFILESIZE 10000
~~~

