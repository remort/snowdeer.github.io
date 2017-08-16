---
layout: post
title: Tmux 사용법
category: Linux
tag: [Linux 명령어]
---
# 가상 터미널

`tmux`는 가상 터미널을 실행할 수 있게 해주는 프로그램입니다. '가상 터미널'이란 Windows나 MacOS의 '가상 데스크탑'과 비슷한 개념이라고 생각하면 됩니다.

`tmux`를 실행하면 다음과 같은 화면이 나옵니다.

 ![image](/assets/2017-08-16-how-to-use-tmux/01.png)

 화면 하단에 초록색의 바(Bar)가 출력이 되며, 가상 터미널 상태가 됩니다. 

 이 상태에서 기존 작업을 저장을 하고 싶으면, <kbd>Ctrl</kbd> + <kbd>B</kbd>를 누른다음 <kbd>D</kbd>를 누르면 기존 작업을 저장하면서 `tmux`를 빠져나가게 됩니다. 

 그 이후 `tmux attach`를 이용해서 기존 작업에 다시 접속할 수 있습니다.

 <br>

 # 명령어 단축키

`tmux`에서 명령어 단축키는 <kbd>Ctrl</kbd> + <kbd>B</kbd>를 누른 후, 다음 키 조합을 이용해서 명령을 내릴 수 있습니다.

단축키 | 설명
--- | ---
<kbd>D</kbd>(Deatch) | 현재 작업을 저장하면서 tmux를 종료
<kbd>C</kbd>(Create) | 새로운 터미널 창을 생성
<kbd>P</kbd>(Previous) | 이전 터미널 창으로 이동
<kbd>N</kbd>(Next) | 다음 터미널 창으로 이동

<br>

# 화면 분할

`tmux`의 제공하는 또 하나의 강력한 기능은 '화면 분할'에 있습니다. <kbd>Ctrl</kbd> + <kbd>B</kbd>를 누른 후 <kbd>"</kbd>를 입력하면 화면이 분할됩니다.

![image](/assets/2017-08-16-how-to-use-tmux/02.png)

이 상태에서는 <kbd>Ctrl</kbd> + <kbd>B</kbd> 이후 방향 키를 이용해서 창을 전환할 수 있습니다.

다만, 이 경우 화면 스크롤에서 어려움이 있을 수 있습니다. 터미널 자체를 스크롤하는 것이 아니라 `tmux` 프로그램 내에서 스크롤을 해야 하기 때문입니다. 이 때는 <kbd>Ctrl</kbd> + <kbd>B</kbd> 이후 <kbd>[</kbd> 키를 누르면 그 이후 방향키나 마우스 휠로 스크롤을 할 수 있습니다. 빠져나오고 싶을 때는 <kbd>Q</kbd> 키를 누르면 됩니다.

<br>

## 화면 가로 분할

화면을 가로로 분할 할 수도 있습니다. 이 때 단축키는 <kbd>Ctrl</kbd> + <kbd>B</kbd>를 누른 후 <kbd>%</kbd>입니다.

![image](/assets/2017-08-16-how-to-use-tmux/03.png)

세로 분할 때와 마찬가지로 <kbd>Ctrl</kbd> + <kbd>B</kbd> 이후 방향 키를 이용해서 창을 전환할 수 있습니다.

화면 분할을 종료하고 싶을 때는 `exit` 명령어를 입력하거나 이 상태에서는 <kbd>Ctrl</kbd> + <kbd>D</kbd>를 누르면 해당 터미널을 종료합니다.