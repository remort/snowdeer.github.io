---
layout: post
title: Terminator 사용자 정의 세팅하는 방법
category: Linux
tag: [리눅스, Ubuntu]
---
# Terminator Config 

`terminator`는 개인적으로 많이 애용하고 있는 터미널 프로그램입니다. 사용자 정의 세팅을 하는 방법은 다음과 같습니다.

만약 `~/.config/terminator/config` 파일이 없으면 새로 만들어주면 됩니다.

<br>

##  ~/.config/terminator/config

<pre class="prettyprint">
[global_config]
  handle_size = 0
  focus = system
[keybindings]
[layouts]
  [[default]]
    [[[child1]]]
      parent = window0
      type = Terminal
    [[[window0]]]
      parent = ""
      size = 1200, 600
      type = Window
[plugins]
[profiles]
  [[default]]
    scrollbar_position = hidden
    scrollback_infinite = True
    use_system_font = False
    background_darkness = 0.9
    background_type = transparent
    background_image = None
    show_titlebar = False
    font = D2Coding 12
</pre>

저는 스크롤바가 화면에 있는게 더 편해서 위 설정에서 `scrollbar_position = hidden` 항목은 제외합니다.

<br>

## Terminator 단축키

* 위/아래로 화면 나누기 : <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>O</kbd>
* 좌/우로 화면 나누기 : <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>E</kbd>
* 현재 화면 닫기 : <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>W</kbd>

* 화면간 이동 : <kbd>Alt</kbd> + <kbd>방향키</kbd>

* 스크롤바 Toggle : <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>S</kbd>

* 검색 : <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>F</kbd>
* 화면 Clear : <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>G</kbd>