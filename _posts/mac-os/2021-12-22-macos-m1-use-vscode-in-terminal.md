---
layout: post
title: 터미널에서 code 명령어 실행할 수 있도록 설정하는 방법

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# 터미널에서 code 명령어 실행할 수 있도록 설정
alias 등을 이용해서 터미널에서 vscode를 실행할 수도 있지만, 아예 vscode 프로그램 내에서 설정하는 방법이 더 깔끔한 것 같습니다.

vscode를 실행한다음 <kbd>Command</kbd> + <kbd>Shift</kbd> + <kbd>P</kbd>를 입력해서 Command Palette를 실행한 다음
`shell`이라고 입력하면, `Shell Command: Install 'code' in PATH`를 선택합니다. 그러면 이제 터미널에서 `code` 명령어를 이용해서
vscode를 실행할 수 있습니다.