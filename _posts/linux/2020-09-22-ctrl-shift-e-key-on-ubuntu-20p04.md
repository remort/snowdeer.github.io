---
layout: post
title: Ctrl + Shift + E 명령어가 안 먹히는 경우(Terminator)

category: Linux
permalink: /mac-os/:year/:month/:day/:title/
tag: [리눅스]
---
저는 Ubuntu 20.04 버전부터 경험했지만, 찾아보니 그 이전 버전에서도 발생했던 문제네요.

`terminator` 터미널 프로그램에서 <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>E</kbd> 명령어를 입력하니
창이 가로로 분할되는 것이 아니라 `e`라는 글자와 함께 이모지 입력 프롬프트로 바뀌는 현상이 있었습니다.

수정하기 위해서는 터미널에서

<pre class="prettyprint">
ibus-setup
</pre>

그 이후 `Emoji` 탭을 선택합니다.

`Emoji annotation` 키보드 단축키가 <kbd>Ctrl</kbd> + <kbd>Shift</kbd> + <kbd>E</kbd> 으로 되어 있는 것이 보이는데
그냥 지워버리면 됩니다.