---
layout: post
title: ZSH 기본 설정 변경
category: Linux
tag: [리눅스]
---
# ZSH 기본 설정 변경

`ZSH` 기본 설정은 `~/.zshrc` 파일에서 세팅할 수 있습니다. `~/.zshrc` 파일은 터미널을 처음 실행했을 때 실행되는 환경 변수 파일이기도 합니다. (`.bashrc` 파일과 유사)

<br>

## 기본 테마를 agnoster로 변경

`~/.zshrc` 파일을 열어서 다음과 같이 변경합니다. 기본 값은 `ZSH_THEME="robbyrussell"` 입니다.

<pre class="prettyprint">
ZSH_THEME="agnoster"
</pre>

만약 실행할 때마다 랜덤으로 테마를 변경하고 싶으면 다음과 같이 설정할 수도 있습니다. 

<pre class="prettyprint">
ZSH_THEME="random"
</pre>
