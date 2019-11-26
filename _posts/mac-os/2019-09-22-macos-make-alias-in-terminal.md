---
layout: post
title: 터미널에서 프로그램 실행 명령어 할당하기

category: MAC OS
permalink: /mac-os/:year/:month/:day/:title/
tag: [MAC OS]
---
# 프로그램 실행 명령어 할당

예를 들어 [비주얼 스튜디오 코드](https://code.visualstudio.com)를 사이트에서 `dmg` 파일을 다운받아 설치할 경우
터미널에 `vscode` 나 `code` 같은 프로그램 실행 명령어가 동작하지 않습니다.
(리눅스에서 `snap` 등으로 `VSCode`를 설치했을 경우, 과거에는 단축 명령어가 `vscode`, 현재는 `code` 입니다.)

<br>

## alias

이 경우 `alias` 명령어를 통해 특정 프로그램 실행 명령어를 할당할 수 있습니다.

<pre class="prettyprint">
alias code='open -a "Visual Studio Code"'
</pre>

<br>

## 함수 생성

터미널용 함수를 만들어서 사용할 수도 있습니다.

<pre class="prettyprint">
code () {
    if [[ $# = 0 ]]
    then
        open -a "Visual Studio Code"
    else
        echo "Opening: "$@
        "/Applications/Visual Studio Code.app/Contents/MacOS/Electron" $@
    fi
}
</pre>

<br>

## Symbolic Link

아래와 같은 방법으로 심볼릭 링크(Synmolic link)를 이용해서 실행 명령어를 생성할 수 있습니다.

<pre class="prettyprint">
ln -s /Applications/Visual\ Studio\ Code.app/Contents/Resources/app/bin/code ~/bin/code
</pre>

<br>

## VSCode 내에서 할당

`VSCode` 같은 경우는 <kbd>Command</kbd> + <kbd>Shift</kbd> + <kbd>P</kbd>를 누르고 `Shell`을 타이핑하고
`Shell Command: Install 'code' Command in PATH` 항목을 선택하면 됩니다.

이 경우는 `/usr/local/bin/code`에 실행파일이 생성되며, `which code`로 확인할 수 있습니다.