---
layout: post
title: Mac OS에서 IntelliJ를 이용해서 Flutter 사용하기
category: Flutter

tag: [Flutter]
---

`Intelli J`에서는 `PATH` 경로를 체크해서 `Flutter`의 설치 여부를 확인합니다.

`.zshrc`에 다음과 같은 `alias`를 추가해준 다음 터미널을 통해 `intelliJ`로 실행하면
`Intelli J`에서 `Flutter`를 사용할 수 있습니다.

<pre class="prettyprint">
# for Flutter
export PATH=$PATH:~/Development/flutter/bin

alias intelliJ="open '/Applications/IntelliJ IDEA CE.app/'"
</pre>