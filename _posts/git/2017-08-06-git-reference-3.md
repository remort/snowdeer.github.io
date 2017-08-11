---
layout: post
title: Git 설명서 - (3) .gitignore
category: Git
tag: [git]
---
# 파일 무시하기

프로젝트를 구성하다보면, 어떤 파일들은 Git에서 관리할 필요가 없는 파일들이 있습니다. 예를 들어 Android Studio 프로젝트에 있는 `properties.local` 같은 파일들은 로컬 PC에 있는 설정값이 기록된 파일이기 때문에 굳이 Git에 올릴 필요가 없는 파일입니다.

이러한 파일들은 `.gitignore` 파일을 이용해서 관리할 수 있습니다.

`.gitignore`에서 사용하는 규칙은 다음과 같습니다.

* 아무 것도 없는 라인이나, # 으로 시작하는 라인은 무시
* 표준 Glob 패턴을 사용
* 폴더는 끝 부분에 '/'를 붙여서 표현
* 느낌표(!)로 시작하는 파일은 무시하지 않음

Glob 패턴은 일반적으로 사용하는 정규표현식의 간략화 버전이라고 생각하면 됩니다.

예를 들면 다음과 같이 사용할 수 있습니다.
다음 예제는 제가 현재 Android Studio 또는 Eclipse에서 사용하는 `.gitignore` 파일의 내용입니다.

<pre class="prettyprint">
*.iml
.gradle
/.idea/workspace.xml
/.idea/libraries
.DS_Store
/build
/captures
.externalNativeBuild

*.apk
*.ap_
*.dex
bin/
gen/
local.properties
.classpath
.project
proguard/
*.ipr
*.iws
</pre>