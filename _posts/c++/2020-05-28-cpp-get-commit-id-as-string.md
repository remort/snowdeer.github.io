---
layout: post
title: Git Commit Id를 문자열로 획득하는 방법
category: C++
tag: [C++]
---
# Git Commit Id 및 날짜 획득 방법

## CMakeLists.txt 예제

`CMakeLists.txt` 파일에 다음 명령어를 추가하면 `Git Commit Id` 및 `Commit Date`를 얻을 수 있습니다.
`add_definitions` 명령어를 이용해서 C++ 코드에 전달할 수 있습니다.

<pre class="prettyprint">
execute_process(
		COMMAND git log -1 --format=%h
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		OUTPUT_VARIABLE GIT_COMMIT_HASH
		OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(
		COMMAND git log -1 --format=%cd
		WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
		OUTPUT_VARIABLE GIT_COMMIT_DATE
		OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_definitions("-DVERSION_HASH=\"${GIT_COMMIT_HASH}\"")
add_definitions("-DCOMMIT_DATE=\"${GIT_COMMIT_DATE}\"")
</pre>

<br>

## C++ 예제

실제 C++ 코드에서 사용하는 예제는 다음과 같습니다. 

<pre class="prettyprint">
#ifndef VERSION_HASH
#define VERSION_HASH "0000000"
#endif

#ifndef COMMIT_DATE
#define COMMIT_DATE ""
#endif

string Prefs::GET_VERSION() {
  return VERSION_HASH;
}

string Prefs::GET_COMMIT_DATE() {
  return COMMIT_DATE;
}
</pre>