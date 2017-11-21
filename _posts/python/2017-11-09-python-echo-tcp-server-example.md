---
layout: post
title: 파일 입출력
category: Python
tag: [Python]
---

일반적으로 파일 입출력(IO, Input/Output)은 다음과 같은 형태를 가집니다.

<pre class="prettyprint">
file = open(filepath, mode="r")
#...
file.close()
</pre>

하지만, `with` 문을 사용할 경우 구문이 종료되면 파일을 자동으로 닫아주기 때문에 좀 더 효율적으로 사용할 수 있습니다.

<pre class="prettyprint">
with open(filepath, mode="r") as file:
</pre>

파일에서 바이너리 데이터를 읽을 경우 다음과 같은 함수들을 사용할 수 있습니다.

* file.read() : 모든 데이터를 문자열이나 바이너리 형태로 읽는다.
* file.read(n) : n 개의 바이트를 읽는다.
* file.readline() : 한 라인을 읽는다.
* file.readlines() : 모든 라인을 문자열 리스트 형태로 읽는다.
* file.write(line) : 한 라인을 기록한다.
* file.writelines(lines) : 문자열 리스트를 기록한다.