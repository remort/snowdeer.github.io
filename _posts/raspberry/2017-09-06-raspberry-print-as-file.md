---
layout: post
title: 커맨드라인에서 파일로 출력하기
category: 라즈베리파이
permalink: /raspberry/:year/:month/:day/:title/

tag: [라즈베리파이, 리눅스]
---
# 커맨드라인에서 파일로 출력하기

커맨드라인에서 파일로 출력하려면 `>` 명령을 사용하면 됩니다. 또한 파일 복사에도 활용할 수 있습니다. 예를 들면 다음과 같이 활용할 수 있습니다.

~~~
$ ls > files.txt

$ more files.txt
Desktop
hello.txt
snowdeer.zip
~~~

<br>

# 파일 병합

`>` 명령어를 이용하면 여러 개의 파일들을 하나의 큰 파일로 합칠 수도 있습니다. 예를 들면 다음과 같습니다.

~~~
$ cat file1.txt file2.txt file3.txt > total_file.txt
~~~
