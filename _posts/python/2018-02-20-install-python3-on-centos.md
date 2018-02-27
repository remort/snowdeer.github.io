---
layout: post
title: CentOS에 Python 3.x 설치하는 방법
category: Python
tag: [Python]
---
# CentOS에 Python 3.x 설치

## Repository를 yum에 추가

<pre class="prettyprint">
$ sudo yum install -y https://centos7.iuscommunity.org/ius-release.rpm
</pre>

<br>

## yum search로 python 3.x 버전 확인

아래 명령어를 수행하면 python3으로 시작하는 라이브러리들을 확인할 수 있습니다.

<pre class="prettyprint">
$ yum search python3
</pre>

</pre>

## 필요 라이브러리들 설치

<pre class="prettyprint">
$ sudo yum install -y python36u python36u-libs python36u-devel python36u-pip
</pre>

<br>

## Python 설치 및 버전 확인

<pre class="prettyprint">
$ python -V
Python 2.7.5

$ python3.6 -V
Python 3.6.4
</pre>

기본적으로 `python` 명령어는 Python 2.x를 가리키고 있기 때문에 Alias 수정으로 이를 변경해줍니다.

<br>

## Alias 수정

먼저 Python 3.x가 설치되어 있는 디렉토리 위치를 찾습니다.

 <pre class="prettyprint">
 $ which python3.6
 /usr/bin/python3.6
 </pre>

그 다음 현재 Alias 확인을 합니다.

<pre class="prettyprint">
$ ls -l /bin/python*
</pre>

Alias 수정을 합니다.

<pre class="prettyprint">
$ sudo unlink /bin/python

$ sudo ln -s /bin/python3.6 /bin/python3

$ sudo ln -s /bin/python3.6 /bin/python

$ sudo ln -s /bin/pip3.6 /bin/pip
</pre>

