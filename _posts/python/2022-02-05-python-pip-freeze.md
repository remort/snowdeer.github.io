---
layout: post
title: pip3 freeze 명령어(requirements.txt)
category: Python
tag: [Python]
---

# pip3 freeze 명령어

`pip3 freeze` 명령어를 이용하면 다음과 같이 현재 사용하고 있는 파이썬 패키지 리스트를 확인할 수 있습니다.

<pre class="prettyprint">
$ pip3 freeze

Deprecated==1.2.13
packaging==21.3
pyparsing==3.0.7
redis==4.1.2
wrapt==1.13.3
</pre>

## 패키지 리스트 내보내기

다음 명령어를 이용해서 현재 사용중인 패키지 리스트를 `requirements.txt`에 저장할 수 있습니다.

<pre class="prettyprint">
pip3 freeze > requirements.txt
</pre>

## 패키지 리스트 가져와서 설치하기

<pre class="prettyprint">
pip3 install -r requirements.txt
</pre>