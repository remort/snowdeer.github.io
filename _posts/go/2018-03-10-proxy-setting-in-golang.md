---
layout: post
title: Go 언어 Proxy 설정 방법
category: Go
tag: [Go]
---
# Proxy Setting

`go get` 명령어를 사용할 때 Proxy를 거치도록 하는 방법입니다.

다음과 같은 방법으로 사용할 수 있습니다.

<pre class="prettyprint">
$ http_proxy=127.0.0.1:8080 go get github.com/revel/revel
</pre>

`alias` 명령을 이용하면 좀 더 편리하게 사용할 수 있습니다.

<pre class="prettyprint">
$ alias go='http_proxy=127.0.0.1:8080 go'
</pre>