---
layout: post
title: Ubuntu 네트워크 특정 포트 사용하는 프로세스 종료하는 명령어
category: Linux
tag: [리눅스]
---
# 특정 포트 사용하는 프로세스 확인

<pre class="prettyprint">
netstat -nap | grep :8081
</pre>

# 특정 포트 사용하는 프로세스 종료

<pre class="prettyprint">
fuser -k -n tcp 8081
</pre>