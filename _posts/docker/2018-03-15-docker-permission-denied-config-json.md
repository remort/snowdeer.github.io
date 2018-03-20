---
layout: post
title: '~/.docker/config.json : permission denied' 오류 발생시
category: Docker
permalink: /docker/:year/:month/:day/:title/

tag: [Docker]
---

이 문제는 `/.docker` 디렉토리가 `root`로 생성된 경우 발생하는 문제입니다. `sudo` 명령어를 이용해서 `docker` 명령을 수행하거나 다음 명령어를 이용해서 디렉토리 소유자를 변경하면 해결됩니다.

<pre class="prettyprint">
$ chown -R snowdeer ~/.docker
</pre>