---
layout: post
title: Git 설명서 - (9) 브랜치 관리
category: Git
tag: [git]
---
# 브랜치 관리

`git branch` 명령어는 아무런 옵션없이 실행할 경우 브랜치의 리스트 및 현재 checkout 되어 있는 브랜치를 보여줍니다.

<pre class="prettyprint">
$ git branch
  iss53
* master
  testing
</pre>

`-v` 옵션을 붙여서 실행할 경우에는 각 브랜치의 마지막 commit 메세지를 보여줍니다.

<pre class="prettyprint">
$ git branch -v
  iss53   93b412c fix javascript issue
* master  7a98805 Merge branch 'iss53'
  testing 782fd34 add scott to the author list in the readmes
</pre>

`--merged` 옵션이나 `--no-merged` 옵션을 이용해서 현재 checkout 한 브랜치 기준으로 정합 작업이 이루어진 브랜치 또는 그렇지 않은 브랜치 리스트를 볼 수 있습니다.

<pre class="prettyprint">
$ git branch --merged
  iss53
* master
</pre>

위에서 '*' 가 붙지 않은 브랜치는 이미 정합 작업이 끝난 브랜치이기 때문에 삭제를 해도 되는 브랜치입니다. `git branch -d` 명령어로 해당 브랜치를 삭제할 수도 있습니다.

정합이 되지 않은 브랜치의 경우에는 `-d` 옵션으로 삭제가 되지 않습니다. 이 경우에는 `-D` 옵션으로 강제적으로 삭제를 할 수도 있습니다. 