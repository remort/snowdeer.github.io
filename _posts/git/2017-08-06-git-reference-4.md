---
layout: post
title: Git 설명서 - (4) 파일 삭제
category: Git
tag: [git]
---
# 파일 삭제

파일 삭제는 `git rm` 명령어를 이용해서 할 수 있습니다. 이 명령어를 수행하면 현재 폴더에 있는 파일도 같이 지워 줍니다.

만약 `git rm`이 아닌, `rm` 명령어로 로컬에 있는 파일만 삭제한 경우라면, `git status`에서 다음과 같은 결과를 볼 수 있습니다.

<pre class="prettyprint">
$ rm snowdeer.cpp

$ git status
On branch master
Changes not staged for commit:
  (use "git add/rm &lt;file&gt;..." to update what will be committed)
  (use "git checkout -- &lt;file&gt;..." to discard changes in working directory)

        deleted:    snowdeer.cpp

no changes added to commit (use "git add" and/or "git commit -a")
</pre>

이 경우는 파일 삭제한 상태가 아직 Staged 상태이지 않기 때문에 `git add` 명령어를 이용해서 파일 삭제 상태를 Staged 하게 만들어줘야 합니다.

`git rm`으로 삭제한 경우는 삭제된 파일은 Staged 상태가 되기 때문에 위 단계를 건너뛸 수 있습니다.

물론 Staged 상태에서 `commit`까지 해주어야 파일 삭제가 Git에 완전히 반영이 됩니다.

<br>

## 로컬 파일은 남기고 Git에서만 삭제하기

이런 경우는 보통 `.gitignore` 파일에 추가하는 작업을 빼먹었거나 실수로 불필요한 파일을 Git에 등록했을 때 필요한 작업입니다.

`--cached` 옵션을 사용하면 로컬에 있는 파일은 그대로 둔채 Git의 파일만 삭제를 할 수 있습니다.

<pre class="prettyprint">
git rm --cached snowdeer.bak
</pre>

<br>

## 복수의 파일이나 폴더 삭제

패턴을 이용하여 여러 개의 파일이나 폴더를 삭제할 수도 있습니다.

<pre class="prettyprint">
$ git rm log/\*.log
</pre>

위와 같은 명령어를 사용하면 'log' 폴더 아래에 있는 '.log' 확장자를 가지는 파일들을 모두 삭제합니다. `*` 앞에 `\`를 붙여야 합니다. (Windows 커맨드에서 실행할 때는 `\`를 붙이지 않습니다.)

<pre class="prettyprint">
$ git rm \*~
</pre>

이와 같은 코드는 `~`으로 끝나는 파일을 모두 삭제합니다.