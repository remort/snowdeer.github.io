---
layout: post
title: Git 설명서 - (5) 되돌리기
category: Git
tag: [git]
---
# 되돌리기

실수를 했을 경우 했던 작업을 복원 및 되돌리기를 해야합니다. 다만, **한 번 되돌린 경우 복구를 할 수 없기 때문에 주의**해야 합니다.

## commit 되돌리기

종종 commit한 결과를 수정해야 할 경우가 있습니다. 어떤 파일을 누락하거나 commit 메세지를 수정할 경우도 이에 해당합니다. `--amend` 옵션을 이용하면 다시 commit을 수행할 수 있습니다.

<pre class="prettyprint">
$ git commit --amend
</pre>

만약 파일 추가를 빼먹었다면 다음과 같은 순서로 명령을 실행할 수 있습니다.

<pre class="prettyprint">
$ git commit -m 'minor bug fixed'

$ git add snowdeer_added.cpp

$ git commit --amend
</pre>

두 번째 commit은 첫 번째 commit를 덮어쓰기 때문에 여기서 실행한 명령어 3개는 모두 하나의 commit으로 기록됩니다.

<br>

## Staged 파일을 Unstaged 상태로 되돌리기

실수로 `git add *` 명령어를 실행하면 의도치 않은 파일들까지 Staged 상태로 되는 경우가 발생할 수 있습니다.

`git status`로 확인하면 다음과 같습니다.

<pre class="prettyprint">
$ git status
On branch master
Changes to be committed:
  (use "git reset HEAD &lt;file&gt;..." to unstage)

        modified:   snowdeer_1.cpp
        modified:   snowdeer_2.cpp
</pre>

여기서 설명에 나온 `git reset HEAD <file>` 명령어를 이용해서 Staged 상태의 파일을 Unstaged 상태로 만들어줄 수 있습니다.

<pre class="prettyprint">
$ git reset HEAD snowdeer_2.cpp
</pre>

<br>

## 수정한 파일을 원래 상태로 되돌리기

파일 수정을 했지만 서버에 있는 내용으로 복원하고 싶은 경우입니다. 자주 발생하는 경우이기도 합니다.

마찬가지로 `git status`를 실행해보면 복구하는 명령어를 조회할 수 있습니다.

<pre class="prettyprint">
Changes not staged for commit:
  (use "git add &lt;file&gt;..." to update what will be committed)
  (use "git checkout -- &lt;file&gt;..." to discard changes in working directory)

        modified:   snowdeer_1.cpp
</pre>

위 설명에 있는 `git checkout -- <file>` 명령어를 이용해서 아래와 같이 실행해주면 됩니다.

<pre class="prettyprint">
$ git checkout -- snowdeer_1.cpp
</pre>

이제 파일이 복원된 것을 확인하면 됩니다.

하지만 위와 같은 방법으로 복원한 것은 기존에 작업하던 내용을 전부 날려버리기 때문에 상당히 위험한 명령어입니다. 따라서 **각별히 주의해서 사용해야 할 것**입니다.

좀 더 안전하게 복원하고 싶으면 `stash` 명령어나 `branch`를 활용하는 것이 더 좋습니다.