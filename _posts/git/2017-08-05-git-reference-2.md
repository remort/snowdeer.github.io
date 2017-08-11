---
layout: post
title: Git 설명서 - (2) 파일을 수정하고 저장하기
category: Git
tag: [git]
---
# 파일의 상태

Git 에서는 파일의 상태를 크게 4가지로 구분합니다.

* untracked : Git에 add 되지 않은 상태
* unmodified : Git에 add는 되었지만, 수정이 되지 않은 상태
* modified : 파일이 수정되었으나 commit 할 준비가 되지 않은 상태
* staged : 파일 수정이 완료되어 commit 만 하면 되는 상태

그리고 파일의 라이프 사이클은 다음과 같습니다.

 ![image](/assets/git-reference/01.png)


<br>

# 파일의 상태 확인

파일의 상태는 `git status` 명령어를 이용해서 확인할 수 있습니다.

최초 상태(init을 하거나 clone을 한 상태)에서 `git status` 명령어를 실행해보면 다음과 같은 결과가 출력될 것입니다.

<pre class="prettyprint">
$ git status
On branch master
nothing to commit, working directory clean
</pre>

현재 특별한 상태에 있는 파일이 하나도 없다는 뜻입니다.

새로운 파일을 하나 추가해봅니다. 'README.md' 파일을 폴더에 생성해보도록 하겠습니다.

<pre class="prettyprint">
$ git status
On branch master
Untracked files:
  (use "git add &lt;file&gt;..." to include in what will be committed)

    README.md

nothing added to commit but untracked files present (use "git add" to track)
</pre>

폴더에는 파일이 추가되었지만, 아직 Git에는 add 가 되지 않았기 때문에 위와 같은 결과가 출력됩니다.

<br>

## 새로 생성한 파일

`git add` 명령어를 이용해서 위에서 생성한 'README.md' 파일을 Git에 등록합니다.

<pre class="prettyprint">
$ git add README.md

$ git status
On branch master
Changes to be committed:
  (use "git reset HEAD &lt;file&gt;..." to unstage)

        new file:   README.md
</pre>

새로운 파일이 Git에 등록되었다는 문구가 출력되었습니다.

<br>

## 수정한 파일을 Staged 상태로 변경하기

'commit' 하기 위해서는 파일 수정이 완전히 끝나고 안정적(Staged)인 단계가 되어야 합니다. 즉, 파일 수정이 다 끝나면 해당 파일의 상태를 Staged 상태로 변경해주어야 합니다.

예를 들어, 'snowdeer.cpp' 라는 파일을 수정했다고 가정하면 `git status` 결과는 다음과 같이 출력될 것입니다.

<pre class="prettyprint">
$ git status
On branch master
Changes to be committed:
  (use "git reset HEAD &lt;file&gt;..." to unstage)

        new file:   README.md

Changes not staged for commit:
  (use "git add &lt;file&gt;..." to update what will be committed)
  (use "git checkout -- &lt;file&gt;..." to discard changes in working directory)

        modified:   snowdeer.cpp
</pre>

Staged 상태로 만들어주는 명령어는 `git add` 입니다. `git add`는 파일을 Git에서 관리하도록 등록해주는 역할도 하며, Staged 상태로 만들어주는 역할도 합니다.

<pre class="prettyprint">
$ git add snowdeer.cpp

$ git status
On branch master
Changes to be committed:
  (use "git reset HEAD &lt;file&gt;..." to unstage)

        new file:   README.md
        modified:   snowdeer.cpp
</pre>

만약 `git add`를 한 상태에서 또 다시 파일을 수정했다면 다시 한 번 `git add` 명령어를 이용해서 해당 수정분을 다시 Staged 상태로 만들어줘야 합니다.

<br>

## 파일 저장(commit)하기

새로 추가가 되었거나 Staged 상태의 파일들은 `git commit` 명령어를 이용해서 저장할 수 있습니다.

또한 다음과 같이 `-m` 옵션을 이용해서 변경 이력 로그를 남길 수도 있습니다.

<pre class="prettyprint">
$ git commit -m "minor bug fixed"
</pre>

<br>

Staged Area는 파일 수정이 완벽하게 이루어진 파일들만 관리하기 때문에 아주 유용하긴 하지만 일반적으로는 번거롭고 귀찮은 과정이기도 합니다.

그래서 `git commit`을 할 때 `-a` 옵션을 이용해서 Staged Area 단계를 건너뛰고 파일을 저장할 수도 있습니다.
