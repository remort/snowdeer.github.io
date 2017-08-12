---
layout: post
title: Git 설명서 - (8) 브랜치와 Merge
category: Git
tag: [git]
---
# 브랜치와 Merge 진행 단계

일반적으로 브랜치를 생성하고 소스를 Merge 하는 단계는 다음과 같습니다.

1. 현재 진행중인 프로젝트에서 새로운 작업을 위해 신규 브랜치를 생성
2. 새로운 브랜치에서 작업을 진행함
3. 그런데 갑자기 **기존 브랜치에서 큰 버그가 발생**해서 긴급 대처를 해야 함
4. 기존 브랜치로 `checkout`
5. 기존 브랜치에서 'Hotfix' 브랜치를 새로 생성
6. 'Hotfix' 브랜치에서 작업한 결과를 기존 브랜치에 정합함

<br>

## 새로운 브랜치 생성

실제 브랜치를 생성하고 정합하는 단계를 조금 더 자세히 살펴보도록 하겠습니다.

<pre class="prettyprint">
$ git branch iss53

$ git checkout iss53
Switched to a new branch 'iss53'
</pre>

현재 브랜치에서 'iss53'이라는 새로운 브랜치를 생성하고 checkout을 하면 소스 트리는 다음 그림과 같은 상태가 됩니다.

![image](/assets/git-reference/10.png)

이 상태에서 'iss53'에서 작업을 계속 진행하고 commit도 하게 되면 다음 그림과 같은 상태가 됩니다.

![image](/assets/git-reference/11.png)

<br>

## 기존 브랜치에 Hotfix 브랜치를 생성

이 때, 기존 브랜치에서 급한 문제가 발생해서 Hotfix를 적용해야 할 상황이 발생했습니다. `master` 브랜치로 checkout 한 다음 'hotfix' 브랜치를 새로 생성하고 checkout 합니다.

<pre class="prettyprint">
$ git checkout master

$ git branch hotfix

$ git checkout hotfix
Switched to a new branch 'hotfix'
</pre>

![image](/assets/git-reference/12.png)

'hotfix'에서 작업을 수행하고 해당 내용을 `master` 브랜치에 반영하기 위해서는 먼저 `master` 브랜치로 checkout 한 다음 `merge`를 해줍니다.

<pre class="prettyprint">
$ git checkout master

$ git merge hotfix
Updating f42c576..3a0874c
Fast-forward
 README | 1 -
 1 file changed, 1 deletion(-)
</pre>

그러면 아래 그림과 같이 자동으로 정합이 되면서 `master` 브랜치도 가장 마지막 브랜치로 이동이 됩니다.

![image](/assets/git-reference/13.png)

여기서 `git branch` 명령어에 `-d` 옵션을 줘서 'hotfix' 브랜치를 삭제합니다.

<pre class="prettyprint">
$ git branch -d hotfix
Deleted branch hotfix (was 3a0874c).
</pre>

![image](/assets/git-reference/14.png)

그리고 원래 브랜치를 분리해서 작업을 하던 'iss53' 브랜치로 이동을 해서 다시 작업을 계속합니다.

작업이 다 끝난 후 다시 `master` 브랜치로 checkout 한 다음 'iss53' 브랜치의 내용을 정합합니다.

<pre class="prettyprint">
$ git checkout master
$ git merge iss53
Auto-merging README
Merge made by the 'recursive' strategy.
 README | 1 +
 1 file changed, 1 insertion(+)
 </pre>

![image](/assets/git-reference/15.png)

<br>

## Merge 작업 도중 충돌 발생한 경우

여러 명의 개발자가 동시에 소스를 수정하다보면 충돌(Conflict)이 발생하기도 합니다. 이 때는 다른 버전 관리 시스템들과 마찬가지로 Git도 자동으로 Merge를 해주지 못합니다. 개발자가 직접 Merge 작업을 해주어야 합니다.

`git status` 명령어를 이용하면 어떤 파일이 충돌났는지 알 수가 있습니다.

<pre class="prettyprint">
$ git status
On branch master
You have unmerged paths.
  (fix conflicts and run "git commit")

Unmerged paths:
  (use "git add <file>..." to mark resolution)

        both modified:      snowdeer.cpp

no changes added to commit (use "git add" and/or "git commit -a")
</pre>

충돌이 난 파일은 프로그래머가 직접 수정을 해주어야 하며, 수정이 다 된 후에는 `git add` 명령어로 Staged 한 상태로 만들어 주면 됩니다. 물론 그 이후에 commit 까지 해주어야 합니다.