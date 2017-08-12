---
layout: post
title: Git 설명서 - (10) 원격 브랜치
category: Git
tag: [git]
---
# 원격 브랜치

원격 브랜치는 원격 서버에 있는 브랜치를 말합니다. 물론 로컬에도 서버에서 Pull로 가져온 원격 브랜치 정보가 모두 존재합니다.

원격 브랜치는 '(remote)/(branch)' 형태의 이름을 가집니다. 예를 들어 `origin/master`와 같은 이름입니다.

다음 예제는 'git.ourcompany.com' 이라는 원격 Git 서버가 있으며, 이 서버로부터 clone 하여 로컬에 저장소를 가지는 경우에 대한 예제입니다.

clone을 하게 되면, 로컬에서는 원격 저장소의 별명을 `origin`이라는 이름을 기본(Default)으로 붙입니다. 즉, 다음 그림과 같은 형태의 소스 트리를 가지게 됩니다.

![image](/assets/git-reference/16.png)

<br>

## 다른 개발자가 원격 저장소의 master 브랜치에 commit 한 경우

이 경우 개발자간 히스토리는 서로 달라지게 되고 다음 그림과 같은 상태가 됩니다.

![image](/assets/git-reference/17.png)

원격 저장소의 내용을 로컬 저장소에 업데이트하려면 `git fetch origin` 명령을 사용해야 합니다. 이렇게 하면, 현재 로컬 저장소에 없는 서버의 내용을 모두 내려받게 되고 로컬의 `origin/master` 포인터를 가장 최신 commit으로 이동시켜줍니다.

![image](/assets/git-reference/18.png)

물론, 로컬에서 기존에 작업하던 내용이 있기 때문에 소스 분기는 이루어지게 됩니다. `fetch`로 내려받았기 때문에 소스 정합은 수동으로 해주어야 합니다.

`fetch`로 내려받은 브랜치의 내용을 정합하려면 `git merge origin/(브랜치 이름)`으로 명령을 내려주면 됩니다.

<br>

## 작업한 내용을 원격 저장소에 Push

로컬 저장소의 내용을 원격 저장소에 업로드할 때는 `push` 명령어를 이용합니다. `git push (remote) (branch)`와 같은 형태로 사용할 수 있습니다.

<pre class="prettyprint">
$ git push origin serverfix
Counting objects: 20, done.
Compressing objects: 100% (14/14), done.
Writing objects: 100% (15/15), 1.74 KiB, done.
Total 15 (delta 5), reused 0 (delta 0)
To git@github.com:schacon/simplegit.git
 * [new branch]      serverfix -> serverfix
</pre>

위 명령은 로컬의 'serverfix'라는 브랜치를 `origin`이라는 이름을 가진 원격 저장소에 'serverfix'라는 이름의 브랜치로 Push 하는 명령입니다.