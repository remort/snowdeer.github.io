---
layout: post
title: Git 브랜치를 Master 브랜치로 변경하는 방법
category: Git
tag: [git]
---
# Git 브랜치를 Master 브랜치로 변경하는 법

* 참고 : https://code.i-harness.com/ko/q/2a28fe

그 전에 모든 수정 사항이 원격 저장소에 `push` 되어 있는지 확인해야 합니다. 

그 이후 다음 명령어를 입력합니다.

<pre class="prettyprint">
git checkout master
</pre>

`master`를 `better_branch`로 덮어 씁니다.

<pre class="prettyprint">
git reset --hard better_branch
</pre>

그 이후 강제로 원격 저장소에 `push` 해 줍니다.

<pre class="prettyprint">
git push -f origin master
</pre>