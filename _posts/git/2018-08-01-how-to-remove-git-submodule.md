---
layout: post
title: Git Submodule 삭제 방법
category: Git
tag: [git]
---
# Git Submodule 삭제 방법

먼저 `git submodule deinit -f` 명령어를 통해서 해당 모듈을 deinit 해줍니다.

<pre class="prettyprint">
git submodule deinit -f test_app
</pre>

그 다음 `.git/modules` 폴더에 들어가서 해당 폴더를 삭제합니다.

<pre class="prettyprint">
rm -rf .git/modules/test_app
</pre>

마지막으로 git에서 해당 폴더를 제거해주면 됩니다.

<pre class="prettyprint">
git rm -f test_app
</pre>

