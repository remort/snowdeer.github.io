---
layout: post
title: git rebase
category: Git
tag: [git]
---
# git rebase

`git rebase`는 말 그대로 base를 다시 정한다는 말입니다. 
주로 특정 브랜치에서 commit 된 내용을 전부 `master`에 반영할 때 사용합니다.

## git rebase 순서

만약 `master` 브랜치가 있고, 여기에 작업을 한 `hotfix` 브랜치가 있다고 하면,

- git checkout hotfix : `hotfix` 브랜치로 이동
- git rebase master : 현재 `master`를 `hotfix` 브랜치의 앞 쪽에 base 설정함

의 순서로 진행하면 됩니다. 현재 작업 중인 브랜치의 base를 `master`로 설정함으로써, 현재 `master`에 이어서 작업 중인 `hotfix` 브랜치의 commit들을 연결할 수 있습니다.