---
layout: post
title: git 컨닝 페이퍼
category: Git
tag: [git]
---
# 기본 명령어
명령어 | 설명
--- | ---
git init | 현재 위치를 git 저장소로 초기화
git add [파일] | 해당 파일을 git에 등록하거나 수정된 파일을 추가
git commit | 변경된 내용을 커밋
git status | 현재 상태를 출력

### 응용
* git add . : 모든 파일을 git에 추가함
git commit --amend | 같은 브랜치 상의 최종 커밋을 취소하고 새로운 내용을 추가한 커밋을 할 수 있음

# 브랜치 관련 명령어
명령어 | 설명
--- | ---
git branch [이름] | 해당 이름의 브랜치를 생성
git checkout [브랜치이름] | 해당 이름의 브랜치로 변경
git merge [브랜치이름] | 현재 브랜치에 [브랜치이름]의 브랜치 내용을 정합

### 응용
* git branch -D [이름] | 해당 이름의 브랜치를 삭제
* git checkout -B [브랜치이름] | 해당 이름의 브랜치를 생성하면서 변경

# 원격 명령어
명령어 | 설명
--- | ---
git clone | 원격 저장소의 모든 내용을 로컬에 저장
git remote | 로컬 저장소를 원격 저장소에 연결
git push | 로컬 저장소의 내용을 원격 저장소에 전달
git fetch | 로컬 저장소와 원격 저장소의 변경 사항이 다를 때 이를 대조하고 최신 데이터를 반영함
git pull | 원격 저장소의 최신 내용을 로컬 저장소에 업데이트

# 고급 명령어
명령어 | 설명
--- | ---
git tag | 커밋을 참조하기 쉽도록 태그를 붙임
git revert | 이전에 작성한 커밋을 지우지만 지운 내역을 남김
git reset | 커밋을 버리고 이전의 특정 버전으로 되돌아감. `git revert`와의 차이는 지운 커밋 내역을 남기지 않는다는 점
git rebase | 브랜치 이력을 확인하면서 정합

### 응용
* git checkout -- [파일] | 커밋하지 않은 변경 내역을 취소
* git revase -i | 커밋 내용을 합치면서 정합
