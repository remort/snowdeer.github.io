---
layout: post
title: Git 설명서 - (6) 원격 저장소 연결
category: Git
tag: [git]
---
# 원격 저장소

여러 명의 인원이 프로젝트를 개발하다보면 결국은 원격 저장소를 활용하게 될 가능성이 높습니다.

## 원격 저장소 확인

`git remote` 명령어를 이용해서 현재 프로젝트가 어떤 원격 저장소에 연결되었는지 확인할 수 있습니다.

만약 clone해서 받은 프로젝트라면 폴더에서 다음과 같이 `git remote` 명령어를 실행해보도록 합시다.

<pre class="prettyprint">
$ git remote
origin
</pre>

만약 `-v` 옵션을 이용하면 서버의 저장소 이름과 URL 까지 같이 볼 수 있습니다.

<pre class="prettyprint">
$ git remote -v
origin  https://github.com/snowdeer/snowdeer.android.service.git (fetch)
origin  https://github.com/snowdeer/snowdeer.android.service.git (push)
</pre>

<br>

## 원격 저장소의 데이터 가져오기

원격 저장소의 데이터를 가져오려면 `git fetch [remote name]` 명령어를 이용할 수 있습니다.
(원격 저장소를 clone 했을 경우, 기본적으로 원격 저장소 이름은 'origin'이 됩니다.)

<pre class="prettyprint">
$ git fetch origin
</pre>

`fetch` 명령어는 로컬에는 없지만, 원격 저장소에는 있는 데이터를 모두 가져옵니다. 그래서 원격 저장소의 모든 브랜치(Branch)를 로컬에서 접근할 수 있어서 소스 정합을 쉽게 할 수 있도록 해줍니다.

다만 `fetch`는 소스 정합을 자동으로 해주지는 않습니다. 사용자가 일일이 직접 정합을 해주어야 하는데, 오히려 직접 확인하면서 정합을 할 수 있기 때문에 자동으로 정합해주는 것보다 더 추천하는 방법입니다.

만약 자동으로 정합까지 하도록 하고 싶으면 `git pull` 명령어를 이용하면 됩니다.

<br>

## 원격 저장소에 업로드

로컬에서 작업한 내용을 원격 저장소에 업로드 할 때는 `git push [원격 저장소 이름] [브랜치 이름]` 명령어를 이용해서 수행할 수 있습니다.

'master' 브랜치를 원격 저장소 'origin'에 업로드하는 명령어는 다음과 같습니다.

<pre class="prettyprint">
$ git push origin master
</pre>

<br>

## 원격 저장소 정보 조회

원격 저장소의 정보 조회는 `git remote show [리모트 저장소 이름]`으로 획득할 수 있습니다.

<pre class="prettyprint">
$ git remote show origin
* remote origin
  Fetch URL: https://github.com/snowdeer/snowdeer.android.service.git
  Push  URL: https://github.com/snowdeer/snowdeer.android.service.git
  HEAD branch: master
  Remote branch:
    master tracked
  Local branch configured for 'git pull':
    master merges with remote master
  Local ref configured for 'git push':
    master pushes to master (up to date)
</pre>

원격 저장소의 URL 및 브랜치 정보 등을 조회할 수 있습니다. 

<br>

## 원격 저장소의 이름 수정 및 삭제

원격 저장소의 이름은 `git remote rename` 명령어를 이용해서 변경할 수 있습니다. 

다음과 같은 코드를 실행하면 원격 저장소의 이름을 'origin'에서 'snowdeer_origin'으로 변경합니다.

<pre class="prettyprint">
$ git remote rename origin snowdeer_origin

$ git remote
snowdeer_origin
</pre>

원격 저장소 삭제는 `git remote rm` 명령어를 이용합니다.

<pre class="prettyprint">
$ git remote rm snowdeer_origin
</pre>