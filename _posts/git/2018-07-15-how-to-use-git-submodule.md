---
layout: post
title: Git Submodule 사용 방법
category: Git
tag: [git]
---
# Git Submodule 사용 방법

Submodule을 추가하려는 부모를 `parent`라고 하고, Submodule로써 추가되려는 자식을 `child`라고 가정할 때, 다음과 같이 사용할 수 있습니다.

<br>

## git submodule 추가 방법

아직 `parent`에 submodule이 추가되지 않은 상태에서 다음 명령어를 입력하여 submodule 추가를 해줄 수 있습니다.

<pre class="prettyprint">
git clone git@github.com:snowdeer/parent.git

cd parent
git submodule add git@github.com:snowdeer/child.git child
git commit -m "submodule is added."
git push
</pre>

이 때, 별도로 `git add` 절차 없이 바로 `commit` 할 수 있습니다.

<br>

## git clone으로 parent를 가져왔을 때

`git clone`으로 `parent`를 가져왔을 때, 내부의 `child`는 디렉토리만 만들어져 있고 내부가 없습니다. 이 때 submodule 초기화 및 업데이틀 해 주어야 합니다. 루트에서 다음 명령어를 실행하면 됩니다.

<pre class="prettyprint">
git submodule init

git submodule update
</pre>

다만, 이 때 submodule의 소스 버전은 최신 버전을 가리키는 것이 아니라, `submodule add`를 수행했을 때의 버전을 가리키고 있습니다. submodule은 리파지토리가 실제로는 분리되어 있기 때문에 각 모듈의 버전이 따로 관리되는데, `parent` 프로젝트에서는 현재 submodule의 버전이 최신인지 아닌지 신경쓸필요없이 안정적인 특정 버전만 가리키면 되기 때문에 프로젝트 배포 등에서는 관리가 수월한 장점이 있습니다. 물론, 개발중인 프로젝트에서는 각 submodule들을 최신 버전으로 유지해야 할 경우 각 submodule들의 업데이트를 수동으로 한 번씩 더 해줘야 하는 단점이 있기도 합니다.

<br>

## submoudle 최신 버전으로 교체

submodule을 최신 버전으로 교체하는 방법은 다음 같습니다.

* `child` 디렉토리에 들어가서 각각의 submodule들을 개별 업데이트 해주는 방식. 각 submodule 디렉토리에서 `git pull` 명령어나 `git checkout` 명령어 등을 이용해서 업데이트 가능.
* `parent`내에서 `git submodule foreach git pull origin master` 명령어를 실행하여 하위 submodule들을 전부 업데이트 해주는 방법