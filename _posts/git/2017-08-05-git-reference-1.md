---
layout: post
title: Git 설명서 - (1) 저장소 만들기
category: Git
tag: [git]
---
# Git 저장소 만들기

Git 저장소를 만드는 방법은 크게 2 가지 방법이 있습니다.

* 현재 프로젝트를 init 하는 방법
* 서버에서 clone 해서 가져오는 방법

<br>

## 현재 프로젝트를 init 하는 방법

현재 프로젝트가 있는 폴더에서 다음 명령어를 입력합니다.

<pre class="prettyprint">
$ git init
</pre>

이 명령은 현재 폴더에 `.git` 이라는 하위 폴더를 하나 만들어줍니다. 여기에는 Git 저장소에 필요한 아주 기본적인 파일들만 들어있으며, 아직 어떤 파일도 관리되고 있지 않습니다. Git이 각 파일들을 관리하게 하기 위해서는 `git add` 명령을 이용해서 파일들을 등록해주어야 합니다.

<pre class="prettyprint">
$ git add *.cpp
$ git add README.md
$ git commit -m 'initial commit'
</pre>

<br>

## 서버에서 clone 하는 방법

기존에 이미 구현되어 있는 서버의 파일들을 로컬에 모두 가져오는 방법입니다. `git clone` 명령어를 이용하며, 프로젝트의 모든 히스토리(History) 내역까지 모두 가져옵니다.

다음과 같은 방법으로 사용할 수 있습니다.

<pre class="prettyprint">
$ git clone https://github.com/snowdeer/BeaconSample.git
</pre>

위와 같은 명령어를 실행하면, 현재 폴더에 git 저장소의 이름에 해당하는 'BeaconSample'이라는 하위 폴더를 생성하고 그 안에 소스 폴더들을 다운로드합니다. 그리고 가장 최신 버전의 소스로 `checkout` 해줍니다.

만약 폴더 이름을 'BeaconSample'가 아닌 다른 이름으로 변경하고 싶으면

<pre class="prettyprint">
$ git clone https://github.com/snowdeer/BeaconSample.git SnowBeaconSample
</pre>

와 같은 형태로 사용하면 됩니다.