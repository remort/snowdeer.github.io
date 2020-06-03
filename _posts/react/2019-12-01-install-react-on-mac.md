---
layout: post
title: React MacOS에 설치하는 방법
category: React
permalink: /openshift/:year/:month/:day/:title/

tag: [React]
---
# React 설치

`brew`를 이용해서 설치하는 방법을 포스팅합니다.

<br>

## brew 설치

<pre class="prettyprint">
$ xcode-select –install

$ ruby -e “$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)”
</pre>

<br>

## NodeJS 설치

<pre class="prettyprint">
brew install node
</pre>

<br>

## npm 업데이트

만약 `npm` 버전이 낮은 경우는 아래 명령어를 이용해서 최신 버전으로 업데이트할 수 있습니다.

<pre class="prettyprint">
npm install -g npm
</pre>

<br>

## yarn 설치

<pre class="prettyprint">
brew install yarn
</pre>

<br>

## create-react-app 설치

<pre class="prettyprint">
npm install -g create-react-app
</pre>

<br>

## 테스트 프로젝트 생성

<pre class="prettyprint">
create-react-app snowdeerapp
</pre>

<br>

## 실행 

<pre class="prettyprint">
cd snowdeerapp
yarn start
</pre>

그 이후 `http://localhost:3000`에 접속해서 확인할 수 있습니다.

