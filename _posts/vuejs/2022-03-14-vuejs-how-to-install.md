---
layout: post
title: Vue.js 설치 방법(MacOS)
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js]
---
# Vue.js 설치 방법

## Node.js 설치

<pre class="prettyprint">
$ brew install node

$ node -v
v17.7.1

$ npm -v
8.5.2
</pre>

## Yarn 설치

위에서 `Node.js`를 이미 설치했기 때문에 `--ignore-dependencies` 옵션을 주고 설치합니다.

<pre class="prettyprint">
$ brew install yarn --ignore-dependencies

$ yarn -v
1.22.17
</pre>

## Vue-Cli 설치

<pre class="prettyprint">
npm install @vue/cli -g

또는 

yarn global add @vue/cli
</pre>

## 샘플 프로젝트 생성해보기

<pre class="prettyprint">
$ vue create snow-vue-sample

?  Your connection to the default yarn registry seems to be slow.
   Use https://registry.npmmirror.com for faster installation? Yes


Vue CLI v5.0.3
? Please pick a preset: Default ([Vue 3] babel, eslint)
? Pick the package manager to use when installing dependencies: Yarn


Vue CLI v5.0.3
✨  Creating project in /Users/snowdeer/Workspace/snow-vue-sample.
🗃  Initializing git repository...
⚙️  Installing CLI plugins. This might take a while...

yarn install v1.22.17
info No lockfile found.
[1/4] 🔍  Resolving packages...
[2/4] 🚚  Fetching packages...
[3/4] 🔗  Linking dependencies...

success Saved lockfile.
✨  Done in 45.85s.
🚀  Invoking generators...
📦  Installing additional dependencies...

yarn install v1.22.17
[1/4] 🔍  Resolving packages...
[2/4] 🚚  Fetching packages...
[3/4] 🔗  Linking dependencies...
[4/4] 🔨  Building fresh packages...

success Saved lockfile.
✨  Done in 9.32s.
⚓  Running completion hooks...

📄  Generating README.md...

🎉  Successfully created project snow-vue-sample.
👉  Get started with the following commands:

 $ cd snow-vue-sample
 $ yarn serve
</pre>

## Chrome의 Vue.js devtools 설치

[Chrome 웹스토어](https://chrome.google.com/webstore/detail/vuejs-devtools/nhdogjmejiglipccpnnnanhbledajbpd)에서 `Vue.js devtools` 플러그인을 설치합니다.

