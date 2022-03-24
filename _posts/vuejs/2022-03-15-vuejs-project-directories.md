---
layout: post
title: Vue.js 프로젝트 디렉토리 구성
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js]
---
# Vue.js 프로젝트 디렉토리 구성

## 프로젝트 만들기

터미널에서 `vue create` 명령어를 통해 Vue.js 프로젝트를 하나 생성합니다.

<pre class="prettyprint">
$ vue create snowdeer-vue-sample

Vue CLI v5.0.4
? Please pick a preset: Default ([Vue 3] babel, eslint)


Vue CLI v5.0.4
✨  Creating project in /Users/snowdeer/Workspace/vue/snowdeer-vue-sample.
🗃  Initializing git repository...
⚙️  Installing CLI plugins. This might take a while...

yarn install v1.22.17
info No lockfile found.
[1/4] 🔍  Resolving packages...
[2/4] 🚚  Fetching packages...
[3/4] 🔗  Linking dependencies...
[4/4] 🔨  Building fresh packages...
success Saved lockfile.
✨  Done in 8.84s.
🚀  Invoking generators...
📦  Installing additional dependencies...

yarn install v1.22.17
[1/4] 🔍  Resolving packages...
[2/4] 🚚  Fetching packages...
[3/4] 🔗  Linking dependencies...
[4/4] 🔨  Building fresh packages...
success Saved lockfile.
✨  Done in 23.39s.
⚓  Running completion hooks...

📄  Generating README.md...

🎉  Successfully created project snowdeer-vue-sample.
👉  Get started with the following commands:

 $ cd snowdeer-vue-sample
 $ yarn serve
</pre>

## 프로젝트 구조

### package.json

프로젝트에 필요한 패키지들은 `package.json`에 설치됩니다.
대략적인 내용은 다음과 같습니다.

<pre class="prettyprint">
{
  "name": "snowdeer-vue-sample",
  "version": "0.1.0",
  "private": true,
  "scripts": {
    "serve": "vue-cli-service serve",
    "build": "vue-cli-service build",
    "lint": "vue-cli-service lint"
  },
  "dependencies": {
    "core-js": "^3.8.3",
    "vue": "^3.2.13"
  },
  "devDependencies": {
    "@babel/core": "^7.12.16",
    "@babel/eslint-parser": "^7.12.16",
    "@vue/cli-plugin-babel": "~5.0.0",
    "@vue/cli-plugin-eslint": "~5.0.0",
    "@vue/cli-service": "~5.0.0",
    "eslint": "^7.32.0",
    "eslint-plugin-vue": "^8.0.3"
  },

  ...
</pre>

위 프로젝트에서 필요한 패키지들은 `dependencies`와 `devDependencies`를 보면 되며
`yarn add` 또는 `npm install` 등의 명령어로 필요 패키지들이 하나씩 추가됩니다.

### public/index.html

해당 프로젝트를 실행하면 `public/index.html`이 실행되며, 특별히 건드릴 부분은 거의 없습니다.

`<body>` 태그 내의 `<div id="app">`에 Vue.js가 렌더링해서 보여줍니다.

<pre class="prettyprint">
&lt;body&gt;
    ...
    &lt;div id="app"&gt;&lt;/div&gt;
    &lt;!-- built files will be auto injected --&gt;
    ...
&lt;/body&gt;
</pre>

### src/main.js

가장 먼저 실행되는 파일이라고 볼 수 있습니다. `createApp` 함수를 통해 프로젝트 전체 인스턴스에 대한 설정이 들어갑니다.

<pre class="prettyprint">
import { createApp } from 'vue'
import App from './App.vue'

createApp(App).mount('#app')
</pre>

### src/App.vue

본격적인 Vue 파일입니다. Vue 파일은 크게 다음의 3부분으로 이루어져 있습니다.

- `<template>` : `html` 코드가 작성됨
- `<script>` : `vue.js` 코드가 작성됨
- `<style>` : `css` 스타일 코드가 작성됨

이와 같이 하나의 파일에 `html`, `script`, `css`가 모두 작성되기 때문에 Vue.js는 SFC(Single-File Components)라고 합니다.

<pre class="prettyprint">
&lt;template&gt;
  &lt;img alt="Vue logo" src="./assets/logo.png"&gt;
  &lt;HelloWorld msg="Welcome to Your Vue.js App"/&gt;
&lt;/template&gt;

&lt;script&gt;
import HelloWorld from './components/HelloWorld.vue'

export default {
  name: 'App',
  components: {
    HelloWorld
  }
}
&lt;/script&gt;

&lt;style&gt;
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 60px;
}
&lt;/style&gt;

</pre>