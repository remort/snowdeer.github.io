---
layout: post
title: Vue.js 3 npm json-server 패키지
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js]
---
# json-server 패키지

`json-server`는 json 파일을 이용해서 간단한 REST 서버를 만들어주는 npm 패키지입니다.

[여기](https://www.npmjs.com/package/json-server)에서 설명을 볼 수 있습니다.

## 설치 방법

<pre class="prettyprint">
npm install -g json-server
</pre>

## Database 파일 작성

`db.json` 파일을 다음과 같이 작성합니다.

<pre class="prettyprint">
{
  "posts": [
    { "id": 1, "title": "json-server", "author": "typicode" }
  ],
  "comments": [
    { "id": 1, "body": "some comment", "postId": 1 }
  ],
  "profile": { "name": "typicode" }
}
</pre>

## 실행 방법

<pre class="prettyprint">
json-server --watch db.json

\{^_^}/ hi!

  Loading db.json
  Done

  Resources
  http://localhost:3000/posts
  http://localhost:3000/comments
  http://localhost:3000/profile

  Home
  http://localhost:3000

  Type s + enter at any time to create a snapshot of the database
  Watching...

</pre>

`json` 파일에 따라 REST API가 자동으로 만들어지며, `json` 파일을 수동으로 변경하더라도 자동으로 서버에 적용됩니다.