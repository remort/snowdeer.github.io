---
layout: post
title: URL 주소 문자열 및 파라메터
category: Node.js
tag: [Node.js]
---

Node.js에서 URL 주소값의 구조를 파악해보고, URL에 포함된
Request 파라메터들은 어떻게 가져올 수 있는지 간단한 코드로 알아봅시다.

<pre class="prettyprint" style="font-size:0.7em;">
var url = require("url");

var originalUrl = "https://search.naver.com/search.naver?where=nexearch&query=snowdeer";
console.log("원본 URL : " + originalUrl);

var currentUrl = url.parse(originalUrl);
console.log("- 현재 URL 정보 -");
console.dir(currentUrl);

var queryString = require("querystring");
var params = queryString.parse(currentUrl.query);

console.log("\n원본 param : " + queryString.stringify(params));
console.log("- 현재 Param 정보 -");
console.dir(params);
console.log("where : " + params.where);
console.log("query : " + params.query);
</pre>

실행 결과는 다음과 같습니다.


<pre class="prettyprint" style="font-size:0.7em;">
원본 URL : https://search.naver.com/search.naver?where=nexearch&query=snowdeer
- 현재 URL 정보 -
Url {
  protocol: 'https:',
  slashes: true,
  auth: null,
  host: 'search.naver.com',
  port: null,
  hostname: 'search.naver.com',
  hash: null,
  search: '?where=nexearch&query=snowdeer',
  query: 'where=nexearch&query=snowdeer',
  pathname: '/search.naver',
  path: '/search.naver?where=nexearch&query=snowdeer',
  href: 'https://search.naver.com/search.naver?where=nexearch&query=snowdeer' }

원본 param : where=nexearch&query=snowdeer
- 현재 Param 정보 -
{ where: 'nexearch', query: 'snowdeer' }
where : nexearch
query : snowdeer
</pre>
