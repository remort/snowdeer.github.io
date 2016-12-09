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

var originalUrl = "https://www.google.co.kr/?gfe_rd=cr&ei=LixKWJnOGtTC8geL06zgBQ#q=snowdeer+web+scraper";
console.log("# 원본 URL : " + originalUrl);

var currentUrl = url.parse(originalUrl);
console.log("# 현재 URL 정보");
console.dir(currentUrl);

var queryString = require("querystring");
var params = queryString.parse(currentUrl.query);

console.log("\n# 원본 param : " + queryString.stringify(params));
console.log("# 현재 Param 정보");
console.dir(params);
console.log("gfe_rd : " + params.gfe_rd);
console.log("ei : " + params.ei);
</pre>

실행 결과는 다음과 같습니다.


~~~
# 원본 URL : https://www.google.co.kr/?gfe_rd=cr&ei=LixKWJnOGtTC8geL06zgBQ#q=snowdeer+web+scraper
# 현재 URL 정보
Url {
  protocol: 'https:',
  slashes: true,
  auth: null,
  host: 'www.google.co.kr',
  port: null,
  hostname: 'www.google.co.kr',
  hash: '#q=snowdeer+web+scraper',
  search: '?gfe_rd=cr&ei=LixKWJnOGtTC8geL06zgBQ',
  query: 'gfe_rd=cr&ei=LixKWJnOGtTC8geL06zgBQ',
  pathname: '/',
  path: '/?gfe_rd=cr&ei=LixKWJnOGtTC8geL06zgBQ',
  href: 'https://www.google.co.kr/?gfe_rd=cr&ei=LixKWJnOGtTC8geL06zgBQ#q=snowdeer+web+scraper' }

# 원본 param : gfe_rd=cr&ei=LixKWJnOGtTC8geL06zgBQ
# 현재 Param 정보
{ gfe_rd: 'cr', ei: 'LixKWJnOGtTC8geL06zgBQ' }
gfe_rd : cr
ei : LixKWJnOGtTC8geL06zgBQ
~~~
