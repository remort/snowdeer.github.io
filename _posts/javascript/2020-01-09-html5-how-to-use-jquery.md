---
layout: post
title: HTML5 jQuery 사용하기
category: Javascript
tag: [javascript, html5]
---

# jQuery 사용하는 방법

자바스크립트를 사용하다보면, `jQuery`를 조금만 활용하면 아주 간편하게 구현이 되는 경우가 많아서 어쩔 수 없이 사용하는 경우가 많습니다.

jQuery 사용법은 [w3schools](https://www.w3schools.com/jquery/jquery_get_started.asp)과 같은 사이트에서 잘 설명되어 있습니다.

jQuery를 사용하는 방법은 크게 2가지 방법이 있습니다.

* 직접 파일을 다운받아 사용하는 방법
* CDN을 이용하는 방법

두 방법 모두 장단 점이 있습니다.

<br>

## 직접 다운받아 사용하는 방법

[jquery.com](https://jquery.com/download/)에서 파일을 직접 받은 다음 다음 코드로 불러올 수 있습니다.

<pre class="prettyprint">
&lt;head>
&lt;script src="jquery-3.4.1.min.js">&lt;/script>
&lt;/head>
</pre>

<br>

## CDN을 이용하는 방법

CDN(Content Delivery Network)을 이용하면 파일을 다운로드 할 필요 없이  다음 호출 코드만으로 jQuery를 사용할 수 있어서 편리합니다.

<pre class="prettyprint">
&lt;head>
&lt;script src="https://ajax.googleapis.com/ajax/libs/jquery/3.4.1/jquery.min.js">&lt;/script>
&lt;/head>
</pre>

또는

<pre class="prettyprint">
&lt;head>
&lt;script src="https://ajax.aspnetcdn.com/ajax/jQuery/jquery-3.4.1.min.js">&lt;/script>
&lt;/head>
</pre>