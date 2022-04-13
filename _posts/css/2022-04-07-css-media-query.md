---
layout: post
title: CSS - Responsible Web - Media Query
category: 블로그
permalink: /css/:year/:month/:day/:title/

tag: [css]
---

# Media Query

편의상 Vue로 개발한 예제입니다.

## App.vue

화면에 보이는 상자는 화면 크기의 `90%`만큼 동적으로 변하는 반응성 상자입니다.

<pre class="prettyprint">
&lt;template&gt;
  &lt;div id="head"&gt;SnowDeer's Example&lt;/div&gt;
  &lt;div&gt;Hello. SnowDeer&lt;/div&gt;
&lt;/template&gt;

&lt;script&gt;
export default {
  name: "App",
};
&lt;/script&gt;

&lt;style&gt;
#head {
  width: 90%;
  height: 500px;
  margin: 0 auto;
  background: yellowgreen;
  border: 4px solid black;
  text-align: center;
}

@media all and (min-width: 500px) {
  body {
    background: teal;
  }
}

@media all and (min-width: 800px) {
  body {
    background: orange;
  }
}
&lt;/style&gt;
</pre>

## 실행 화면

웹 브라우저가 `500px` 이상이면 배경이 `teal` 색상으로 변경됩니다.

![Image](/assets/css/001.png)

웹 브라우저가 `800px` 이상이면 배경이 `orange` 색상으로 변경됩니다.

![Image](/assets/css/002.png)

<hr>

## Media Query 문법

Media Query 문법은 다음과 같습니다.

<pre class="prettyprint">
@media [only 또는 not] [미디어 유형] [and 또는 ,] (조건문) {실행문}
</pre>