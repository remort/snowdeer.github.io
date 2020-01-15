---
layout: post
title: Kotlin Ktor과 React 같이 사용하기
category: Kotlin
tag: [Kotlin, ktor]
---

# React 다운로드

먼저 React를 사용하기 위해서 `react.js`와 `react-dom.js` 파일을 다운로드합니다.

[여기](https://react-cn.github.io/react/downloads.html)에서 다운로드 가능하며, 저는 `min` 버전으로 다운로드했습니다.

<br>

## react.html

<pre class="prettyprint">
&lt;html>
&lt;head>
    &lt;script src="/static/web/react/react.js">&lt;/script>
    &lt;script src="/static/web/react/react-dom.js">&lt;/script>
&lt;/head>

&lt;body>
&lt;div id="content">hello, react&lt;/div>
&lt;script type="module">
    import { init } from '/static/web/react_test.js';

    init();
&lt;/script>

&lt;/body>
&lt;/html>
</pre>

<br>

## react_test.js

<pre class="prettyprint">
export function init() {
    console.log("init()");

    var h1=React.createElement('h1', null, 'Hello, React')
    ReactDOM.render(
        h1,
        document.getElementById('content')
    )
}
</pre>