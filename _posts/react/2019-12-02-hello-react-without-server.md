---
layout: post
title: 별도 Front-end 서버없이 구동하는 React Hello World
category: React
permalink: /openshift/:year/:month/:day/:title/

tag: [React]
---

## Hello.html

<pre class="prettyprint">
&lt;head>
    &lt;script src="https://fb.me/react-0.14.3.js">&lt;/script>
    &lt;script src="https://fb.me/react-dom-0.14.3.js">&lt;/script>
&lt;/head>

&lt;body>
&lt;div id="content">ccc&lt;/div>
&lt;script>
    var h1=React.createElement('h1', null, 'Hello, snowdeer')
    ReactDOM.render(
        h1,
        document.getElementById('content')
    )
&lt;/script>

&lt;/body>
</pre>
