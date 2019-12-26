---
layout: post
title: 별도 Front-end 서버없이 구동하는 React Hello World
category: react
permalink: /openshift/:year/:month/:day/:title/

tag: [react]
---

## Hello.html

{%raw%}
<pre class="prettyprint">
<head>
    <script src="https://fb.me/react-0.14.3.js"></script>
    <script src="https://fb.me/react-dom-0.14.3.js"></script>
</head>

<body>
<div id="content">ccc</div>
<script>
    var h1=React.createElement('h1', null, 'Hello, snowdeer')
    ReactDOM.render(
        h1,
        document.getElementById('content')
    )
</script>

</body>
</pre>
{%endraw%}
