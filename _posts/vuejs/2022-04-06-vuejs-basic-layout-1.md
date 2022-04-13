---
layout: post
title: Vue.js 3.0 기본 레이아웃 - (1) 스켈레톤 레이아웃
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js, css]
---
# 기본 레이아웃

<pre class="prettyprint">
&lt;template&gt;
  &lt;div id="wrapper"&gt;
    &lt;div id="drawer"&gt;&lt;/div&gt;
    &lt;div id="main"&gt;
      &lt;div id="appbar"&gt;&lt;/div&gt;
      &lt;div id="content"&gt;&lt;/div&gt;
    &lt;/div&gt;
  &lt;/div&gt;
&lt;/template&gt;

&lt;style&gt;
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
}

#wrapper {
  display: -webkit-flex;
  display: flex;
  flex-flow: row wrap;
  justify-content: flex-start;
  width: 100vw;
  height: 100vh;
  margin: 0 auto;
}

#drawer {
  width: 200px;
  height: 100vh;
  background: #5585b5;
}

#main {
  flex: 1;
  flex-flow: column wrap;
  justify-content: flex-start;
  background: #bbe4e9;
}

#appbar {
  height: 48px;
  background: #53a8b6;
}
&lt;/style&gt;
</pre>

![image](/assets/vue/006.png)