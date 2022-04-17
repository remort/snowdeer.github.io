---
layout: post
title: Vue.js 3.0 기본 레이아웃 - (3) Navigation Drawer 레이아웃
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js, css]
---
# Navigation Drawer

<pre class="prettyprint">
&lt;template&gt;
  &lt;div id="wrapper"&gt;
    &lt;div id="drawer"&gt;
      &lt;div id="drawer-header"&gt;SnowDeer Drawer&lt;/div&gt;
      &lt;div id="drawer-content"&gt;
        &lt;div id="drawer-menu-item"&gt;
          &lt;i class="bi bi-house"&gt;&lt;/i&gt;
          &lt;span&gt;Home&lt;/span&gt;
        &lt;/div&gt;
        &lt;div id="drawer-menu-item"&gt;
          &lt;i class="bi bi-gift"&gt;&lt;/i&gt;
          &lt;span&gt;Hello&lt;/span&gt;
        &lt;/div&gt;
        &lt;div id="drawer-menu-item"&gt;
          &lt;i class="bi bi-info-square"&gt;&lt;/i&gt;
          &lt;span&gt;About&lt;/span&gt;
        &lt;/div&gt;
      &lt;/div&gt;
    &lt;/div&gt;
    &lt;div id="main"&gt;
      &lt;div id="appbar"&gt;
        &lt;div id="hamburg-menu" class="h-center v-center"&gt;
          &lt;i class="bi bi-list"&gt;&lt;/i&gt;
        &lt;/div&gt;
        &lt;div id="title" class="v-center"&gt;AppBar Title&lt;/div&gt;
        &lt;div id="action-items"&gt;
          &lt;i class="action-item bi bi-heart"&gt;&lt;/i&gt;
          &lt;i class="action-item bi bi-three-dots-vertical"&gt;&lt;/i&gt;
        &lt;/div&gt;
      &lt;/div&gt;
      &lt;div id="content"&gt;&lt;/div&gt;
    &lt;/div&gt;
  &lt;/div&gt;
&lt;/template&gt;

&lt;style&gt;
.h-center {
  display: flex;
  flex-direction: row;
  justify-content: center;
}

.v-center {
  display: flex;
  flex-direction: row;
  align-items: center;
}

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

#appbar {
  display: flex;
  flex-direction: row;
  height: 48px;
  background: #edb1f1;
}

#hamburg-menu {
  width: 48px;
  height: 48px;
  background: #d59bf6;
  cursor: pointer;
}

#title {
  flex: 1;
  height: 48px;
}

#action-items {
  display: flex;
  flex-direction: row;
  height: 48px;
  background: #d59bf6;
}

.action-item {
  width: 48px;
  height: 48px;
  display: flex;
  justify-content: center;
  align-items: center;
  cursor: pointer;
}

#drawer {
  display: flex;
  flex-direction: column;
  width: 200px;
  height: 100vh;
  background: darkcyan;
  color: white;
}

#drawer-header {
  width: 200px;
  height: 100px;
  padding: 20px;
  background: teal;
}

#drawer-content {
  flex: 1;
}

#drawer-menu-item {
  padding: 16px;
  cursor: pointer;
}

#drawer-menu-item:hover {
  color: black;
  background: turquoise;
}

#main {
  flex: 1;
  flex-flow: column wrap;
  justify-content: flex-start;
  background: #bbe4e9;
}
&lt;/style&gt;

</pre>

![image](/assets/vue/008.png)