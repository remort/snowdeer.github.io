---
layout: post
title: Vue.js 3.0 Responsible Navigation Drawer 예제
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js, css]
---
# 화면 크기에 따라 동작이 바뀌는 Navigation Drawer

화면의 너비(width)에 따라 슬라이딩 및 오버레이 동작으로 작동하는 Navigation Drawer 입니다.
브라우저의 폭이 넓으면 슬라이딩(Sliding, Push) 방식으로 동작하며, 폭이 좁은 경우에는 오버레이(Overlay, OffCanvas)로
동작합니다.

BootStrap과 scss를 사용하지만, 실제 동작은 순수 Vue와 css로 동작합니다. 화면 크기는 Media Query를 이용해서 확인합니다.

## 필요 패키지 설치

<pre class="prettyprint">
npm i bootstrap --save
npm i bootstrap-icons --save
npm i sass --save
npm i sass-loader --save
</pre>

<hr>

## 소스 코드들

### main.js

<pre class="prettyprint">
import { createApp } from "vue";
import App from "./App.vue";
import bootstrap from "bootstrap";
import "bootstrap-icons/font/bootstrap-icons.css";

createApp(App).use(bootstrap).mount("#app");
</pre>

### App.vue

<pre class="prettyprint">
&lt;template&gt;
  &lt;div id="app"&gt;
    &lt;NavigationDrawer
      id="drawer"
      :class="isDrawerOpened ? 'opened' : 'closed'"
      @menu-selected="drawerMenuSelected"
    /&gt;
    &lt;div
      id="drawer-dismiss-panel"
      v-if="isDrawerOpened"
      @click="closeDrawer"
    &gt;&lt;/div&gt;
    &lt;div id="main" :class="{ 'drawer-opened': isDrawerOpened }"&gt;
      &lt;AppBar id="appbar" @toggle-drawer="onToggleDrawer" /&gt;

      &lt;div id="content"&gt;This is a navigation drawer sample.&lt;/div&gt;
    &lt;/div&gt;
  &lt;/div&gt;
&lt;/template&gt;

&lt;script setup&gt;
import NavigationDrawer from "@/components/NavigationDrawer.vue";
import AppBar from "@/components/AppBar.vue";
import { ref } from "@vue/reactivity";

const isDrawerOpened = ref(false);

const onToggleDrawer = () => {
  isDrawerOpened.value = !isDrawerOpened.value;
};

const closeDrawer = () => {
  isDrawerOpened.value = false;
};

const drawerMenuSelected = (menu) => {
  console.log(`Menu(${menu}) is selected !!`);
};
&lt;/script&gt;

&lt;style lang="scss" scoped&gt;
$primary: #009688;
$secondary: #4db6ac;
$dark: #004d40;
$accent: #64ffda;
$translucent: rgba(0, 0, 0, 0.7);

#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;

  display: flex;
  flex-direction: row;
  background: black;
  color: white;

  width: 100vw;
  height: 100vh;
}

#drawer {
  position: absolute;
  z-index: 10;
  width: 180px;
  margin-left: -180px;
  height: 100vh;
  overflow: hidden;
  transition: 0.3s;
}
#drawer.opened {
  margin-left: 0px;
}
#drawer.closed {
  margin-left: -180px;
}
#drawer-dismiss-panel {
  display: none;
}

#main {
  position: absolute;
  z-index: 1;
  width: 100vw;
  height: 100vh;
  transition: 0.3s;
}
#main.drawer-opened {
  margin-left: 180px;
  width: calc(100% - 180px);
}
#main.drawer-closed {
  margin-left: 0px;
  width: 100%;
}

#appbar {
  margin: 0;
  background: $primary;
}

#content {
  margin: 0;
  color: black;
  background-color: whitesmoke;
  width: 100%;
  height: 100%;
}

@media screen and (max-width: 480px) {
  #drawer {
    position: fixed;
  }
  #drawer-dismiss-panel {
    display: inline-block;
    position: fix;
    z-index: 5;
    width: 100vw;
    height: 100vh;
    background-color: $translucent;
  }
  #main {
    display: block;
    position: absolute;
  }
  #main.drawer-opened {
    margin-left: 0px;
    width: 100%;
  }
}
&lt;/style&gt;
</pre>

### components/AppBar.vue

<pre class="prettyprint">
&lt;template&gt;
  &lt;div id="appbar"&gt;
    &lt;div
      id="hamburg-menu"
      class="align-horizontal-center align-vertical-center"
      @click="toggleDrawer"
    &gt;
      &lt;i class="bi bi-list"&gt;&lt;/i&gt;
    &lt;/div&gt;
    &lt;div id="title" class="align-vertical-center"&gt;AppBar Title&lt;/div&gt;
    &lt;div id="action-items"&gt;
      &lt;i class="action-item bi bi-heart"&gt;&lt;/i&gt;
      &lt;i class="action-item bi bi-three-dots-vertical"&gt;&lt;/i&gt;
    &lt;/div&gt;
  &lt;/div&gt;
&lt;/template&gt;

&lt;script setup&gt;
import { defineEmits } from "vue";

const emit = defineEmits(["toggle-drawer"]);
const toggleDrawer = () => {
  emit("toggle-drawer");
};
&lt;/script&gt;

&lt;style lang="scss" scoped&gt;
$accent: #64ffda;

.align-horizontal-center {
  display: flex;
  flex-direction: row;
  justify-content: center;
}

.align-vertical-center {
  display: flex;
  flex-direction: row;
  align-items: center;
}

#appbar {
  display: flex;
  flex-direction: row;
  height: 48px;
}

#hamburg-menu {
  width: 48px;
  height: 48px;
}
#hamburg-menu:hover {
  color: black;
  background: $accent;
}

#title {
  flex: 1;
  height: 48px;
}

#action-items {
  display: flex;
  flex-direction: row;
  height: 48px;
}

.action-item {
  width: 48px;
  height: 48px;
  display: flex;
  justify-content: center;
  align-items: center;
}
.action-item:hover {
  color: black;
  background: $accent;
}
&lt;/style&gt;
</pre>

### components/NavigationDrawer.vue

<pre class="prettyprint">
&lt;template&gt;
  &lt;div id="drawer"&gt;
    &lt;div id="header"&gt;SnowDeer Drawer&lt;/div&gt;
    &lt;div id="content"&gt;
      &lt;div class="menu-item"&gt;
        &lt;i class="bi bi-house"&gt;&lt;/i&gt;
        &lt;span&gt;Home&lt;/span&gt;
      &lt;/div&gt;
      &lt;div class="menu-item"&gt;
        &lt;i class="bi bi-gift"&gt;&lt;/i&gt;
        &lt;span&gt;Hello&lt;/span&gt;
      &lt;/div&gt;
      &lt;div class="menu-item"&gt;
        &lt;i class="bi bi-info-square"&gt;&lt;/i&gt;
        &lt;span&gt;About&lt;/span&gt;
      &lt;/div&gt;
    &lt;/div&gt;
  &lt;/div&gt;
&lt;/template&gt;

&lt;style lang="scss" scoped&gt;
$primary: #009688;
$secondary: #4db6ac;
$dark: #004d40;
$accent: #64ffda;
$translucent: rgba(0, 0, 0, 0.7);

#header {
  padding-top: 20px;
  padding-left: 12px;
  height: 64px;
  font-size: large;
  font-weight: bold;
  color: $accent;
  background: $dark;
  white-space: nowrap;
}

#content {
  background: $secondary;
  height: 100vh;
}

.menu-item {
  padding: 16px;
  cursor: pointer;
}

.menu-item:hover {
  color: black;
  background: turquoise;
}
&lt;/style&gt;
</pre>