---
layout: post
title: Vue.js 3.0 Router 사용하기
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js]
---
# Vue.js 3.0 Router 예제

Routing 기능은 특정 페이지로 이동하는 기능입니다.

<hr>

## 설치 방법

<pre class="prettyprint">
npm install vue-router@4

또는

yarn add vue-router@4
</pre>

## 간단한 예제

다음은 Vue Router를 설치하면 기본적으로 생성되는 예제입니다. &lt;router-link&gt; 태그 안에 Routing된 페이지가 출력됩니다.

Routing을 위해 &lt;a&gt; 태그를 이용해도 되지만, 이 경우 전체 페이지가 리로딩이 되는데, &lt;router-link&gt;를 이용할 경우 전체 페이지 리로딩 없이 &lt;router-link&gt; 안에 Routing된 페이지를 출력할 수 있습니다.

### App.vue

<pre class="prettyprint">
&lt;template&gt;
  &lt;nav&gt;
    &lt;router-link to="/"&gt;Home&lt;/router-link&gt;
    &lt;router-link to="/about"&gt;About&lt;/router-link&gt;
  &lt;/nav&gt;
  &lt;router-view /&gt;
&lt;/template&gt;

&lt;style&gt;
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
}

nav {
  padding: 30px;
}

nav a {
  font-weight: bold;
  color: #2c3e50;
}

nav a.router-link-exact-active {
  color: #42b983;
}
&lt;/style&gt;

</pre>

### router/index.js

Routing을 원하는 경로를 `router/index.js`에 추가하면 됩니다.

<pre class="prettyprint">
import { createRouter, createWebHistory } from "vue-router";
import HomeView from "../views/HomeView.vue";
import AboutView from "../views/AboutView.vue";

const routes = [
  {
    path: "/",
    name: "home",
    component: HomeView,
  },
  {
    path: "/about",
    name: "about",
    component: AboutView,
  },
];

const router = createRouter({
  history: createWebHistory(process.env.BASE_URL),
  routes,
});

export default router;
</pre>

### main.js

<pre class="prettyprint">
import { createApp } from "vue";
import App from "./App.vue";
import router from "./router";

createApp(App).use(router).mount("#app");
</pre>

<hr>

## Todo App의 Router 예제

간단한 Todo App을 만든다고 할 때 Router 기능 사용 예제입니다.

### router/index.js

<pre class="prettyprint">
import { createRouter, createWebHistory } from "vue-router";
import HomeView from "../views/HomeView.vue";
import TodoListView from "../views/TodoListView.vue";
import TodoView from "../views/TodoView.vue";
import AboutView from "../views/AboutView.vue";

const routes = [
  {
    path: "/",
    name: "home",
    component: HomeView,
  },
  {
    path: "/about",
    name: "about",
    component: AboutView,
  },
  {
    path: "todos",
    name: "todos",
    component: TodoListView,
  },
  {
    path: "todos/:id",
    name: "todo",
    component: TodoView,
  },
];

const router = createRouter({
  history: createWebHistory(process.env.BASE_URL),
  routes,
});

export default router;
</pre>

### TodoListView.vue

#### `router.push()` 함수를 이용한 Routing

<pre class="prettyprint">
...
&lt;script&gt;
  setup() {
    ...
    const router = useRouter();
    const moveToPage = (todoId) => {
      router.push(`/todos/${todoId}`);
    }; 
  }
&lt;/script&gt;
</pre>

#### Routing Name을 이용한 Routing

<pre class="prettyprint">
...
&lt;script&gt;
  setup() {
    ...
    const router = useRouter();
    const moveToPage = (todoId) => {
      router.push({
        name: "Todo",
        params: {
          id: todoId
        }
      });
    }; 
  }
&lt;/script&gt;
</pre>