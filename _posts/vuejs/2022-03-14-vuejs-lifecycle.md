---
layout: post
title: Vue.js Lifecycle
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js]
---
# Vue.js Lifecycle

## 라이프사이클 순서 

Vue의 라이프사이클은 아래와 같은 순서로 이루어집니다.

* beforeCreate
* created
* beforeMount
* mounted
* beforeUpdate
* updated
* beforeUnmount
* unmounted

### beforeCreate

인스턴스가 생성된 후 가장 처음 실행되는 단계입니다. `data` 및 `methods` 속성이 아직 인스턴스에 정의되지 않았으며,
DOM 요소에도 접근할 수 없습니다.

### created

`data` 및 `methods` 속성이 정의되었습니다. 하지만 인스턴스가 화면에 마운트되지 않았기 때문에 DOM 요소에는 접근할 수 없습니다.

### beforeMount

인스턴스가 DOM에 마운트되기 직전에 호출되는 단계입니다. `render()` 함수가 호출되기 직전의 로직을 넣기 적합합니다.

### mounted

DOM에 인스턴스가 마운트되고나서 호출되는 함수입니다.

## App.vue 예제 코드

<pre class="prettyprint">
&lt;template&gt;
  &lt;img alt="Vue logo" src="./assets/logo.png" /&gt;
&lt;/template&gt;

&lt;script&gt;
export default {
  name: "App",
  components: {},
  beforeCreate() {
    console.log("[snowdeer] beforeCreate()");
  },
  created() {
    console.log("[snowdeer] created()");
  },
  beforeMount() {
    console.log("[snowdeer] beforeMount()");
  },
  mounted() {
    console.log("[snowdeer] mounted()");
  },
  beforeUpdate() {
    console.log("[snowdeer] beforeUpdate()");
  },
  updated() {
    console.log("[snowdeer] updated()");
  },
  beforeUnmount() {
    console.log("[snowdeer] beforeUnmount()");
  },
  unmounted() {
    console.log("[snowdeer] unmounted()");
  },
};
&lt;/script&gt;

&lt;style&gt;
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 60px;
}
&lt;/style&gt;

</pre>

예제 코드를 실행하면 모든 로그가 출력되는 것이 아니라 `beforeCreate`, `created`, `beforeMount`, `mounted` 까지만
출력됩니다. 그 이유는 `updated`는 데이터 변경이 발생했을 때 화면 갱신을 위해 호출되는 로직이기 때문입니다.

따라서 아래와 같이 코드를 조금 수정해서 데이터를 변경하면 `updated` 까지 로그가 출력되는 것을 확인할 수 있습니다.

## 수정된 App.vue 예제 코드

<pre class="prettyprint">
&lt;template&gt;
  &lt;img alt="Vue logo" src="./assets/logo.png" /&gt;
  &lt;h1&gt;{{ message }}&lt;/h1&gt;
&lt;/template&gt;

&lt;script&gt;
export default {
  name: "App",
  components: {},
  data() {
    return {
      message: `hello, snowdeer`,
    };
  },
  beforeCreate() {
    console.log("[snowdeer] beforeCreate()");
  },
  created() {
    console.log("[snowdeer] created()");
  },
  beforeMount() {
    console.log("[snowdeer] beforeMount()");
  },
  mounted() {
    console.log("[snowdeer] mounted()");
    this.message = `hello, snowdeer +_+`;
  },
  beforeUpdate() {
    console.log("[snowdeer] beforeUpdate()");
  },
  updated() {
    console.log("[snowdeer] updated()");
  },
  beforeUnmount() {
    console.log("[snowdeer] beforeUnmount()");
  },
  unmounted() {
    console.log("[snowdeer] unmounted()");
  },
};
&lt;/script&gt;

&lt;style&gt;
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 60px;
}
&lt;/style&gt;
</pre>
