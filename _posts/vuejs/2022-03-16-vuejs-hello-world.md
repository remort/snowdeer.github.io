---
layout: post
title: Vue.js 3 Hello World
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js]
---
# Hello World

Vue.js 3에서 도입된 Composition API를 이용해서 Hello World를 출력하는 예제입니다.

<pre class="prettyprint">
&lt;template&gt;
  &lt;h1&gt;{{ greeting() }}, {{ name }}&lt;/h1&gt;
&lt;/template&gt;

&lt;script&gt;
export default {
  setup() {
    const name = "snowdeer";

    const greeting = () => {
      return "Hello";
    };

    return {
      name,
      greeting,
    };
  },
};
&lt;/script&gt;

&lt;style&gt;
&lt;/style&gt;

</pre>

위의 예제에서는 `name`이라는 변수, `greeting` 이라는 메소드를 작성한 예제입니다. `export default` 내에 `setup` 함수 안에 변수, 메소드 등을 선언할 수 있으며, `setup`의 리턴 값으로 `html`에서 사용할 변수 또는 메소드를 반환해주면 됩니다.

