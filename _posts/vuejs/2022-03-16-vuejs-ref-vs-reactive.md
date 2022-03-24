---
layout: post
title: Vue.js 3 ref vs reactive
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js]
---
# ref vs reactive

`ref`와 `reactive`의 사용법 예제입니다.

Vue에서 변수들의 값이 변경되었을 때 화면이 자동으로 갱신되지 않습니다. 데이터 변경에 반응형으로 자동 갱신을 하기 위해서는 `ref` 또는 `reactive`를 사용해야 합니다.

`ref` 값을 변경할 때는 `변수명.value` 값을 변경하면 되며, `reactive`는 일반 구조체처럼 값을 변경하면 됩니다.

<pre class="prettyprint">
&lt;template&gt;
  &lt;h1&gt;ref vs reactive&lt;/h1&gt;
  &lt;div&gt;
    &lt;h2&gt;ref: {{ refData }}&lt;/h2&gt;
    &lt;h2&gt;reactive: {{ reactiveData.name }}, {{ reactiveData.address }}&lt;/h2&gt;
  &lt;/div&gt;
  &lt;button @click="update"&gt;Update&lt;/button&gt;
&lt;/template&gt;

&lt;script&gt;
import { reactive, ref } from "@vue/reactivity";
export default {
  setup() {
    const refData = ref("snowdeer");
    const reactiveData = reactive({
      name: "snowdeer",
      address: "Seoul",
    });

    const update = () => {
      refData.value = "snowdeer-ref";
      reactiveData.name = "snowdeer-reactive";
    };

    return {
      refData,
      reactiveData,
      update,
    };
  },
};
&lt;/script&gt;

&lt;style&gt;
&lt;/style&gt;

</pre>

## ref, reactive 차이

* `ref`는 `변수명.value`로 값을 변경하며, `reactive`는 데이터 구조체처럼 값을 변경하면 됩니다.
* `reactive`는 `primitive` 값에 대해서는 반응형을 갖지 않습니다. 다만 구조체 형태로 선언하면 반응형으로 동작합니다.
