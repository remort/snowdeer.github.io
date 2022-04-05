---
layout: post
title: Vue.js 3.0 Event 상위로 전달하는 방법(Emit)
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js]
---
# Emit 사용 방법

`emit`는 컴포넌트에서 발생한 이벤트를 상위 부모에게 전달하는 키워드입니다.
다음과 같이 `script` 코드에서 이벤트를 발생하는 방법과 `template` 코드에서 이벤트를 발생시키는 2가지 방법이 있습니다.

## emit 예제

<pre class="prettyprint">
&lt;template&gt;
  &lt;h2&gt;Child Component&lt;/h2&gt;

  &lt;button @click="onClicked"&gt;Emit from script&lt;/button&gt;
  &lt;button @click="$emit('event2', 2)"&gt;Emit from template&lt;/button&gt;
&lt;/template&gt;

&lt;script&gt;
export default {
  emits: ["event1", "event2"],
  setup(props, context) {
    const { emit } = context;
    const onClicked = () => {
      emit("event1", 10);
    };

    return {
      onClicked,
    };
  },
};
&lt;/script&gt;
</pre>

## getCurrentInstance 사용

`emit`는 `getCurrentInstance()`를 이용해서도 획득할 수 있었습니다. 기존에는 `useContext`를 이용해서 획득할 수 있었는데, Vue 3.2.0 버전부터 `useContext`는 `deprecated` 되었습니다.

<pre class="prettyprint">
import { getCurrentInstance } from "vue";

export default {
  emits: ["event1", "event2"],
  setup() {
    const { emit } = getCurrentInstance();
    const onClicked = () => {
      emit("event1", 10);
    };

    return {
      onClicked,
    };
  },
};
</pre>