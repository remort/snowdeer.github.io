---
layout: post
title: Vue.js 3.0 기본 여백(margin) 제거하기
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js, css]
---

# Vue 기본 여백(margin) 제거하기

Vue.js로 화면을 띄우면 기본적으로 상하좌우 `8px`의 여백이 있습니다.
아래의 코드를 이용해서 여백을 없앨 수 있습니다.

위치는 `App.vue`에 넣어도 되고, `public/index.html`에 넣어도 되는데 후자가 더 깔끔한 위치라고 생각합니다.

<pre class="prettyprint">
&lt;style&gt;
body {
  margin: 0px;
}
&lt;/style&gt;
</pre>
