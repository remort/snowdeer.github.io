---
layout: post
title: Vue.js 3에서 Vuetify 사용하기
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js]
---
# Vuetify

현재 `Vuetify`는 Vue 3를 지원하지 않습니다. [공식 페이지](https://vuetifyjs.com/en/introduction/roadmap/#in-development)에서는 
다음과 같이 2022년 5월 Vue 3를 지원하는 3.0이 릴리즈된다고 합니다.

```
The current version of Vuetify does not support Vue 3. Support for Vue 3 will come with the release of Vuetify v3. When creating a new project, please ensure you selected Vue 2 from the Vue CLI prompts, or that you are installing to an existing Vue 2 project.
```

따라서 현재는 Vue 3에서는 Vuetify 3 beta 버전을 이용해야 합니다. 물론 테스트 목적으로만 사용하고 공식적인 프로그램에서는 사용하지 말라고 경고하고 있습니다.

Vuetify 3 beta 버전 페이지는 [여기](https://next.vuetifyjs.com/en/getting-started/installation/)입니다.

<hr>

## 설치 방법

`Vuetify`를 설치하기 위해서는 아래 명령어를 이용합니다.

<pre class="prettyprint">
vue add vuetify
</pre>

설치되고 나며, 자동으로 `package.json`와 `main.js`에 `Vuetify` 관련 코드가 삽입됩니다.

### main.js

<pre class="prettyprint">
import { createApp } from 'vue'
import App from './App.vue'
import vuetify from './plugins/vuetify'
import { loadFonts } from './plugins/webfontloader'

loadFonts()

createApp(App)
  .use(vuetify)
  .mount('#app')
</pre>

<hr>

## 확인

확인을 위해 [여기](https://next.vuetifyjs.com/en/features/application-layout/)에서 간단한 예제를
`App.vue`에 삽입해보고 `Vuetify`가 잘 동작하는지 확인해봅니다.

### App.vue

<pre class="prettyprint">
&lt;template&gt;
  &lt;v-app ref="app"&gt;
    &lt;v-app-bar color="grey-lighten-2" name="app-bar" class="justify-center"&gt;
      &lt;div class="d-flex justify-center align-center w-100"&gt;
        &lt;v-btn @click="print('app-bar')"&gt;Get data&lt;/v-btn&gt;
      &lt;/div&gt;
    &lt;/v-app-bar&gt;
    &lt;v-navigation-drawer color="grey-darken-2" permanent name="drawer"&gt;
      &lt;div class="d-flex justify-center align-center h-100"&gt;
        &lt;v-btn @click="print('drawer')"&gt;Get data&lt;/v-btn&gt;
      &lt;/div&gt;
    &lt;/v-navigation-drawer&gt;
    &lt;v-main&gt;
      &lt;v-card height="200px"&gt;&lt;/v-card&gt;
    &lt;/v-main&gt;
  &lt;/v-app&gt;
&lt;/template>

&lt;script&gt;
export default {
  name: "App",
  data: () => ({
    layout: null,
  }),
  methods: {
    print(key) {
      alert(JSON.stringify(this.$refs.app.getLayoutItem(key), null, 2));
    },
  },
};
&lt;/script&gt;
</pre>

#### 실행화면

![image](/assets/vue/001.png)