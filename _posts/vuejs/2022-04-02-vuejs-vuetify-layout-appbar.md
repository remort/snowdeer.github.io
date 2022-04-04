---
layout: post
title: Vuetify 기본 레이아웃(App Bar)
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js]
---
# Vuetify 기본 레이아웃

기본적으로 Material Design은 `App Bar`와 `Content`로 이루어집니다. 
여기에 `Navigation Drawer`가 추가되기도 합니다.

<hr>

## App Bar와 Content, Footer 레이아웃

먼저 App Bar와 Content로 이루어진 레이아웃입니다.

<pre class="prettyprint">
&lt;template&gt;
  &lt;v-app ref="app"&gt;
    &lt;v-app-bar&gt;&lt;/v-app-bar&gt;
    &lt;v-main&gt;&lt;/v-main&gt;
    &lt;v-footer&gt;&lt;/v-footer&gt;
  &lt;/v-app&gt;
&lt;/template&gt;
</pre>

## App Bar 꾸미기

이 중에서 App Bar는 다음과 같은 요소로 구성되어 있습니다.

![image](/assets/vue/002.png)

1. Container: `v-app-bar`로 생성되는 전체 영역 컨테이너
2. App Bar Icon: `v-app-bar-nav-icon`로 추가
3. Title: `v-app-bar-title`
4. Action Items: `v-btn icon`
5. Overflow menu: `v-btn icon`

<hr>

## 간단한 App Bar

App Bar 태그 안에 `v-btn icon` 태그를 추가하면 자동으로 오른 정렬되어 추가가 됩니다. `Overflow menu`도 버튼의 아이콘만 다른 동일한 버튼입니다.

<pre class="prettyprint">
&lt;template&gt;
  &lt;v-app ref="app"&gt;
    &lt;v-app-bar&gt;&lt;/v-app-bar&gt;
      &lt;v-app-bar-nav-icon&gt;&lt;/v-app-bar-nav-icon&gt;
      &lt;v-toolbar-title&gt;AppBar Title&lt;/v-toolbar-title&gt;

      &lt;v-btn icon&gt;
        &lt;v-icon&gt;mdi-heart&lt;/v-icon&gt;
      &lt;/v-btn&gt;
      &lt;v-btn icon&gt;
        &lt;v-icon&gt;mdi-dots-vertical&lt;/v-icon&gt;
      &lt;/v-btn&gt;
    &lt;v-main&gt;&lt;/v-main&gt;
    &lt;v-footer&gt;&lt;/v-footer&gt;
  &lt;/v-app&gt;
&lt;/template&gt;
</pre>

![image](/assets/vue/003.png)

## 참고 사이트

* https://next.vuetifyjs.com/en/components/app-bars/
