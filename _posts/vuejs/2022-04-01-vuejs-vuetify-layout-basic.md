---
layout: post
title: Vuetify 기본 레이아웃(전체 레이아웃)
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js]
---
# Vuetify 전체 레이아웃

기본적으로 Material Design의 레이아웃입니다.

![image](/assets/vue/004.png)

* `v-app-bar`
* `v-bottom-navigation`
* `v-footer`
* `v-navigation-drawer`
* `v-system-bar`
* `v-content`

<pre class="prettyprint">
&lt;template&gt;
  &lt;v-app ref="app"&gt;
    &lt;v-navigation-drawer&gt; &lt;/v-navigation-drawer&gt;

    &lt;v-app-bar&gt; &lt;/v-app-bar&gt;

    &lt;v-main&gt;
      &lt;v-container fluid&gt;
        &lt;router-view&gt;&lt;/router-view&gt;
      &lt;/v-container&gt;
    &lt;/v-main&gt;

    &lt;v-footer&gt; &lt;/v-footer&gt;
  &lt;/v-app&gt;
&lt;/template&gt;
</pre>

## 간단한 내용을 채운 예제

<pre class="prettyprint">
&lt;template&gt;
  &lt;v-app&gt;
    &lt;v-navigation-drawer&gt; &lt;/v-navigation-drawer&gt;

    &lt;v-app-bar color="primary"&gt;
      &lt;v-app-bar-nav-icon&gt;&lt;/v-app-bar-nav-icon&gt;
      &lt;v-toolbar-title&gt;SnowDeer's Basic Layout&lt;/v-toolbar-title&gt;
    &lt;/v-app-bar&gt;

    &lt;v-main class="pb-16"&gt;
      &lt;v-container fluid&gt;
        &lt;h2&gt;What is Lorem Ipsum?&lt;/h2&gt;
        &lt;div&gt;
          Lorem Ipsum is simply dummy text of the printing and typesetting
          industry. Lorem Ipsum has been the industry's standard dummy text ever
          since the 1500s, when an unknown printer took a galley of type and
          scrambled it to make a type specimen book. It has survived not only
          five centuries, but also the leap into electronic typesetting,
          remaining essentially unchanged. It was popularised in the 1960s with
          the release of Letraset sheets containing Lorem Ipsum passages, and
          more recently with desktop publishing software like Aldus PageMaker
          including versions of Lorem Ipsum.
        &lt;/div&gt;
        &lt;br /&gt;
        &lt;h2&gt;Where does it come from?&lt;/h2&gt;
        &lt;div&gt;
          Contrary to popular belief, Lorem Ipsum is not simply random text. It
          has roots in a piece of classical Latin literature from 45 BC, making
          it over 2000 years old. Richard McClintock, a Latin professor at
          Hampden-Sydney College in Virginia, looked up one of the more obscure
          Latin words, consectetur, from a Lorem Ipsum passage, and going
          through the cites of the word in classical literature, discovered the
          undoubtable source. Lorem Ipsum comes from sections 1.10.32 and
          1.10.33 of "de Finibus Bonorum et Malorum" (The Extremes of Good and
          Evil) by Cicero, written in 45 BC. This book is a treatise on the
          theory of ethics, very popular during the Renaissance. The first line
          of Lorem Ipsum, "Lorem ipsum dolor sit amet..", comes from a line in
          section 1.10.32.
        &lt;/div&gt;
        &lt;br /&gt;
        &lt;div&gt;
          The standard chunk of Lorem Ipsum used since the 1500s is reproduced
          below for those interested. Sections 1.10.32 and 1.10.33 from "de
          Finibus Bonorum et Malorum" by Cicero are also reproduced in their
          exact original form, accompanied by English versions from the 1914
          translation by H. Rackham.
        &lt;/div&gt;
        &lt;br /&gt;
        &lt;h2&gt;Why do we use it?&lt;/h2&gt;
        &lt;div&gt;
          It is a long established fact that a reader will be distracted by the
          readable content of a page when looking at its layout. The point of
          using Lorem Ipsum is that it has a more-or-less normal distribution of
          letters, as opposed to using 'Content here, content here', making it
          look like readable English. Many desktop publishing packages and web
          page editors now use Lorem Ipsum as their default model text, and a
          search for 'lorem ipsum' will uncover many web sites still in their
          infancy. Various versions have evolved over the years, sometimes by
          accident, sometimes on purpose (injected humour and the like).
        &lt;/div&gt;
      &lt;/v-container&gt;
    &lt;/v-main&gt;
    &lt;v-footer color="secondary" bottom fixed padless style="width: 100%"&gt;
      &lt;div class="mx-auto"&gt;Copyright &copy;&lt;/div&gt;
    &lt;/v-footer&gt;
  &lt;/v-app&gt;
&lt;/template&gt;
</pre>

## 참고 사이트

* https://next.vuetifyjs.com/en/components/application/
