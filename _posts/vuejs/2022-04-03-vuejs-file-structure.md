---
layout: post
title: Vue.js 3.0 파일 구조
category: Vue.js
permalink: /vuejs/:year/:month/:day/:title/

tag: [Vue.js]
---
# Vue.js 3.0 파일 구조

# Single-File Components

Vue는 SFC(Single-File Components)로 되어 있습니다. 하나의 파일에 JavaScripts, Template, Style 모두 저장되어 있어 관리가 용이합니다.


## vue 파일 구조

하나의 `vue` 파일의 구조는 다음과 같습니다. [Vue 공식 홈페이지](https://vuejs.org/guide/scaling-up/sfc.html#introduction)에서 더 자세히 확인할 수 있습니다

<pre class="prettyprint">
&lt;script&gt;
export default {
  data() {
    return {
      greeting: 'Hello World!'
    }
  }
}
&lt;/script&gt;

&lt;template&gt;
  &lt;p class="greeting"&gt;{{ greeting }}&lt;/p&gt;
&lt;/template&gt;

&lt;style&gt;
.greeting {
  color: red;
  font-weight: bold;
}
&lt;/style&gt;
</pre>

순서는 크게 상관은 없지만, Vue 공식 홈페이지 기준으로는 `script`, `template`, `style` 순으로 배치합니다. 

## vscode의 vue 자동 생성 템플릿 수정(Vetur 플러그인)

Vue 개발시 주로 `vscode`에 `Vetur` 플러그인을 많이 사용할텐데, 빈 문서에서 `vue`를 타이핑하고 엔터를 치면 Scaffolding 기능을 통해 다음과 같은 형태로 기본 문서가 만들어집니다.

![image](/assets/vue/005.png)

<pre class="prettyprint">
&lt;template&gt;
  
&lt;/template&gt;

&lt;script&gt;
export default {

}
&lt;/script&gt;

&lt;style&gt;

&lt;/style&gt;
</pre>

이 부분을 공식 Vue 가이드 문서와 같은 형태로 변경하려면 다음과 같은 방법으로 해야 합니다.

현재 개발하는 프로젝트의 루트 디렉토리에서 `.vscode/vetur/snippets` 디렉토리 아래 `vue3-default.vue` 파일을 생성하고 아래 내용을 채웁니다.

<pre class="prettyprint">
&lt;script setup&gt;

&lt;/script&gt;

&lt;template&gt;
  
&lt;/template&gt;

&lt;style scoped&gt;

&lt;/style&gt;
</pre>

그 다음 `vscode`를 재실행하면 새로 추가한 Scaffold를 선택해서 초기 코드를 생성할 수 있습니다.