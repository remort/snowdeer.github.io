---
layout: post
title: HTML5 ES6 module (import/export) 사용하기
category: Javascript
tag: [javascript, html5]
---

# ES6 module

ES6(ECMAScript 6, ES2015)부터 `module`이라는 기능이 생겼습니다.

<pre class="prettyprint">
&lt;script type="module" src="hello.mjs">&lt;/script>
</pre>

기존에 자바스크립트를 불러오는 코드에 `type="module"` 옵션을 추가해주면 모듈을 불러올 수 있습니다. 쉽게 구분하고 사용할 수 있도록 `mjs` 확장자를 사용하는 것을 추천하고 있습니다.

<br>

## 모듈의 특징

모듈은 기본적으로 자바스크립트의 특징을 모두 갖고 있습니다. 그리고 추가적으로 다음과 같은 특징이 있습니다.

* `import`, `export`를 사용할 수 있음
* 모듈 바깥쪽에 선언한 변수들은 전역(Globas scope)가 아닌 Module scope로 선언됨
* 기본적으로 `Strict mode`로 동작
* 같은 모듈을 다른 모듈에서 여러 번 불러도, 모둘 내부 코드는 단 한 번만 실행됨

<br>

## 사용법

모듈 외부에서 사용할 수 있도록 공개된 변수나 함수 앞에 `export` 키워드를 붙이면 되고, 다른 모듈에서는 `import` 키워드를 이용해서 불러올 수 있습니다.

### export 사용 예제

<pre class="prettyprint">
export const arrs = [10, 20, 30, 40];

export function getName() {
    // ...
}
</pre>

또는 기본 문법으로 구현된 모듈 뒷 부분에 다음 라인을 추가하면 됩니다.

<pre class="prettyprint">
export { arrs, getName };
</pre>

아래 예제처럼 `alias`를 지정할 수도 있습니다.

<pre class="prettyprint">
export { arrs, getName as name };
</pre>

<br>

### import 사용 예제

<pre class="prettyprint">
import { arrs, getName } from './sample_module.js';

import arrs from './sample_module.js';

import getName as name from './sample_module.js';

import * as name from './sample_module.js';
</pre>

<br>

## 예제

### module.html

<pre class="prettyprint">
&lt;!DOCTYPE html>
&lt;html lang="en">
&lt;head>
    &lt;meta charset="UTF-8">
    &lt;title>Module Test&lt;/title>
&lt;/head>
&lt;body>
    Hello

    &lt;script type="module">
        import { init } from '/static/js/test.js';

        window.onload = function() {
            console.log("window.onload()");
            init();
        }
    &lt;/script>
&lt;/body>
&lt;/html>
</pre>

<br>

### sample_module.js

<pre class="prettyprint">
export const arrs = [10, 20, 30, 40];

const name = "snowdeer";

export function getName() {
    return name;
}
</pre>

<br>

### test.js

<pre class="prettyprint">
import { arrs, getName } from './sample_module.js';

console.log(arrs);
printName();

function printName() {
    console.log(getName())
}

export function init() {
    console.log("init()");
    printName();
}
</pre>
