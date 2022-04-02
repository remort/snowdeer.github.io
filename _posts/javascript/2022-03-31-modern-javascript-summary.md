---
layout: post
title: 모던 자바스크립트 ES6+ 기본 내용
category: Javascript
tag: [javascript, html5]
---

# 모던 자바스크립트 기본 내용

자바스크립트가 ES6+로 변경되면서 추가, 변경된 점들입니다.

<hr>

## 블럭함수

### 선언식

선언식은 기존에 자바스크립트에서 사용하던 방식입니다.

<pre class="prettyprint">
function hello() {
    alert("Hello");
}
</pre>

### 표현식

표현식은 함수를 변수에 할당하는 방식이며, 함수의 이름이 없습니다.
<pre class="prettyprint">
const add = function (a, b) {
    return a + b;
}
</pre>

#### 호이스팅

호이스팅(Hoisting)은 자바스크립트의 기능 중 하나로 코드 실행 전에 내부의 변수와 함수 선언부를
맨 위로 옮겨주는 기능입니다. 
다만, 함수 정의 방법인 '선언식'과 '표현식' 중 '선언식'에만 작동합니다.

즉, 아래와 같은 코드에서

<pre class="prettyprint">
hello();
function hello() {
    alert("Hello");
}

const c = add(3, 5);
const add = function (a, b) {
    return a + b;
}
</pre>

아래 쪽의 표현식으로 정의된 코드는 `add is not defined` 오류가 발생합니다. 즉, 순서를 생각해서 구현해야 합니다.

### 화살표 함수

화살표 함수는 ES6에서 새로 추가된 함수 정의 방식입니다. 

> 자바스크립트가 ES6으로 넘어오면서 가장 효과적으로 바뀐 문법이며, 모던 자바스크립트 프로그래밍에서 가장 많이 사용되는 방식입니다.

아래와 같은 형태로 표현할 수 있습니다.

<pre class="prettyprint">
const hello = () => {
    alert("Hello");
}

const add = (a, b) => {
    return a + b;
}
</pre>

<hr>

## 변수 선언 var, let, const

### var

`var`는 ES6 이전부터 지원하던 키워드이며, `let`, `const`는 ES6 이후에 추가된 키워드입니다.

`var` 키워드의 가장 큰 특징은 함수 스코프(Function  Scope) 지원입니다. 
중괄호로 표현되는 블럭 스코프(Block Scope)를 지원하지 않으며, 블럭 내부에서 블럭 외부의 변수 값을 변경하면
그대로 변경되는 특징이 있습니다. 또한 같은 이름으로 중복 선언해도 오류가 발생하지 않기 때문에 
프로그램이 커질 경우 블럭 내부에서 선언된 변수가 블럭 외부의 중요한 변수 값을 변경시킬 수 있는 위험이 있습니다.

### let, const

`let`과 `const`는 블럭 스코프를 지원합니다. 동일 스코프내에서 같은 이름의 변수를 선언할 수 없는 특징이 있습니다.
다만 블럭 스코프 외부의 전역에서는 사용할 수 없습니다.

- `let` : 변수 값을 계속 변경 가능
- `const` : 변수 값을 한 번 할당하면 바꿀 수 없음

`let`과 `const`를 사용하면 `var`은 더 이상 사용할 필요가 없으며, `ES6+` 버전부터는 `var`를 사용하지 않습니다.
단, 전역 변수가 필요한 경우에 예외적으로 `var`를 사용할 수 있습니다.

<hr>

## 모듈 export, import

모듈(Module)은 코드를 관리하는 가장 작은 단위입니다. 프로젝트내 많은 코드에서 공통적으로 호출해서 사용할 수 있었지만,
변수 이름이나 함수 이름 충돌이 자주 발생할 수 있는 문제가 있었고, ES6부터 모듈의 export, import 기능이 생겨 이 문제가 개선되었습니다.

### module export

모듈 Export는 이름으로 내보내는 방식과 `default`로 내보내는 방식 2가지가 있습니다.

#### 이름으로 내보내기

<pre class="prettyprint">
const message = "Hello";

const add = (a, b) => {
    return a + b;
}

export { message, add };
</pre>

#### 이름으로 가져오기

<pre class="prettyprint">
import { message, add } from './snow_library.js";

import * as snowLibrary from './snow_library.js";
</pre>

### default로 내보내기

<pre class="prettyprint">
const add = (a, b) => {
    return a + b;
}

export default add;
</pre>

기본으로 내보내는 경우 모듈당 하나의 함수나 클래스만 공유할 수 있습니다.

### default로 가져오기

`default`로 선언된 모듈을 가져올 때는 다음과 같이 이름을 변경할 수 있습니다.

<pre class="prettyprint">
import myAdd snowLibrary from './snow_library.js";
</pre>

<hr>

## JSON

### JSON.stringfy()

자바스크립트 객체를 텍스트로 변환합니다.

### JSON.parse()

텍스트를 자바스크립트 객체로 변환합니다.

<hr>

## Promise

`Promise`는 ES6부터 추가된 기능으로 기존 자바스크립트에서 비동기 처리를 위해서는 콜백(Callback) 함수를 이용했습니다.
콜백 함수로 대부분의 기능을 구현할 수 있었으나 다음과 같은 한계가 있었습니다.

* 콜백 지옥 문제 : 콜백의 중첩으로 코드 관리가 힘들어지는 경우가 발생
* 반환값 처리 문제 : 콜백 실행 시점과 종료 시점이 분리되어 반환값을 관리하기가 어려워짐

ES6에 추가된 `Promise`는 비동기 처리 방식으로 실행된 결과의 성공과 실패를 담는 객체입니다.
예제는 다음과 같습니다.

<pre class="prettyprint">
&lt;html&gt;
  &lt;body&gt;
    &lt;h1&gt;Hello, Promise !!&lt;/h1&gt;
    &lt;button onclick="testPromise()"&gt;Click&lt;/button&gt;
    &lt;script&gt;
      const func = () => {
        const resp = Math.random();
        console.log(`request() => ${resp}`);

        return resp > 0.5;
      };

      const testPromise = () => {
        console.log(`testPromise() is called.`);

        const request = new Promise((onSuccess, onFailed) => {
          const result = func();

          console.log(`result : ${result}`);

          if (result) {
            onSuccess();
          } else {
            onFailed();
          }
        });

        request.then(
          () => {
            console.log("Success !!");
          },
          () => {
            console.log("Failed !!!");
          }
        );
      };
    &lt;/script&gt;
  &lt;/body&gt;
&lt;/html&gt;

</pre>

<hr>

## await, async

`Promise`를 이용해서 비동기 처리를 훌륭하게 처리할 수 있었지만,
보다 유연하게 처리하기 위해서 `await`와 `async` 키워드가 ES8에 추가되었습니다.
`Promise`에 비해 코드가 간결해지고 가독성이 높아집니다.

### await

`await`는 비동기로 이루어지는 함수의 결과를 대기하는 키워드입니다.
위에서 구현한 'request()' 함수를 아래와 같이 대기할 수 있습니다.

<pre class="prettyprint">
const result = await request();
console.log(result);
</pre>

하지만 `await`만 사용할 경우, 요청한 결과값이 나오지 않을 경우 무한 대기에 빠지는 문제가 있어서
비동기 처리 함수는 반드시 `async`를 붙여야 합니다.

### async

일반 함수를 선언할 때 `async`를 붙여 비동기 방식으로 선언하는 키워드입니다.