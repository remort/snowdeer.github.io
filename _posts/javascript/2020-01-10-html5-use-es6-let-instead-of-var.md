---
layout: post
title: HTML5 var 보다는 let을 사용해라
category: Javascript
tag: [javascript, html5]
---

# 변수 선언

ES5 까지는 변수 선언을 위해 `var` 키워드를 사용했었는데, ES6 부터 `const`와 `let`이 추가되었습니다.

`const`는 보이는대로 상수값 선언이기 때문에 깔끔합니다. 가장 많이 사용되어야 할 상수 선언 키워드입니다.

`var`의 경우 어휘적 유효 범위(Lexical scope)를 가지는데, `let`은 블럭 유효 범위(Block scope)를 가집니다. 해당 블럭까지만 유효한 변수입니다. 우리가 다른 언어에서 흔히 볼 수 있는 로컬 변수(Local variable)와 같다고 볼 수 있습니다.

<br>

## var 의 문제점

`var` 키워드를 사용하면 다음과 같은 버그가 발생할 수 있습니다.

<pre class="prettyprint">
function init() {
    var a = 1;

    if(true) {
        var a = 2;

        console.log("a: " + a);
    }

    console.log("a: " + a);
}
</pre>

위 코드의 실행 결과는 다음과 같습니다.

<pre class="prettyprint">
2
2
</pre>

블럭 안에서 `var a = 2;`를 선언했고, 블럭이 끝났음에도 이전 블럭에 있는 `a` 변수 값을 바꿔버렸습니다. 이러한 특성은 잠재적 오류를 발생시키기 쉽습니다.

`let`으로 바꾸면 다음과 같습니다.

<pre class="prettyprint">
function init() {
    let a = 1;

    if(true) {
        let a = 2;

        console.log("a: " + a);
    }

    console.log("a: " + a);
}
</pre>

<pre class="prettyprint">
2
1
</pre>

<br>

또한 `const`와 `let`이 `var`에 비해 가지는 장점은 또 있습니다.
`const`와 `let`은 같은 이름의 변수를 다시 선언할 수 없습니다. `var`은 같은 Scope 내에서 같은 이름의 변수를 여러 번 선언할 수 있습니다. 

따라서 `const`와 `let`은 미연에 실수를 방지해주기 때문에 `var` 보다는 `let`을 사용하는 것이 좋습니다.