---
layout: post
title: HTML5 ES6 클래스 사용하기
category: Javascript
tag: [javascript, html5]
---

# ES6 클래스

ES6(ECMAScript 6, ES2015)에 클래스를 사용하는 구문이 추가되었습니다. 기존에도 클래스를 사용할 수는 있었지만, 다음과 같이 `function`을 이용하거나 클래스 표현식을 이용해서 사용했었습니다.

## 기존 클래스 선언 방법

<pre class="prettyprint">
function Person(id, name, email) {
    this.id = id;
    this.name = name;
    this.email = email;
}

Person.prototype.id = function() {
    return id;
}

Person.prototype.name = function() {
    return "[name: " + this.name + 
        ", email: " + this.email + "]";
}
</pre>

<br>

## ES6 에서의 클래스 선언 방법

<pre class="prettyprint">
class Person {
    constructor(id, name, email) {
        this.id = id;
        this.name = name;
        this.email = email;
    }

    id() {
        return id;
    }

    name() {
        return name;
    }

    info() {
        return "[name: " + this.name +
            ", email: " + this.email + "]";
    }
}
</pre>

훨씬 더 Java의 문법과 비슷해졌습니다. 커다란 중괄호로 묶이기 때문에 기존 클래스 선언보다 가독성도 더 좋아졌습니다.

<br>

## 함수 선언문과 클래스 선언문의 차이

* 클래스 선언문은 자바스크립트 엔진이 미리 읽어들이지 않음. 따라서 생성자를 사용하기 전에 클래스 선언문이 먼저 호출되어야 함
* 클래스 선언문은 한 번만 작성되어져야 함

<br>

## 클래스 Property 접근자 작성하기

<pre class="prettyprint">
class Person {
    constructor(name) {
        this._name = name;
    }

    get name() {
        return this._name;
    }

    set name(value) {
        this._name = value;
    }
}
</pre>

위에서 `name()` 메소드에 `get`, `set` 키워드를 사용했습니다. 
따라서 다음과 같이 사용할 수 있습니다.

<pre class="prettyprint">
const p = new Person("snowdeer", "Seo", "snowdeer0314@gmail.com");
p.name = "Won";
console.log(p.info());
</pre>

위에서 `get` 이나 `set` 키워드 함수 내부를 보면 `this._name` 변수를 사용하고 있는 것이 보입니다. 자바스크립트에서 게터나 세터와 같은 이름의 속성을 만들 수 없기 때문에 별도의 변수를 추가해서 이를 해결한 방법이며, 자바스크립트에서는 `private` 속성을 지원하지 않아서 `prefix`로 `_`을 추가해서 형식적으로 표현만 했습니다.

<br>

## static 메소드 작성하기

<pre class="prettyprint">
export default class Person {
    constructor(name) {
        this._name = name;;

        Person.count++;
    }

    get name() {
        return this._name;
    }

    set name(value) {
        this._name = value;
    }

    static count() {
        return Person.count;
    }
}

Person.count = 0;
</pre>

사용법은 다음과 같습니다.

<pre class="prettyprint">
function test() {
    const p1 = new Person("snowdeer", "Seo");
    console.log(Person.count);

    const p2 = new Person("ran", "Lee");
    const p3 = new Person("yang", "Yang");

    console.log(Person.count);
    console.log(p3.count);
}
</pre>

실행 결과는 다음과 같습니다.

<pre class="prettyprint">
1
3
undefined
</pre>

<br>

## 클래스 상속

<pre class="prettyprint">
class Person {
    constructor(name) {
        this._name = name;;
    }

    get name() {
        return this._name;
    }

    set name(value) {
        this._name = value;
    }
}

class Student extends Person {
    hello() {
        console.log("Hello, My name is " + super.name + ". ");
    }
}
</pre>