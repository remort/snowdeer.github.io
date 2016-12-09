---
layout: post
title: Node.js Event 처리
category: Node.js
tag: [Node.js, event]
---

Node.js에서 이벤트는 EventEmitter를 통해서 전달할 수 있습니다.
EventEmitter를 상속받은 후, on()과 once(), emit() 메소드를 사용할 수 있습니다.

간단하게 다음과 같은 코드를 이용해서 테스트해보도록 하겠습니다.

먼저, 이벤트를 받을 객체입니다. "MyEventEmitter.js" 파일로 생성했습니다.

<pre class="prettyprint" style="font-size:0.7em;">
var util = require("util");
var EventEmitter = require("events").EventEmitter;

var MyEventEmitter = function () {
    this.on("hello", function () {
       console.log("MyEventEmitter : hello !!");
    });
}

util.inherits(MyEventEmitter, EventEmitter);

module.exports = MyEventEmitter;
</pre>


그리고 이벤트를 전송할 코드를 만들어보겠습니다. 저는 "EventEmitterCaller.js"로 만들었습니다.

<pre class="prettyprint" style="font-size:0.7em;">
var MyEventEmitter = require("./MyEventEmitter");

var myEventEmitter = new MyEventEmitter();
myEventEmitter.emit("hello");

console.log("MyEventEmitter에 'hello' 이벤트를 전달했습니다.");
</pre>

<br>

실행결과는 다음과 같습니다.

<pre class="prettyprint" style="font-size:0.7em;">
MyEventEmitter : hello !!
MyEventEmitter에 'hello' 이벤트를 전달했습니다.
</pre>