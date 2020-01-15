---
layout: post
title: HTML5 Tetris 게임 만들기 - (2) requestAnimationFrame 메소드 이용해서 주기적 화면 갱신하기
category: Javascript
tag: [javascript, html5]
---

## requestAnimationFrame

### requestAnimationFrame과 setInterval 메소드의 차이

`setInterval()` 메소드를 사용하게 되면 타이머를 이용해서 화면의 갱신을 하게 되는데, 타이밍이 어긋나거나 프레임 손실이 발생해서 애니메이션이 끊어지는 현상이 발생할 수 있습니다. 또한, 시스템 리소스를 많이 사용하기 때문에 성능 저하나 배터리 소모가 크게 발생할 수 있습니다. 시스템 사양이 좋지 않은 경우 애니메이션이 느려지기도 합니다.

`requestAnimationFrame()` 메소드는 브라우저가 화면을 업데이트하는 경우에만 콜백 함수를 호출합니다. 브라우저가 최소화 되거나 다른 탭이 선택되는 경우 콜백 함수를 호출하지 않습니다. 콜백 호출 주기는 브라우저가 관리하기 때문에 `setInterval()` 메소드처럼 계속 호출되지 않습니다. 

`setInterval()` 메소드와 `requestAnimationFrame()` 메소드는 사용법이 같기 때문에 쉽게 변환을 할 수 있습니다.

<br>

## js/tetris.js

<pre class="prettyprint">
var canvas;
var ctx;

var x, y;

function init() {
    canvas = document.getElementById('canvas');
    ctx = canvas.getContext('2d');

    x = canvas.width / 2;
    y = 20;

    addKeyEventListener();
}

function start() {
   animate(-1);
}

function draw() {
    drawBackground();
    drawBlock();
}

function drawBackground() {
    ctx.fillStyle = 'black';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
}


function drawBlock() {
    ctx.fillStyle = 'orange';
    ctx.fillRect(x-10, y-10, 20, 20);
}

function addKeyEventListener() {
    addEventListener('keydown', function(event) {
        switch(event.key) {
            case 'ArrowLeft':
                console.log("Left");
                x -=20;
                break;

            case 'ArrowRight':
                console.log("Right");
                x += 20;
                break;

            case 'ArrowUp':
                console.log("Up");
                y -= 20;
                break;

            case 'ArrowDown':
                console.log("Down");
                y += 20;
                break;
        }
    });
}

function animate(lastTime) {
    var time = (new Date()).getTime();
    console.log("time: " + time);

    draw();

    requestAnimationFrame(function() {
        animate(time);
    });
}
</pre>

`requestAnimationFrame()` 메소드를 사용하게 되면, 그 전에 Key Event에서 화면을 갱신하던 부분을 제거해줘야 합니다.
그래서 `addKeyEventListener()` 메소드 내에서 `draw()`를 호출하는 부분을 제거했으며, 
별도로 `animate()` 메소드를 추가했습니다. 변수로 들어오는 `lastTime`는 나중에 시간과 속도를 계산해서
이동한 거리를 계산하기 위한 항목입니다.

위 코드를 실행하게 되면 `animate()` 메소드가 계속해서 반복되면서 화면을 갱신하게 됩니다. 

만약 시스템 부하나 배터리 효율 등을 고려해서 1초마다 렌더링하고 싶으먄 다음과 같이 변경할 수 있습니다.

<pre class="prettyprint">
function animate(lastTime) {
    var curTime = (new Date()).getTime();
    var diff = curTime - lastTime;
    
    if(diff > 1000) {
        console.log("render - time diff: " + diff);
        draw();

        lastTime = curTime;
    }

    requestAnimationFrame(function() {
        animate(lastTime);
    });
}
</pre>

<br>

일반적으로 30프레임 또는 60프레임 렌더링을 많이 하기 때문에 다음처럼 코드를 작성할 수 있습니다.

<pre class="prettyprint">
var canvas;
var ctx;

var x, y;

const FRAME = 60;
var drawingTimeDelay = 1000/FRAME;

function init() {
    canvas = document.getElementById('canvas');
    ctx = canvas.getContext('2d');

    x = canvas.width / 2;
    y = 20;

    addKeyEventListener();
}

function start() {
   animate(-1);
}

function draw() {
    drawBackground();
    drawBlock();
}

function drawBackground() {
    ctx.fillStyle = 'black';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
}


function drawBlock() {
    ctx.fillStyle = 'orange';
    ctx.fillRect(x-10, y-10, 20, 20);
}

function addKeyEventListener() {
    addEventListener('keydown', function(event) {
        switch(event.key) {
            case 'ArrowLeft':
                console.log("Left");
                x -=20;
                break;

            case 'ArrowRight':
                console.log("Right");
                x += 20;
                break;

            case 'ArrowUp':
                console.log("Up");
                y -= 20;
                break;

            case 'ArrowDown':
                console.log("Down");
                y += 20;
                break;
        }
    });
}

function animate(lastTime) {
    var curTime = (new Date()).getTime();
    var diff = curTime - lastTime;
    
    if(diff > drawingTimeDelay) {
        console.log("render - time diff: " + diff);
        draw();

        lastTime = curTime;
    }

    requestAnimationFrame(function() {
        animate(lastTime);
    });
}
</pre>

<br>

## 1초마다 1칸씩 블럭이 떨어지도록 하기

<pre class="prettyprint">
const BLOCK_DROP_DELAY = 1000;
var lastBlockDownTime = 0;

function animate(lastTime) {
    var curTime = (new Date()).getTime();
    var diff = curTime - lastTime;
    
    if(diff > drawingTimeDelay) {
        console.log("render - time diff: " + diff);
        draw();

        lastTime = curTime;
    }

    handleBlockDown();

    requestAnimationFrame(function() {
        animate(lastTime);
    });
}

function handleBlockDown() {
    var curTime = (new Date()).getTime();
    var diff = curTime - lastBlockDownTime;

    if(diff > BLOCK_DROP_DELAY) {
        y += 20;
        lastBlockDownTime = curTime;
    }
</pre>