---
layout: post
title: HTML5 Tetris 게임 만들기 - (1) 뼈대 만들기
category: Javascript
tag: [javascript, html5]
---

## Tetris 게임 뼈대 구현

테트리스 게임을 만들기 전에 먼저 HTML5 canvas 위에 작은 박스를 그려보고,
Keyboard 화살표 입력을 이용해서 이동을 할 수 있는 코드를 작성해보도록 하겠습니다.

<br>

## tetris.html

<pre class="prettyprint">
&lt;!DOCTYPE html>
&lt;html lang="en">
&lt;head>
    &lt;meta charset="UTF-8">
    &lt;title>;Tetris&lt;/title>

    &lt;script src="/js/tetris.js">&lt;/script>
&lt;/head>
&lt;body>
    &lt;canvas id="canvas" width="240" height="480">
        이 브라우저는 HTML5 Canvas를 지원하지 않습니다.
    &lt;/canvas>

    &lt;br>

    &lt;script>
        window.onload = function() {
            init();

            draw();
        }
    &lt;/script>
&lt;/body>
&lt;/html>
</pre>

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
</pre>

여기까지 코드를 작성하면 화면에 캔버스를 그리고 검정색 백그라운드에 오렌지색 박스를 하나 그리게 됩니다.

<br>

### 키보드 입력 이벤트 리스너 등록

<pre class="prettyprint">
function addKeyEventListener() {
    addEventListener('keydown', function(event) {
        switch(event.key) {
            case 'ArrowLeft':
                console.log("Left");
                x -=20;

                draw();
                break;

            case 'ArrowRight':
                console.log("Right");
                x += 20;

                draw();
                break;

            case 'ArrowUp':
                console.log("Up");
                y -= 20;

                draw();
                break;

            case 'ArrowDown':
                console.log("Down");
                y += 20;

                draw();
                break;
        }
    });
}
</pre>