---
layout: post
title: HTML5 Canvas에 사각형 그리고 Drag & Drop 구현하기
category: Javascript
tag: [javascript, html5]
---

# Canvas에 사각형 그리고 Drag & Drop 구현하기

<pre class="prettyprint">
var canvas;
var ctx;

var screenRect;

var rectX, rectY;
var WIDTH = 100, HEIGHT = 50;
var prevMouseX, prevMouseY;
var isMouseDown = false;
var isRectSelected = false;

function init() {
    console.log("init()");
    canvas = document.getElementById('canvas');
    ctx = canvas.getContext('2d');

    screenRect = canvas.getBoundingClientRect();

    rectX = canvas.width / 2;
    rectY = canvas.height / 2;

    addKeyEventListener();

    animate();
}

function addKeyEventListener() {

    canvas.onmousedown = function(e) {
        var x = event.clientX - screenRect.left;
        var y = event.clientY - screenRect.top;

        console.log("MouseDown : (" + x + ", " + y + ")");

        prevMouseX = x;
        prevMouseY = y;

        isMouseDown = true;

        if(isRectClicked(x, y)) {
            isRectSelected = true;
        }
        else {
            isRectSelected = false;
        }
    }

    canvas.onmousemove = function(e) {
        if(isMouseDown && isRectSelected) {
            var x = event.clientX - screenRect.left;
            var y = event.clientY - screenRect.top;

            var dx = x - prevMouseX;
            var dy = y - prevMouseY;

            rectX += dx;
            rectY += dy;

            prevMouseX = x;
            prevMouseY = y;
        }     
    }

    canvas.onmouseup = function(e) {
        var x = event.clientX - screenRect.left;
        var y = event.clientY - screenRect.top;

        console.log("MouseUp : (" + x + ", " + y + ")");

        isMouseDown = false;
        isRectSelected = false;
    }

    canvas.onmouseout = function(e) {
        isMouseDown = false;
    }
}

function animate() {
    drawCanvasBackground();
    drawRect();

    requestAnimationFrame(function() {
        animate();
    });
}

function drawCanvasBackground() {
    ctx.fillStyle = 'white';
    ctx.strokeStyle = 'black';

    ctx.fillRect(0, 0, canvas.width, canvas.height);
    ctx.beginPath();
    ctx.rect(0, 0, canvas.width, canvas.height);
    ctx.stroke();
}

function drawRect() {
    ctx.strokeStyle = 'black';

    ctx.beginPath();
    ctx.rect(rectX - WIDTH / 2, rectY - HEIGHT / 2, WIDTH, HEIGHT);
    ctx.stroke();
}

function isRectClicked(x, y) {
    if ((x >= (rectX - WIDTH / 2)) && 
        (x <= (rectX + WIDTH / 2)) && 
        (y >= (rectY - HEIGHT / 2)) && 
        (y <= (rectY + HEIGHT / 2))) {
        return true;
    }

    return false;

}
</pre>