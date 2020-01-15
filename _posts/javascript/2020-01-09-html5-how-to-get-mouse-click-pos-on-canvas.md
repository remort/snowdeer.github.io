---
layout: post
title: HTML5 Canvas의 마우스 클릭 위치 얻기
category: Javascript
tag: [javascript, html5]
---

# Canvas의 마우스 클릭 위치 얻기

<pre class="prettyprint">
var canvas;
var ctx;

function init() {
    console.log("init()");
    canvas = document.getElementById('canvas');
    ctx = canvas.getContext('2d');

    drawCanvasBorder();

    canvas.addEventListener("click", function(event) {
        var rect = canvas.getBoundingClientRect();
        var x = event.clientX - rect.left;
        var y = event.clientY - rect.top;

        console.log("(" + x + ", " + y + ") is clicked.");
    });
}

function drawCanvasBorder() {
    ctx.strokeStyle = 'black';

    ctx.beginPath();
    ctx.rect(0, 0, canvas.width, canvas.height);
    ctx.stroke();
}
</pre>