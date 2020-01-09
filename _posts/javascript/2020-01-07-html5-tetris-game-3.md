---
layout: post
title: HTML5 Tetris 게임 만들기 - (3) 화면을 Grid 형태로 구성하고, 격자 좌표계로 블럭 이동, 그리기
category: Javascript
tag: [javascript, html5]
---

## Grid 적용

## js/tetris.js

<pre class="prettyprint">
var canvas;
var ctx;

var x, y;

const BLOCK_DROP_DELAY = 1000;
var lastBlockDownTime = 0;

const FRAME = 60;
var drawingTimeDelay = 1000/FRAME;

const ROWS = 24;
const COLS = 12;
var grid;

var cellWidth;
var cellHeight;

function init() {
    console.log("init()");
    canvas = document.getElementById('canvas');
    ctx = canvas.getContext('2d');

    initGrid(COLS, ROWS);

    x = COLS / 2;
    y = 0;

    addKeyEventListener();
}

function initGrid(cols, rows) {
    console.log("initGrid(" + cols + ", " + rows + ")");

    cellWidth = canvas.width / COLS;
    cellHeight = canvas.height / ROWS;

    grid = new Array(rows);
    for(var i=0; i < rows; i++) {
        grid[i] = new Array(cols);
    }

    for(var r=0; r < rows; r++) {
        for(var c=0; c < cols; c++) {
            grid[r][c] = 0;
        }
    }
}

function start() {
   animate(-1);
}

function draw() {
    drawBackground();
    drawGridLine();
    drawBlock();
}

function drawBackground() {
    ctx.fillStyle = 'black';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
}

function drawGridLine() {
    ctx.strokeStyle = 'gray';

    for(var i=1; i < COLS; i++) {
        ctx.beginPath();
        ctx.moveTo(i * cellWidth, 0);
        ctx.lineTo(i * cellWidth, canvas.height);
        ctx.stroke();
    }

    for(var i=1; i < ROWS; i++) {
        ctx.beginPath();
        ctx.moveTo(0, i * cellHeight);
        ctx.lineTo(canvas.width, i * cellHeight);
        ctx.stroke();
    }
}

function drawBlock() {
    ctx.fillStyle = 'orange';

    var canvasX = x * cellWidth;
    var canvasY = y * cellHeight;

    ctx.fillRect(canvasX, canvasY, cellWidth, cellHeight);
}

function addKeyEventListener() {
    addEventListener('keydown', function(event) {
        switch(event.key) {
            case 'ArrowLeft':
                console.log("Left");
                x -=1;
                break;

            case 'ArrowRight':
                console.log("Right");
                x += 1;
                break;

            case 'ArrowUp':
                console.log("Up");
                y -= 1;
                break;

            case 'ArrowDown':
                console.log("Down");
                y += 1;
                break;
        }
    });
}

function animate(lastTime) {
    var curTime = (new Date()).getTime();
    var diff = curTime - lastTime;
    
    if(diff > drawingTimeDelay) {
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
        y += 1;
        lastBlockDownTime = curTime;
    }
 
}
</pre>