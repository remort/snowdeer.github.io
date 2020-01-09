---
layout: post
title: HTML5 Tetris 게임 만들기 - (4) 바닥에 도착한 블럭 쌓이게 하기
category: Javascript
tag: [javascript, html5]
---

## 바닥에 도착한 블럭 쌓이게 하기

## js/tetris.js

<pre class="prettyprint">
var canvas;
var ctx;

var x, y;

const BLOCK_DROP_DELAY = 100;
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
    initBlock();    

    addKeyEventListener();
}

function initBlock() {
    x = COLS / 2;
    y = 0;
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
    drawBlocks();
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

function drawBlocks() {
    for(var r=0; r < ROWS; r++) {
        for(var c=0; c < COLS; c++) {
            if(grid[r][c] == 1) {
                fillGrid(c, r);
            }
        }
    }
}

function fillGrid(x, y) {
    ctx.fillStyle = 'green';

    var canvasX = x * cellWidth;
    var canvasY = y * cellHeight;

    ctx.fillRect(canvasX, canvasY, cellWidth, cellHeight);
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
                if(x > 0) {
                    if(grid[y][x - 1] == 0) {
                        x -= 1;    
                    }    
                }
                break;

            case 'ArrowRight':
                console.log("Right");
                if(x < COLS - 1) {
                    if(grid[y][x + 1] == 0) {
                        x += 1;        
                    }
                }
                break;

            case 'ArrowUp':
                console.log("Up");
                break;

            case 'ArrowDown':
                console.log("Down");
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
        if(canBlockMoveToDown()) {
            y += 1;    
        }
        else {
            console.log("block down stopped(x: " + x, "y: " + y + ")");
            grid[y][x] = 1;

            initBlock();
        }
        lastBlockDownTime = curTime;
    }
}

function canBlockMoveToDown() {
    if(y >= (ROWS - 1)) return false;
    if(grid[y + 1][x] == 1) return false;

    return true;
}
</pre>