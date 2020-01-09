---
layout: post
title: HTML5 Tetris 게임 만들기 - (6) 블럭 회전
category: Javascript
tag: [javascript, html5]
---

## 블럭 회전 기능 구현

블럭 회전을 위해서 다음 함수들을 추가했습니다.

<pre class="prettyprint">
function handleRotate() {
    var nextShape = new Array(curShape.length);

    for(var i=0; i < curShape.length; i++) {
        var x = curShape[i][0];
        var y = curShape[i][1];

        nextShape[i] = new Array(2);        
        nextShape[i][0] = y;
        nextShape[i][1] = -x;
    }

    if(canRotate(nextShape)) {
        curShape = nextShape;
    }
}

function canRotate(nextShape) {
    for(var i=0; i < nextShape.length; i++) {
        var shapeX = x + nextShape[i][0];
        var shapeY = y + nextShape[i][1];

        if(shapeX < 0) return false;
        if(shapeX >= COLS) return false;
        if(shapeY < 0) return false;
        if(shapeY >= ROWS) return false;
        
        if(grid[shapeY][shapeX] == 1) return false;
    }
    return true;
}
</pre>

블럭을 회전시키기 전에 미리 가상의 블럭을 생성해서 회전한 다음, 회전이 가능한지 판단합니다. 만약 회전이 가능하면, 회전한 가상 블럭을 진짜 블럭으로 교체하며, 회전이 안되는 경우는 이를 무시하도록 되어 있습니다.

<br>

<pre class="prettyprint">
function canBlockMoveTo(toX, toY) {
    for(var i=0; i < curShape.length; i++) {
        var shapeX = toX + curShape[i][0];
        var shapeY = toY + curShape[i][1];

        if(shapeX < 0) return false;
        if(shapeX >= COLS) return false;
        if(shapeY < 0) return false;
        if(shapeY >= ROWS) return false;
        
        if(grid[shapeY][shapeX] == 1) return false;
    }

    return true;
}

function canRotate(nextShape) {
    for(var i=0; i < nextShape.length; i++) {
        var shapeX = x + nextShape[i][0];
        var shapeY = y + nextShape[i][1];

        if(shapeX < 0) return false;
        if(shapeX >= COLS) return false;
        if(shapeY < 0) return false;
        if(shapeY >= ROWS) return false;
        
        if(grid[shapeY][shapeX] == 1) return false;
    }
    return true;
}
</pre>

`canRotate(nextShape)` 코드를 자세히 보면 `canBlockMoveTo(toX, toY)` 메소드와 내용이 거의 흡사합니다. 회전 역시 이동의 일종으로 생각할 수 있기 때문에 두 메소드는 아래와 같이 하나의 메소드로 합칠 수 있습니다.

<pre class="prettyprint">
function canBlockMoveTo(toX, toY, shape) {
    for(var i=0; i < shape.length; i++) {
        var shapeX = toX + shape[i][0];
        var shapeY = toY + shape[i][1];

        if(shapeX < 0) return false;
        if(shapeX >= COLS) return false;
        if(shapeY < 0) return false;
        if(shapeY >= ROWS) return false;
        
        if(grid[shapeY][shapeX] == 1) return false;
    }

    return true;
}
</pre>

<br>

## 전체 코드

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

const BLOCK_SHAPE = {
    shape1 : [[0, 0], [1, 0], [0, 1], [1, 1]],
    shape2 : [[0, -1], [0, 0], [1, 0], [0, 1]],
    shape3 : [[0, -1], [0, 0], [0, 1], [0, 2]],
    shape4 : [[0, -1], [0, 0], [0, 1], [1, 1]],
    shape5 : [[0, -1], [0, 0], [1, 0], [1, 1]],
};
var curShape = BLOCK_SHAPE.shape1;

//console.log("curShape size: " + curShape.length);

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

    const keys = Object.keys(BLOCK_SHAPE);
    const randomIdx = Math.floor(Math.random() * keys.length);
    curShape = BLOCK_SHAPE[keys[randomIdx]].slice();
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
    drawFallingBlock();
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

function drawFallingBlock() {
    ctx.fillStyle = 'orange';

    for(var i=0; i < curShape.length; i++) {
        var canvasX = (x + curShape[i][0]) * cellWidth;
        var canvasY = (y + curShape[i][1]) * cellHeight;

        ctx.fillRect(canvasX, canvasY, cellWidth, cellHeight);
    }
}

function addKeyEventListener() {
    addEventListener('keydown', function(event) {
        switch(event.key) {
            case 'ArrowLeft':
                console.log("Left");
                if(canBlockMoveTo(x - 1, y, curShape)) {
                    x -= 1;
                }
                break;

            case 'ArrowRight':
                console.log("Right");
                if(canBlockMoveTo(x + 1, y, curShape)) {
                    x += 1;
                }
            break;

            case 'ArrowUp':
                console.log("Up");
                handleRotate();
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
        if(canBlockMoveTo(x, y + 1, curShape)) {
            y += 1;    
        }
        else {
            console.log("block down stopped(x: " + x, "y: " + y + ")");

            for(var i=0; i < curShape.length; i++) {
                var shapeX = x + curShape[i][0];
                var shapeY = y + curShape[i][1];
                grid[shapeY][shapeX] = 1;
            }
            
            initBlock();
        }
        lastBlockDownTime = curTime;
    }
}

function canBlockMoveTo(toX, toY, shape) {
    for(var i=0; i < shape.length; i++) {
        var shapeX = toX + shape[i][0];
        var shapeY = toY + shape[i][1];

        if(shapeX < 0) return false;
        if(shapeX >= COLS) return false;
        if(shapeY < 0) return false;
        if(shapeY >= ROWS) return false;
        
        if(grid[shapeY][shapeX] == 1) return false;
    }

    return true;
}

function handleRotate() {
    var nextShape = new Array(curShape.length);

    for(var i=0; i < curShape.length; i++) {
        nextShape[i] = new Array(2);        
        nextShape[i][0] = curShape[i][1];
        nextShape[i][1] = -curShape[i][0];
    }

    if(canBlockMoveTo(x, y, nextShape)) {
        curShape = nextShape;
    }
}
</pre>