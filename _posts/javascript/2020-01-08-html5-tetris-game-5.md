---
layout: post
title: HTML5 Tetris 게임 만들기 - (5) 다양한 모양 블럭 추가하기
category: Javascript
tag: [javascript, html5]
---

## 다양한 모양 블럭 추가하기

다양한 블럭 모양을 추가하기 위해서는 블럭도 배열 형태로 바꿔줘야 합니다.

기존 코드에 다음과 같은 항목들을 추가했습니다.

<pre class="prettyprint">
const BLOCK_SHAPE = {
    shape1 : [[0, 0], [1, 0], [0, 1], [1, 1]],
};
var curShape = BLOCK_SHAPE.shape1;
</pre>

블럭의 중점은 `(0, 0)` 이며, 그 주위로 채워진 블럭의 정보를 `JSON` 형태 배열로 정의했습니다.

일단 조금 쉽게 접근하기 위해서 블럭은 2x2 사이즈의 정사각형으로 정했고,
기본 틀이 만들어지면 다양한 모양의 블럭을 더 추가하면 됩니다.

<br>

<pre class="prettyprint">
function drawFallingBlock() {
    ctx.fillStyle = 'orange';

    for(var i=0; i < curShape.length; i++) {
        var canvasX = (x + curShape[i][0]) * cellWidth;
        var canvasY = (y + curShape[i][1]) * cellHeight;

        ctx.fillRect(canvasX, canvasY, cellWidth, cellHeight);
    }
}
</pre>

기존의 `drawBlock()` 메소드 이름을 조금 더 명확하게 하기 위해서 `drawFallingBlock()`으로 수정했습니다.

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
</pre>

기존에 아래 방향으로 떨어질 수 있는지를 판단하던 `canBlockMoveToDown()` 메소드도 좀 더 범용적으로 쓸 수 있게 `canBlockMoveTo(toX, toY)` 형태로 수정했습니다. 

따라서 이를 호출하던 부분들도 전부 수정했습니다.

<br>

<pre class="prettyprint">
function addKeyEventListener() {
    addEventListener('keydown', function(event) {
        switch(event.key) {
            case 'ArrowLeft':
                console.log("Left");
                if(canBlockMoveTo(x - 1, y)) {
                    x -= 1;
                }
                break;

            case 'ArrowRight':
                console.log("Right");
                if(canBlockMoveTo(x + 1, y)) {
                    x += 1;
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

function handleBlockDown() {
    var curTime = (new Date()).getTime();
    var diff = curTime - lastBlockDownTime;

    if(diff > BLOCK_DROP_DELAY) {
        if(canBlockMoveTo(x, y + 1)) {
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
</pre>

<br>

## 블럭 모양 추가

이제 다양한 모양의 블럭들을 추가해봅니다.

<pre class="prettyprint">
const BLOCK_SHAPE = {
    shape1 : [[0, 0], [1, 0], [0, 1], [1, 1]],
    shape2 : [[0, -1], [0, 0], [1, 0], [0, 1]],
    shape3 : [[0, -1], [0, 0], [0, 1], [0, 2]],
    shape4 : [[0, -1], [0, 0], [0, 1], [1, 1]],
    shape5 : [[0, -1], [0, 0], [1, 0], [1, 1]],
};
var curShape = BLOCK_SHAPE.shape1;
</pre>

<br>

그리고 랜덤하게 블럭이 변경되도록 `initBlock()` 메소드도 수정합니다.

<pre class="prettyprint">
function initBlock() {
    x = COLS / 2;
    y = 0;

    const keys = Object.keys(BLOCK_SHAPE);
    const randomIdx = Math.floor(Math.random() * keys.length);
    curShape = BLOCK_SHAPE[keys[randomIdx]];
}
</pre>

<br>

## 전체 소스

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
    curShape = BLOCK_SHAPE[keys[randomIdx]];
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
                if(canBlockMoveTo(x - 1, y)) {
                    x -= 1;
                }
                break;

            case 'ArrowRight':
                console.log("Right");
                if(canBlockMoveTo(x + 1, y)) {
                    x += 1;
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
        if(canBlockMoveTo(x, y + 1)) {
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
</pre>