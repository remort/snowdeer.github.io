---
layout: post
title: HTML5 Tetris 게임 만들기 - (7) 블럭 고속 낙하
category: Javascript
tag: [javascript, html5]
---

## 블럭 고속 낙하 기능 구현

키보드 아래 버튼을 누르면 블럭이 바로 아래로 떨어지도록 해봅니다.
앞 부분의 코드들에 비해서 난이도가 쉽습니다.

기존에 `canBlockMoveTo(toX, toY, shape)` 메소드를 만들어 놓았으니깐 이를 활용해서 구현해봅니다.

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

function dropBlock() {
    for(var toY=ROWS-1; toY >= 0; toY--) {
        if(canBlockMoveTo(x, toY, curShape)) {
            y = toY;    
            return;
        }
    }
}
</pre>

그 이후 키보드 아래 버튼을 눌렀을 때 `dropBlock()` 메소드가 호출되도록 하면 됩니다.