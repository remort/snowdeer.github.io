---
layout: post
title: Pixi.js 기본 흐름
category: Javascript
tag: [javascript, html5]
---
# Pixi.js의 기본 흐름

Pixi.js에 대한 더 자세한 내용은 [여기](https://pixijs.io/guides/basics/getting-started.html)에서 확인할 수 있습니다.

## 기본 흐름

Pixi.js의 기본 흐름은 다음과 같습니다.

1. HTML 작성
2. Pixi Application 생성
3. HTML에 Pixi Application 연동
4. stage에 오브젝트 추가
5. Update Looping

### Pixi Application 생성 및 HTML 연동

Pixi Application을 생성해서 HTML에 연동하는 예제 코드는 [이전 포스팅](http://snowdeer.github.io/javascript/2022/04/15/pixijs-example/)에서 언급했기 때문에 여기서는 `stage에 오브젝트 추가` 및 `Update Looping`에 대한 예제만 다룹니다.

### stage에 오브젝트 추가

stage에 오브젝트 추가는 화면에 오브젝트를 렌더링하는 단계가 아닙니다. 단순히 stage라는 컨테이너(Container)에 오브젝트를 추가할 뿐이며, 추가된 오브젝트는 별도의 렌더링 과정에서 그리게 됩니다.

다음과 같은 코드를 이용해서 stage에 오브젝트를 추가할 수 있습니다.

<pre class="prettyprint">
const sprite = PIXI.Sprite.from('pikachu.png');
 app.stage.addChild(sprite);
</pre>

### Update Loop

다음과 같은 코드를 통해 매 Frame 마다 화면을 갱신할 수 있습니다. `delta`에는 Tick마다 흘러간 시간이 매개변수로 전달됩니다. 따라서 `delta` 값을 이용해서 흘러간 시간을 체크할 수 있고, Frame rate도 조절할 수 있습니다. 

<pre class="prettyprint">
// Add a variable to count up the seconds our demo has been running
let elapsed = 0.0;
// Tell our application's ticker to run a new callback every frame, passing
// in the amount of time that has passed since the last tick
app.ticker.add((delta) => {
  // Add the time to our total elapsed time
  elapsed += delta;
  // Update the sprite's X position based on the cosine of our elapsed time.  We divide
  // by 50 to slow the animation down a bit...
  sprite.x = 100.0 + Math.cos(elapsed/50.0) * 100.0;
});
</pre>