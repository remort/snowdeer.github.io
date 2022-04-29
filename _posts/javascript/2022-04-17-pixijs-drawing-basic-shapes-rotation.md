---
layout: post
title: Pixi.js 기본 도형 그린 후 회전하기
category: Javascript
tag: [javascript, html5]
---
# 화면에 도형 그룹 그린 후 회전하는 코드

<pre class="prettyprint">
&lt;template&gt;
  &lt;div id="container"&gt;
    &lt;canvas id="pixi-canvas"&gt;&lt;/canvas&gt;
  &lt;/div&gt;
&lt;/template&gt;

&lt;script&gt;
import * as PIXI from "pixi.js";
import { onMounted } from "@vue/runtime-core";

export default {
  setup() {
    onMounted(() => {
      const app = createPixiApp();

      const container = createContainerWithDrawingObjects();
      app.stage.addChild(container);

      container.pivot.x = container.width / 2;
      container.pivot.y = container.height / 2;
      container.x = container.width / 2;
      container.y = container.height / 2;

      let elapsed = 0.0;
      app.ticker.add((delta) => {
        elapsed += delta;

        if (elapsed < 1) return;
        elapsed = 0;

        container.rotation = container.rotation + 0.01;
      });
    });

    const createPixiApp = () => {
      var canvas = document.getElementById("pixi-canvas");

      const app = new PIXI.Application({
        width: 800,
        height: 800,
        antialias: true,
        backgroundAlpha: true,
        view: canvas,
      });

      return app;
    };

    const createContainerWithDrawingObjects = () => {
      const container = new PIXI.Container();

      const rect = new PIXI.Graphics();
      rect.beginFill(0xff0000);
      rect.drawRect(50, 50, 200, 200);
      rect.endFill();
      container.addChild(rect);

      const c1 = new PIXI.Graphics();
      c1.lineStyle(0);
      c1.beginFill(0x008080);
      c1.drawCircle(400, 100, 50);
      c1.endFill();
      container.addChild(c1);

      const c2 = new PIXI.Graphics();
      c2.lineStyle(10, 0xfeeb77, 1);
      c2.beginFill(0x650a5a, 1);
      c2.drawCircle(400, 250, 50);
      c2.endFill();
      container.addChild(c2);

      const polyline = new PIXI.Graphics();
      polyline.lineStyle(10, 0x00ffff);
      polyline.moveTo(50, 350);
      polyline.lineTo(200, 350);
      polyline.lineTo(280, 400);
      polyline.lineTo(100, 450);
      container.addChild(polyline);

      const polygon = new PIXI.Graphics();
      polygon.lineStyle(0);
      polygon.beginFill(0x3500fa, 1);
      polygon.moveTo(550, 70);
      polygon.lineTo(650, 160);
      polygon.lineTo(730, 120);
      polygon.lineTo(680, 270);
      polygon.lineTo(540, 220);
      polygon.closePath();
      polygon.endFill();
      container.addChild(polygon);

      return container;
    };
  },
};
&lt;/script&gt;

&lt;style&gt;
#container {
  display: block;
  background: white;
  padding: 20px;
}

#pixi-canvas {
  width: 600px;
  height: 600px;
}
&lt;/style&gt;

</pre>

## Container를 회전하는 코드

여기서 중요한 부분은 아래 코드 부분입니다.
`container.pivot`는 회전할 때의 중심 좌표를 의미합니다. 기본값은 `0` 이며, 이 경우 회전할 때 Container의 왼쪽상단 부분의 꼭지점을 기준으로 회전합니다.

그리고 `pivot`를 Container의 중심으로 정하면, 화면에 좌표를 기준도 Container의 중심으로 변경됩니다. 따라서 기존과 같은 좌표에 도형을 그리기 위해서는 Container의 (x, y) 좌표를 변경할 필요가 있습니다.

<pre class="prettyprint">
container.pivot.x = container.width / 2;
container.pivot.y = container.height / 2;
container.x = container.width / 2;
container.y = container.height / 2;

let elapsed = 0.0;
app.ticker.add((delta) => {
  elapsed += delta;

  if (elapsed < 1) return;
  elapsed = 0;

  container.rotation = container.rotation + 0.01;
});
</pre>

위에서 

<pre class="prettyprint">
container.pivot.x = container.width / 2;
container.pivot.y = container.height / 2;
container.x = container.width / 2;
container.y = container.height / 2;
</pre>

코드는

<pre class="prettyprint">
container.pivot.set(container.width / 2, container.height / 2);
container.position.set(container.width / 2, container.height / 2);
</pre>

로도 작성 가능합니다.