---
layout: post
title: Pixi.js 기본 도형 그리기
category: Javascript
tag: [javascript, html5]
---
# 화면에 피카츄 그리기

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

      createDrawingObjects(app);
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

    const createDrawingObjects = (app) => {
      const rect = new PIXI.Graphics();
      rect.beginFill(0xff0000);
      rect.drawRect(50, 50, 200, 200);
      rect.endFill();
      app.stage.addChild(rect);

      const c1 = new PIXI.Graphics();
      c1.lineStyle(0);
      c1.beginFill(0x008080);
      c1.drawCircle(400, 100, 50);
      c1.endFill();
      app.stage.addChild(c1);

      const c2 = new PIXI.Graphics();
      c2.lineStyle(10, 0xfeeb77, 1);
      c2.beginFill(0x650a5a, 1);
      c2.drawCircle(400, 250, 50);
      c2.endFill();
      app.stage.addChild(c2);

      const polyline = new PIXI.Graphics();
      polyline.lineStyle(10, 0x00ffff);
      polyline.moveTo(50, 350);
      polyline.lineTo(200, 350);
      polyline.lineTo(280, 400);
      polyline.lineTo(100, 450);
      app.stage.addChild(polyline);

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
      app.stage.addChild(polygon);
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

## 실행 화면

![image](/assets/javascript/003.png)