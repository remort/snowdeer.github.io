---
layout: post
title: Pixi.js 간단한 예제 (with Vue) - (2)
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

      createObject(app);
    });

    const createPixiApp = () => {
      var canvas = document.getElementById("pixi-canvas");

      const app = new PIXI.Application({
        width: 1000,
        height: 1000,
        antialias: true,
        backgroundAlpha: true,
        view: canvas,
      });

      return app;
    };

    const createObject = (app) => {
      const pikachu = PIXI.Sprite.from(require("@/assets/pikachu.png"));
      pikachu.x = 50;
      pikachu.y = 50;
      pikachu.width = 832;
      pikachu.height = 846;

      app.stage.addChild(pikachu);
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
  width: 800px;
  height: 800px;
}
&lt;/style&gt;

</pre>

## 사용한 리소스

![image](/assets/javascript/pikachu.png)

## 실행 화면

![image](/assets/javascript/002.png)