---
layout: post
title: Pixi.js 간단한 예제 with Vue
category: Javascript
tag: [javascript, html5]
---
# Pixi.js 샘플

<pre class="prettyprint">
&lt;template&gt;
  &lt;div id="container" class="connections"&gt;
    &lt;canvas id="pixi-canvas"&gt;&lt;/canvas&gt;
  &lt;/div&gt;
&lt;/template&gt;

&lt;script&gt;
import * as PIXI from "pixi.js";
import { onMounted } from "@vue/runtime-core";

export default {
  setup() {
    onMounted(() => {
      drawPixi();
    });
    const drawPixi = () => {
      var canvas = document.getElementById("pixi-canvas");

      console.log(`window size(${window.innerWidth}, ${window.innerHeight})`);
      console.log(`canvas size(${canvas.width}, ${canvas.height})`);

      const app = new PIXI.Application({
        width: window.innerWidth,
        height: window.innerHeight,
        antialias: true,
        backgroundAlpha: true,
        view: canvas,
      });

      const graphics = new PIXI.Graphics();
      graphics.lineStyle(8, 0x008080);
      graphics.moveTo(0, 250);
      graphics.lineTo(800, 500);
      app.stage.addChild(graphics);

      const sprite = PIXI.Sprite.from(require("@/assets/sample.png"));
      app.stage.addChild(sprite);

      let elapsed = 0.0;
      app.ticker.add((delta) => {
        elapsed += delta;
        sprite.x = 100.0 + Math.cos(elapsed / 50.0) * 100.0;
      });
    };

    return {
      drawPixi,
    };
  },
};
&lt;/script&gt;

&lt;style&gt;
#app {
  font-family: Avenir, Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
}

#container {
  display: block;
  background: white;
  padding: 20px;
}

#pixi-canvas {
  width: 80vw;
  height: 80vh;
}
&lt;/style&gt;

</pre>

## 실행 화면

![image](/assets/javascript/001.png)