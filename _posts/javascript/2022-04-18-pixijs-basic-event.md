---
layout: post
title: Pixi.js Event 등록하기
category: Javascript
tag: [javascript, html5]
---
# Pixi 캔버스 상의 오브젝트에 이벤트 등록하기

아래와 같은 코드로 작성하면 다음과 같은 이벤트들을 사용할 수 있습니다.
- pointerdown
- pointermove
- pointerup
- click
- pointerover 
- pointerout

참고로, 이벤트들은 Pixi.js에서 등록된 이벤트가 아니라 HTML5에 정의되어 있는 이벤트들입니다.

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

      createInteractionExample(app);
    });

    const createPixiApp = () => {
      var canvas = document.getElementById("pixi-canvas");

      const app = new PIXI.Application({
        view: canvas,
      });

      return app;
    };

    const createInteractionExample = (app) => {
      const container = new PIXI.Container();
      const c1 = new PIXI.Graphics();
      c1.lineStyle(0);
      c1.beginFill(0x008080);
      c1.drawCircle(400, 100, 50);
      c1.endFill();
      container.addChild(c1);

      app.stage.addChild(container);

      c1.interactive = true;
      c1.on("pointerdown", (e) => {
        console.log("pointerdown");
        console.log(e);
      });
      // c1.on("pointermove", (e) => {
      //   console.log("pointermove");
      //   console.log(e);
      // });
      c1.on("pointerup", (e) => {
        console.log("pointerup");
        console.log(e);
      });
      c1.on("click", (e) => {
        console.log("click");
        console.log(e);
      });
      c1.on("pointerover", (e) => {
        console.log("pointerover");
        console.log(e);
      });
      c1.on("pointerout", (e) => {
        console.log("pointerout");
        console.log(e);
      });
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

## 이벤트

개인적으로 Pixi이 Event들은 만족스럽지 않습니다. 제 프로젝트의 문제인지 일시적 버그인지 모르겠지만, 위의 예제에서 오브젝트에 설정한 `pointermove` 이벤트가 오브젝트가 아닌 캔버스 전체에 적용되는 버그도 있으며, `pointdown`, `pointup`, `click` 이벤트들이 일반적인 형태로 동작하지 않는 것 같았습니다. 보통 `pointdown` 후 좌표를 크게 이동한 후 `pointup`을 할 경우 `click` 이벤트는 호출되지 않는데, Pxii에서는 항상 호출되네요.

그리고 아래와 같이 로그에서도 모든 이벤트의 `type`이 `mousemove`로 전달되어 오는 등 아쉬운 점이 많네요.

![image](/assets/javascript/004.png)