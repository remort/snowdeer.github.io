---
layout: post
title: TextView의 Width를 Programatically하게 얻기
category: Android
tag: [Android]
---

`TextView`에 Text가 렌더링되기 전에 미리 width를 얻는 코드입니다.
Text의 width는 TextView에서 출력되는 폰트의 종류와 크기 등에 영향을 받기 때문에, TextView의 현재 `Paint()` 정보를 가져와서 계산을 해줍니다.

<pre class="prettyprint">
textView.setText(texxt);
int width = (int)textView.getPaint().measureText(text);
</pre>
