---
layout: post
title: ListView의 가장 아래쪽으로 자동 스크롤 시키기
category: Android
tag: [Android]
---

안드로이드 ListView의 가장 아래쪽 아이템으로 자동 스크롤 시키는 코드입니다.

<br>

# alwaysScroll 옵션 적용

먼저 ListView에 `alwaysScroll` 옵션을 적용시켜줍니다.

XML 레이아웃내의 ListView attribute에 

<pre class="prettyprint">
android:transcriptMode="alwaysScroll"
</pre>

항목을 추가시키거나, Java 코드에서 

<pre class="prettyprint">
listview.setTranscriptMode(ListView.TRANSCRIPT_MODE_ALWAYS_SCROLL);
</pre>

와 같이 작성하시면 됩니다.

<br>

# 자동 스크롤 코드

ListView의 아이템이 변경되었다는 이벤트가 왔을 때 아래 코드를 수행하시면, ListView의 최하단 칸으로 강제 이동 시켜줄 수 있습니다. 

<pre class="prettyprint">
listview.setSelection(adapter.getCount() - 1);
</pre>