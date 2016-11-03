---
layout: post
title: CardView 스타일 적용
category: Android
tag: [android, cardview, style]
---

ListView를 CardView 스타일로 보여주는 예제입니다. 


먼저 drawable 폴더에 각 xml 파일들을 작성합니다.

<br>

### layer_card_background.xml 

<pre class="prettyprint" style="font-size:0.7em;">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;layer-list xmlns:android="http://schemas.android.com/apk/res/android"&gt;
    &lt;item&gt;
        &lt;shape android:shape="rectangle"&gt;
            &lt;solid android:color="#CABBBBBB"/&gt;
            &lt;corners android:radius="2dp" /&gt;
        &lt;/shape&gt;
    &lt;/item&gt;

    &lt;item
        android:left="0dp"
        android:right="0dp"
        android:top="0dp"
        android:bottom="2dp"&gt;
        &lt;shape android:shape="rectangle"&gt;
            &lt;solid android:color="@android:color/white"/&gt;
            &lt;corners android:radius="2dp" /&gt;
        &lt;/shape&gt;
    &lt;/item&gt;
&lt;/layer-list&gt;
</pre>

<br>

### layer_card_background_selected.xml

<pre class="prettyprint" style="font-size:0.7em;">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;layer-list xmlns:android="http://schemas.android.com/apk/res/android"&gt;
    &lt;item&gt;
        &lt;shape android:shape="rectangle"&gt;
            &lt;solid android:color="#CABBBBBB"/&gt;
            &lt;corners android:radius="2dp" /&gt;
        &lt;/shape&gt;
    &lt;/item&gt;

    &lt;item
        android:left="0dp"
        android:right="0dp"
        android:top="0dp"
        android:bottom="2dp"&gt;
        &lt;shape android:shape="rectangle"&gt;
            &lt;solid android:color="#CCCCCC"/&gt;
            &lt;corners android:radius="2dp" /&gt;
        &lt;/shape&gt;
    &lt;/item&gt;
&lt;/layer-list&gt;
</pre>

<br>

### selector_card_background.xml

<pre class="prettyprint" style="font-size:0.7em;">
&lt;?xml version="1.0" encoding="utf-8"?&gt;

&lt;selector xmlns:android="http://schemas.android.com/apk/res/android"&gt;
    &lt;item
        android:state_pressed="true"
        android:drawable="@drawable/layer_card_background_selected" /&gt;

    &lt;item android:drawable="@drawable/layer_card_background" /&gt;
&lt;/selector&gt;
</pre>

<br>

### ListView

<pre class="prettyprint" style="font-size:0.7em;">
&lt;ListView
        android:id="@+id/listview"
        android:layout_above="@id/layout_bottom_buttons"
        android:divider="@null"
        android:dividerHeight="10dp"
        android:listSelector="@android:color/transparent"
        android:cacheColorHint="@android:color/transparent"
        android:headerDividersEnabled="true"
        android:footerDividersEnabled="true"
        android:layout_width="match_parent"
        android:layout_height="match_parent"/&gt;
</pre>