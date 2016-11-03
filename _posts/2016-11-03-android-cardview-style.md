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

<br>

### 각 Item들을 출력하는 View

<pre class="prettyprint" style="font-size:0.7em;">
&lt;FrameLayout xmlns:android="http://schemas.android.com/apk/res/android"
    xmlns:tools="http://schemas.android.com/tools"
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    android:paddingLeft="15dp"
    android:paddingRight="15dp"
    android:descendantFocusability="beforeDescendants"
    android:id="@+id/layout_background"&gt;

    &lt;LinearLayout
        android:orientation="horizontal"
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:paddingLeft="15dp"
        android:paddingTop="15dp"
        android:paddingBottom="15dp"
        android:paddingRight="15dp"
        android:background="@drawable/selector_card_background"
        android:descendantFocusability="afterDescendants"&gt;

        &lt;ImageView
            android:src="@drawable/footprint"
            android:layout_gravity="center_vertical"
            android:layout_width="64dp"
            android:layout_height="64dp" /&gt;

        &lt;LinearLayout
            android:layout_marginLeft="15dp"
            android:orientation="vertical"
            android:layout_width="match_parent"
            android:layout_height="wrap_content"&gt;

            &lt;TextView
                android:id="@+id/tv_date"
                android:textSize="14sp"
                android:textColor="@color/darkgray"
                android:text="2016-11-02 13:30:30"
                android:layout_width="match_parent"
                android:layout_height="wrap_content" /&gt;

            &lt;TextView
                android:id="@+id/tv_data"
                android:textSize="12sp"
                android:textColor="@color/black"
                android:text="Beacon1 : 35"
                android:layout_width="match_parent"
                android:layout_height="wrap_content" /&gt;
        &lt;/LinearLayout&gt;


    &lt;/LinearLayout&gt;

&lt;/FrameLayout&gt;
</pre>