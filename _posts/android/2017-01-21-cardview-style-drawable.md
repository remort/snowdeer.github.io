---
layout: post
title: CardView 스타일 Drawable
category: Android
tag: [Android, UX]
---

안드로이드 L(롤리팝) 부터 [CardView](https://developer.android.com/training/material/lists-cards.html?hl=ko)라는
UX 컴포넌트가 추가되었습니다. 하지만 여기서는 그 이전 버전에서도 사용할 수 있도록
CardView와 유사한 형태의 Drawable를 만들어보고자 합니다.

먼저 drawable 폴더에 각 xml 파일들을 작성합니다.

<br>
## layer_card_background.xml
<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;layer-list xmlns:android="http://schemas.android.com/apk/res/android"&gt;
  &lt;item&gt;
    &lt;shape android:shape="rectangle"&gt;
      &lt;solid android:color="#CABBBBBB" /&gt;
      &lt;corners android:radius="2dp" /&gt;
    &lt;/shape&gt;
  &lt;/item&gt;

  &lt;item
    android:bottom="2dp"
    android:left="0dp"
    android:right="0dp"
    android:top="0dp"&gt;
    &lt;shape android:shape="rectangle"&gt;
      &lt;solid android:color="@android:color/white" /&gt;
      &lt;corners android:radius="2dp" /&gt;
    &lt;/shape&gt;
  &lt;/item&gt;
&lt;/layer-list&gt;</pre>
&nbsp;
<h3>layer_card_background_selected.xml</h3>
<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;layer-list xmlns:android="http://schemas.android.com/apk/res/android"&gt;
  &lt;item&gt;
    &lt;shape android:shape="rectangle"&gt;
      &lt;solid android:color="#CABBBBBB" /&gt;
      &lt;corners android:radius="2dp" /&gt;
    &lt;/shape&gt;
  &lt;/item&gt;

  &lt;item
    android:bottom="2dp"
    android:left="0dp"
    android:right="0dp"
    android:top="0dp"&gt;
    &lt;shape android:shape="rectangle"&gt;
      &lt;solid android:color="#CCCCCC" /&gt;
      &lt;corners android:radius="2dp" /&gt;
    &lt;/shape&gt;
  &lt;/item&gt;
&lt;/layer-list&gt;</pre>

<br>
## selector_card_background.xml
<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;

&lt;selector xmlns:android="http://schemas.android.com/apk/res/android"&gt;
  &lt;item
    android:drawable="@drawable/layer_card_background_selected"
    android:state_pressed="true" /&gt;

  &lt;item android:drawable="@drawable/layer_card_background" /&gt;
&lt;/selector&gt;</pre>
&nbsp;
<h3>ListView</h3>
<pre class="prettyprint">&lt;ListView
  android:cacheColorHint="@android:color/transparent"
  android:divider="@null"
  android:dividerHeight="10dp"
  android:footerDividersEnabled="true"
  android:headerDividersEnabled="true"
  android:id="@+id/listview"
  android:layout_above="@id/layout_bottom_buttons"
  android:layout_height="match_parent"
  android:layout_width="match_parent"
  android:listSelector="@android:color/transparent" /&gt;</pre>
<br>
## 각 Item들을 출력하는 View
<pre class="prettyprint">&lt;FrameLayout xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:tools="http://schemas.android.com/tools"
  android:id="@+id/layout_background"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:paddingLeft="15dp"
  android:paddingRight="15dp"
  android:descendantFocusability="beforeDescendants"&gt;

  &lt;LinearLayout
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:paddingTop="15dp"
    android:paddingBottom="15dp"
    android:paddingLeft="15dp"
    android:paddingRight="15dp"
    android:background="@drawable/selector_card_background"
    android:descendantFocusability="afterDescendants"
    android:orientation="horizontal"&gt;

    &lt;ImageView
      android:layout_width="64dp"
      android:layout_height="64dp"
      android:layout_gravity="center_vertical"
      android:src="@drawable/footprint" /&gt;

    &lt;LinearLayout
      android:layout_width="match_parent"
      android:layout_height="wrap_content"
      android:layout_marginLeft="15dp"
      android:orientation="vertical"&gt;

      &lt;TextView
        android:id="@+id/tv_date"
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:text="2016-11-02 13:30:30"
        android:textColor="@color/darkgray"
        android:textSize="14sp" /&gt;

      &lt;TextView
        android:id="@+id/tv_data"
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:text="Beacon1 : 35"
        android:textColor="@color/black"
        android:textSize="12sp" /&gt;
    &lt;/LinearLayout&gt;


  &lt;/LinearLayout&gt;

&lt;/FrameLayout&gt;</pre>
