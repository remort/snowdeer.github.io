---
layout: post
title: CustomView 만들기
category: Android
tag: [Android, UX]
---
# 사용자 정의 View

안드로이드에서 CustomView를 만드는 방법입니다.

## custom_view.xml

먼저 레이아웃을 작성합니다.

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:padding="10dp"
  android:orientation="horizontal"&gt;

  &lt;ToggleButton
    android:id="@+id/button1"
    android:layout_width="0px"
    android:layout_height="wrap_content"
    android:layout_weight="1"
    android:textOff="Off"
    android:textOn="On" /&gt;

  &lt;ToggleButton
    android:id="@+id/button2"
    android:layout_width="0px"
    android:layout_height="wrap_content"
    android:layout_weight="1"
    android:textOff="Off"
    android:textOn="On" /&gt;

  &lt;ToggleButton
    android:id="@+id/button3"
    android:layout_width="0px"
    android:layout_height="wrap_content"
    android:layout_weight="1"
    android:textOff="Off"
    android:textOn="On" /&gt;

&lt;/LinearLayout&gt;
</pre>

<br>

## attrs.xml에 속성 추가

그런 다음, CustomView에 대한 속성을 정의합니다. Java 코드 내에서 속성을 지정할 때는
굳이 이런 과정이 필요없지만, XML에서도 속성을 지정하고 싶을 때는 아래와 같이 
`attrs.xml`에 원하는 속성을 지정해야 합니다. `attrs.xml` 파일은 `values` 폴더 아래에 위치합니다.

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;resources&gt;
  &lt;declare-styleable name="custom_view"&gt;
    &lt;attr format="integer" name="selected" /&gt;
  &lt;/declare-styleable&gt;
&lt;/resources&gt;
</pre>

<br>

## CustomView.java

<pre class="prettyprint">
package snowdeer.customviewexample;

import android.content.Context;
import android.content.res.TypedArray;
import android.support.annotation.AttrRes;
import android.support.annotation.NonNull;
import android.support.annotation.Nullable;
import android.util.AttributeSet;
import android.view.LayoutInflater;
import android.widget.LinearLayout;
import android.widget.ToggleButton;

public class CustomView extends LinearLayout {

  ToggleButton[] buttons = new ToggleButton[3];
  int selectedId = 0;

  public CustomView(@NonNull Context context,
      @Nullable AttributeSet attrs,
      @AttrRes int defStyleAttr) {
    super(context, attrs, defStyleAttr);
    inflateViews(context, attrs);
  }

  public CustomView(@NonNull Context context,
      @Nullable AttributeSet attrs) {
    super(context, attrs);
    inflateViews(context, attrs);
  }

  void inflateViews(Context context, AttributeSet attrs) {
    LayoutInflater inflater = (LayoutInflater) context
        .getSystemService(Context.LAYOUT_INFLATER_SERVICE);
    inflater.inflate(R.layout.custom_view, this);

    if (attrs != null) {
      TypedArray array = context.obtainStyledAttributes(attrs, R.styleable.custom_view);
      selectedId = array.getInteger(0, 0);
      array.recycle();
    }
  }

  @Override
  protected void onFinishInflate() {
    super.onFinishInflate();

    buttons[0] = (ToggleButton) findViewById(R.id.button1);
    buttons[1] = (ToggleButton) findViewById(R.id.button2);
    buttons[2] = (ToggleButton) findViewById(R.id.button3);

    setSelected(selectedId);
  }

  public void setSelected(int id) {
    if ((id < 0) || (id > 3)) {
      return;
    }

    selectedId = id;
    for (ToggleButton btn : buttons) {
      btn.setChecked(false);
    }

    buttons[id].setChecked(true);
  }

  public int getSelectedId() {
    return selectedId;
  }

}
</pre>

<br>

## 활용

이제 CustomView 클래스의 생성이 끝났기 때문에 원하는 곳에서 사용을 하면 됩니다. 예를 들어,
`activity_main.xml`에서 사용을 한다면 다음과 같이 배치해주면 됩니다.

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:app="http://schemas.android.com/apk/res-auto"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:orientation="vertical"&gt;

  &lt;snowdeer.customviewexample.CustomView
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    app:selected="1" /&gt;

&lt;/LinearLayout&gt;
</pre>