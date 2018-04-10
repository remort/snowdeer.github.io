---
layout: post
title: Circle Layout 예제
category: Android
tag: [Android]
---
# Open Source를 이용한 Circle Layout

## ArcLayout

ArcLayout은 [여기](https://github.com/ogaclejapan/ArcLayout)에 공유되어 있습니다.

사용법은 간단합니다. 먼저 `gradle`에 다음과 같이 세팅합니다.

<br>

### gradle 세팅

<pre class="prettyprint">
dependencies {
    ...
    implementation 'com.ogaclejapan.arclayout:library:1.1.0@aar'
}
</pre>

<br>

### 사용법

XML에 레이아웃 설정만해도 사용할 수 있기 때문에 간단합니다.

<pre class="prettyprint">
&lt;com.ogaclejapan.arclayout.ArcLayout
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    app:arc_axisRadius="120dp"
    app:arc_color="#4DFFFFFF"
    app:arc_freeAngle="false"
    app:arc_origin="center"
    app:arc_radius="140dp"
    app:arc_reverseAngle="false"&gt;

    &lt;Button
        android:layout_width="48dp"
        android:layout_height="48dp"
        android:background="#f44336"
        android:gravity="center"
        android:text="A"
        android:textColor="#FFFFFF"
        app:arc_origin="center"/&gt;

    &lt;Button
        android:layout_width="48dp"
        android:layout_height="48dp"
        android:background="#9c27b0"
        android:gravity="center"
        android:text="B"
        android:textColor="#FFFFFF"
        app:arc_origin="center"/&gt;

    &lt;Button
        android:layout_width="48dp"
        android:layout_height="48dp"
        android:background="#3f51b5"
        android:gravity="center"
        android:text="C"
        android:textColor="#FFFFFF"
        app:arc_origin="center"/&gt;

    &lt;Button
        android:layout_width="48dp"
        android:layout_height="48dp"
        android:background="#4caf50"
        android:gravity="center"
        android:text="D"
        android:textColor="#FFFFFF"
        app:arc_origin="center"/&gt;

    &lt;Button
        android:layout_width="48dp"
        android:layout_height="48dp"
        android:background="#ff5722"
        android:gravity="center"
        android:text="E"
        android:textColor="#FFFFFF"
        app:arc_origin="center"/&gt;

    &lt;/com.ogaclejapan.arclayout.ArcLayout&gt;
</pre>

![image](/assets/android/001.png)

레이아웃의 속성을 변경해서 다음과 같이 Arc 형태로 컴포넌트를 배치할 수도 있습니다.

![image](/assets/android/002.png)

<br>

## Circular-Layout

Circular Layout은 [여기](https://github.com/andreilisun/Circular-Layout)에 공유되어 있습니다. 좋아요 개수랑 Fork 수는 얼마되지 않지만, 역시나 사용하기 편리한 오픈소스입니다. 사용법은 다음과 같습니다.

<br>

### gradle 세팅

<pre class="prettyprint">
dependencies {
    ...
    implementation 'com.github.andreilisun:circular-layout:1.0'
}
</pre>

<br>

### 사용법

XML에 레이아웃 설정만해도 사용할 수 있기 때문에 간단합니다.

<pre class="prettyprint">
&lt;com.github.andreilisun.circular_layout.CircularLayout
  android:layout_width="220dp"
  android:layout_height="220dp"&gt;

  &lt;android.support.v7.widget.SwitchCompat
    android:layout_width="wrap_content"
    android:layout_height="wrap_content" /&gt;

  &lt;ImageView
    android:layout_width="30dp"
    android:layout_height="30dp"
    android:src="@mipmap/ic_launcher" /&gt;

  &lt;Button
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:text="ok" /&gt;

  &lt;CheckBox
    android:layout_width="wrap_content"
    android:layout_height="wrap_content" /&gt;

  &lt;RadioButton
    android:layout_width="wrap_content"
    android:layout_height="wrap_content" /&gt;

  &lt;Chronometer
    android:layout_width="wrap_content"
    android:layout_height="wrap_content" /&gt;

&lt;/com.github.andreilisun.circular_layout.CircularLayout&gt;
</pre>

또한 다음과 같은 형태로 Java 코드내에서 프로그래밍적으로 사용을 할 수도 있습니다.

<pre class="prettyprint">
public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        final CircularLayout circularLayout = findViewById(R.id.circular_layout);
        int expectedViewsQuantity = 12;
        circularLayout.setCapacity(expectedViewsQuantity);
        for (int i = 0; i < expectedViewsQuantity; i++) {
            TextView textView = (TextView)
                    LayoutInflater.from(this).inflate(R.layout.number_text_view, null);
            textView.setText(String.valueOf(i));
            circularLayout.addView(textView);
        }
    }
}
</pre>

<br>

## Circle Layout

Circle Layout은 [여기](https://github.com/francoiscampbell/CircleLayout)에서 확인할 수 있습니다. 위에서 다룬 Circular Layout보다 좋아요 개수랑 Fork 횟수가 더 많습니다. 사용성은 동일합니다.

## gradle 세팅

<pre class="prettyprint">
dependencies {
    ...
    implementation 'io.github.francoiscampbell:circlelayout:0.3.0'
}
</pre>

<br>

### 사용법

<pre class="prettyprint">
&lt;io.github.francoiscampbell.circlelayout.CircleLayout 
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  cl:cl_angleOffset="90"
  cl:cl_centerView="@+id/centerView"
  cl:cl_direction="clockwise"&rt;

  &lt;Switch
    android:id="@+id/centerView"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"/&rt;

  &lt;TextView
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:text="12"
    android:textColor="@color/testTextColor"
    android:textSize="@dimen/clockTestSize"/&rt;

  &lt;Button
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:text="Button"/&rt;

  &lt;CheckBox
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"/&rt;

  &lt;SeekBar
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"/&rt;

&lt;/io.github.francoiscampbell.circlelayout.CircleLayout&rt;
</pre>

위의 오픈 소스들을 사용하면 손쉽게 원형 레이아웃을 구현할 수 있습니다. 하지만, 최근에 구글에서 `ConstraintLayout`에도 원형 레이아웃 기능을 집어넣어서 개인적으로는 `ConstraintLayout`을 활용하는 것이 좀 더 낫지 않나 생각이 듭니다.