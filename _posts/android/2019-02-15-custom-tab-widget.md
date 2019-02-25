---
layout: post
title: Custom Tab Widget 사용 예제
category: Android
tag: [Android]
---
## drawable/tab_focused.xml

<pre class="prettyprint">
&lt;shape xmlns:android="http://schemas.android.com/apk/res/android"
  android:padding="10dp"
  android:shape="rectangle"&gt;
  &lt;gradient
    android:angle="-90"
    android:endColor="#ffcc00"
    android:startColor="#ffcc00"/&gt;
&lt;/shape&gt;
</pre>

<br>

## drawable/tab_pressed.xml

<pre class="prettyprint">
&lt;layer-list xmlns:android="http://schemas.android.com/apk/res/android"&gt;
  &lt;item&gt;
    &lt;shape android:shape="rectangle"&gt;
      &lt;solid android:color="#FFcccccc"/&gt;
    &lt;/shape&gt;
  &lt;/item&gt;

  &lt;item
    android:bottom="0dp"
    android:left="-3dp"
    android:right="-3dp"
    android:top="-3dp"&gt;
    &lt;shape
      android:shape="rectangle"&gt;
      &lt;padding
        android:bottom="15dp"
        android:left="10dp"
        android:right="10dp"
        android:top="15dp"/&gt;
      &lt;stroke
        android:color="#FF00ccff"
        android:width="2dp"/&gt;
      &lt;solid android:color="#00000000"/&gt;
    &lt;/shape&gt;
  &lt;/item&gt;
&lt;/layer-list&gt;
</pre>

<br>

## drawable/tab_selected.xml

<pre class="prettyprint">
&lt;layer-list xmlns:android="http://schemas.android.com/apk/res/android"&gt;
  &lt;item&gt;
    &lt;shape android:shape="rectangle"&gt;
      &lt;solid android:color="#FFefefef"/&gt;
    &lt;/shape&gt;
  &lt;/item&gt;

  &lt;item
    android:bottom="0dp"
    android:left="-3dp"
    android:right="-3dp"
    android:top="-3dp"&gt;
    &lt;shape
      android:shape="rectangle"&gt;
      &lt;padding
        android:bottom="15dp"
        android:left="10dp"
        android:right="10dp"
        android:top="15dp"/&gt;
      &lt;stroke
        android:color="#FF00ccff"
        android:width="2dp"/&gt;
      &lt;solid android:color="#00000000"/&gt;
    &lt;/shape&gt;
  &lt;/item&gt;
&lt;/layer-list&gt;
</pre>

<br>

## drawable/tab_unselected.xml

<pre class="prettyprint">
&lt;layer-list xmlns:android="http://schemas.android.com/apk/res/android"&gt;
  &lt;item&gt;
    &lt;shape android:shape="rectangle"&gt;
      &lt;solid android:color="#FFefefef"/&gt;
    &lt;/shape&gt;
  &lt;/item&gt;

  &lt;item
    android:bottom="0dp"
    android:left="-3dp"
    android:right="-3dp"
    android:top="-3dp"&gt;
    &lt;shape
      android:shape="rectangle"&gt;
      &lt;padding
        android:bottom="15dp"
        android:left="10dp"
        android:right="10dp"
        android:top="15dp"/&gt;
      &lt;stroke
        android:color="#FF00ccff"
        android:width="2dp"/&gt;
      &lt;solid android:color="#00000000"/&gt;
    &lt;/shape&gt;
  &lt;/item&gt;
&lt;/layer-list&gt;
</pre>

<br>

## drawable/tab_indicator.xml

<pre class="prettyprint">
&lt;selector xmlns:android="http://schemas.android.com/apk/res/android"&gt;

  &lt;!-- Non focused states --&gt;
  &lt;item android:drawable="@drawable/tab_unselected" android:state_focused="false" android:state_pressed="false" android:state_selected="false"/&gt;
  &lt;item android:drawable="@drawable/tab_selected" android:state_focused="false" android:state_pressed="false" android:state_selected="true"/&gt;

  &lt;!-- Focused states --&gt;
  &lt;item android:drawable="@drawable/tab_focused" android:state_focused="true" android:state_pressed="false" android:state_selected="false"/&gt;
  &lt;item android:drawable="@drawable/tab_focused" android:state_focused="true" android:state_pressed="false" android:state_selected="true"/&gt;

  &lt;!-- Pressed --&gt;
  &lt;item android:drawable="@drawable/tab_pressed" android:state_pressed="true" android:state_selected="true"/&gt;
  &lt;item android:drawable="@drawable/tab_pressed" android:state_pressed="true"/&gt;

&lt;/selector&gt;
</pre>

<br>

## tab_menu.xml

<pre class="prettyprint">
&lt;LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
  android:layout_width="fill_parent"
  android:layout_height="fill_parent"
  android:background="@drawable/tab_indicator"
  android:orientation="vertical"&gt;

  &lt;TextView
    android:id="@+id/textView"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:layout_gravity="center_vertical|center_horizontal"
    android:textSize="12sp"/&gt;

&lt;/LinearLayout&gt;
</pre>

<br>

## main_activity.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
<&lt; xmlns:app="http://schemas.android.com/apk/res-auto"
  xmlns:tools="http://schemas.android.com/tools"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:orientation="horizontal"
  tools:context=".MainActivity"&gt;

  &lt;LinearLayout
    android:layout_width="0px"
    android:layout_height="wrap_content"
    android:layout_weight="1"
    android:orientation="vertical"&gt;
    &lt;TabHost
      android:id="@+id/tab_host"
      android:layout_width="match_parent"
      android:layout_height="match_parent"&gt;

      &lt;LinearLayout
        android:layout_width="match_parent"
        android:layout_height="match_parent"
        android:orientation="vertical"&gt;

        &lt;TabWidget
          android:id="@android:id/tabs"
          android:layout_width="match_parent"
          android:layout_height="wrap_content"
          android:theme="@style/MenuTabStyle"/&gt;

        &lt;FrameLayout
          android:id="@android:id/tabcontent"
          android:layout_width="match_parent"
          android:layout_height="wrap_content"&gt;

          &lt;LinearLayout
            android:id="@+id/tab1"
            android:layout_width="match_parent"
            android:layout_height="wrap_content"
            android:orientation="vertical"&gt;
            &lt;Button
              android:id="@+id/clearBtn"
              android:layout_width="match_parent"
              android:layout_height="wrap_content"
              android:text="Clear"/&gt;

          &lt;/LinearLayout&gt;

          &lt;LinearLayout
            android:id="@+id/tab2"
            android:layout_width="match_parent"
            android:layout_height="wrap_content"
            android:orientation="vertical"&gt;

            &lt;Button
              android:id="@+id/addRectBtn"
              android:layout_width="match_parent"
              android:layout_height="wrap_content"
              android:text="Add Rect"/&gt;


          &lt;/LinearLayout&gt;

          &lt;LinearLayout
            android:id="@+id/tab3"
            android:layout_width="match_parent"
            android:layout_height="wrap_content"
            android:orientation="vertical"&gt;

            &lt;Button
              android:id="@+id/addLineBtn"
              android:layout_width="match_parent"
              android:layout_height="wrap_content"
              android:text="Add Line"/&gt;

          &lt;/LinearLayout&gt;
        &lt;/FrameLayout&gt;
      &lt;/LinearLayout&gt;

    &lt;/TabHost&gt;

  &lt;/LinearLayout&gt;

  &lt;FrameLayout
    android:id="@+id/canvas_view"
    android:layout_width="0px"
    android:layout_height="match_parent"
    android:layout_weight="5"/&gt;


&lt;/LinearLayout&gt;
</pre>

<br>

## MainActivity.java

<pre class="prettyprint">
public class MainActivity extends AppCompatActivity {

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    TabHost tabHost = (TabHost) findViewById(R.id.tab_host);
    tabHost.setup();
    setNewTab(tabHost, "Normal", R.id.tab1);
    setNewTab(tabHost, "Rect", R.id.tab2);
    setNewTab(tabHost, "Line", R.id.tab3);

    tabHost.setOnTabChangedListener(new OnTabChangeListener() {
      @Override
      public void onTabChanged(String tabId) {
        //TODO

      }
    });
  }

  private void setNewTab(TabHost host, String title, int contentID) {
    TabHost.TabSpec tabSpec = host.newTabSpec(title);
    tabSpec.setIndicator(getTabIndicator(title));
    tabSpec.setContent(contentID);
    host.addTab(tabSpec);
  }

  private View getTabIndicator(String title) {
    View view = LayoutInflater.from(getApplicationContext()).inflate(R.layout.tab_menu, null);
    TextView tv = view.findViewById(R.id.textView);
    tv.setText(title);
    return view;
  }
}

</pre>