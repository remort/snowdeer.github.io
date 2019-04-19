---
layout: post
title: Kotlin Material 스타일 TabLayout 예제
category: Android
tag: [Android, Kotlin]
---

## colors.xml

~~~
<?xml version="1.0" encoding="utf-8"?>
<resources>
  <color name="colorPrimary">#F48FB1</color>
  <color name="colorPrimaryDark">#F48FB1</color>
  <color name="textColorPrimary">#EC407A</color>
  <color name="windowBackground">#FFFFFF</color>
  <color name="navigationBarColor">#000000</color>
  <color name="colorAccent">#c8e8ff</color>
</resources>
~~~

<br>

## dimens.xml

~~~
<resources>
  <!-- Default screen margins, per the Android Design guidelines. -->
  <dimen name="activity_horizontal_margin">16dp</dimen>
  <dimen name="activity_vertical_margin">16dp</dimen>
  <dimen name="tab_max_width">264dp</dimen>
  <dimen name="tab_padding_bottom">16dp</dimen>
  <dimen name="tab_label">14sp</dimen>
  <dimen name="custom_tab_layout_height">72dp</dimen>
</resources>
~~~

<br>

## styles.xml

~~~
<resources>
  <!-- Base application theme. -->
  <style name="AppTheme" parent="RanMaterialTheme.Base">
    <!-- Customize your theme here. -->
    <item name="colorPrimary">@color/colorPrimary</item>
    <item name="colorPrimaryDark">@color/colorPrimaryDark</item>
    <item name="colorAccent">@color/colorAccent</item>
  </style>

  <style name="RanMaterialTheme.Base" parent="Theme.AppCompat.Light.DarkActionBar">
    <item name="windowNoTitle">true</item>
    <item name="windowActionBar">false</item>
  </style>
</resources>
~~~

<br>

## activity_main.xml

~~~
<?xml version="1.0" encoding="utf-8"?>

<android.support.design.widget.CoordinatorLayout xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:app="http://schemas.android.com/apk/res-auto"
  android:layout_width="match_parent"
  android:layout_height="match_parent">

  <android.support.design.widget.AppBarLayout
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:theme="@style/ThemeOverlay.AppCompat.Dark.ActionBar">

    <android.support.v7.widget.Toolbar
      android:id="@+id/toolbar"
      android:layout_width="match_parent"
      android:layout_height="?attr/actionBarSize"
      android:background="?attr/colorPrimary"
      app:layout_scrollFlags="scroll|enterAlways"
      app:popupTheme="@style/ThemeOverlay.AppCompat.Light"/>

    <RelativeLayout
      android:layout_width="match_parent"
      android:layout_height="wrap_content">

      <Button
        android:id="@+id/add_tab"
        android:layout_width="wrap_content"
        android:layout_height="wrap_content"
        android:layout_alignParentEnd="true"
        android:layout_gravity="end"
        android:text="+"/>

      <android.support.design.widget.TabLayout
        android:id="@+id/tabs"
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:layout_toStartOf="@id/add_tab"
        app:tabGravity="fill"
        app:tabMode="scrollable"/>

    </RelativeLayout>

  </android.support.design.widget.AppBarLayout>
  
  <android.support.v4.view.ViewPager
    android:id="@+id/viewpager"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    app:layout_behavior="@string/appbar_scrolling_view_behavior"/>

</android.support.design.widget.CoordinatorLayout>
~~~

<br>

## MainActivity.kt

<pre class="prettyprint">
import android.support.v7.app.AppCompatActivity
import android.os.Bundle
import android.support.design.widget.TabLayout
import android.support.v7.widget.Toolbar
import kotlinx.android.synthetic.main.activity_main.*

class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val toolbar = findViewById&lt;Toolbar&gt;(R.id.toolbar)

        setSupportActionBar(toolbar)
        supportActionBar?.setDisplayHomeAsUpEnabled(true)

        val list = ArrayList&lt;String&gt;()
        list.add("Year")
        list.add("Month")
        list.add("Week")
        list.add("Day")

        val adapter = TodoListPagerAdapter(supportFragmentManager)
        adapter.setList(list)
        viewpager.adapter = adapter

        val tabLayout = findViewById&lt;TabLayout&gt;(R.id.tabs)
        tabLayout.setupWithViewPager(viewpager)

        add_tab.setOnClickListener {
            list.add("Added")
            adapter.setList(list)
        }
    }
}
</pre>
