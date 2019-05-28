---
layout: post
title: CollapsingToolbarLayout 예제
category: Android
tag: [Android]
---

## styles.xml

<pre class="prettyprint">

</pre>
&lt;resources&gt;

  &lt;style name="AppTheme" parent="Theme.AppCompat.Light.DarkActionBar"&gt;
    &lt;item name="colorPrimary">#80CBC4&lt;/item&gt;
    &lt;item name="colorPrimaryDark">#80CBC4&lt;/item&gt;
    &lt;item name="colorAccent">#3F51B5&lt;/item&gt;
  &lt;/style&gt;

  &lt;style name="SnowDeerTheme" parent="AppTheme"&gt;
    &lt;item name="windowNoTitle">true&lt;/item&gt;
    &lt;item name="windowActionBar">false&lt;/item&gt;
  &lt;/style&gt;

&lt;/resources&gt;

<br>

## activity_main.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;android.support.design.widget.CoordinatorLayout
  xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:app="http://schemas.android.com/apk/res-auto"
  xmlns:tools="http://schemas.android.com/tools"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  tools:context=".MainActivity"&gt;

  &lt;android.support.design.widget.AppBarLayout
    android:id="@+id/appBarLayout"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:theme="@style/ThemeOverlay.AppCompat.Dark.ActionBar"&gt;

    &lt;android.support.design.widget.CollapsingToolbarLayout
      android:id="@+id/collapsing_toolbar"
      android:layout_width="match_parent"
      android:layout_height="match_parent"
      android:fitsSystemWindows="true"
      app:contentScrim="@color/colorPrimary"
      app:expandedTitleMarginEnd="0dp"
      app:expandedTitleMarginStart="0dp"
      app:layout_scrollFlags="scroll|enterAlways|enterAlwaysCollapsed"&gt;

      &lt;LinearLayout
        android:layout_width="match_parent"
        android:layout_height="wrap_content"
        android:layout_margin="12dp"
        android:orientation="vertical"
        app:layout_collapseMode="parallax"
        app:layout_collapseParallaxMultiplier="0.7"&gt;

        &lt;ImageView
          android:layout_width="64dp"
          android:layout_height="64dp"
          android:layout_gravity="center_horizontal"
          app:srcCompat="@drawable/ic_launcher"/&gt;

        &lt;TextView
          style="@style/TextAppearance.AppCompat.Widget.ActionBar.Title"
          android:layout_width="wrap_content"
          android:layout_height="wrap_content"
          android:layout_gravity="center_horizontal"
          android:text="@string/app_name"/&gt;

      &lt;/LinearLayout&gt;

      &lt;android.support.v7.widget.Toolbar
        android:id="@+id/toolbar"
        android:layout_width="match_parent"
        android:layout_height="?attr/actionBarSize"
        android:popupTheme="@style/SnowDeerTheme"
        android:title="@string/app_name"
        app:layout_collapseMode="parallax"
        app:layout_scrollFlags="scroll|exitUntilCollapsed"/&gt;

    &lt;/android.support.design.widget.CollapsingToolbarLayout&gt;

    &lt;RelativeLayout
      android:layout_width="match_parent"
      android:layout_height="wrap_content"&gt;

      &lt;android.support.design.widget.TabLayout
        android:id="@+id/tabs"
        android:layout_width="wrap_content"
        android:layout_height="wrap_content"
        app:tabGravity="fill"
        app:tabMaxWidth="200dp"
        app:tabMode="scrollable"/&gt;
    &lt;/RelativeLayout&gt;
  &lt;/android.support.design.widget.AppBarLayout&gt;

  &lt;FrameLayout
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    android:layout_marginTop="8dp"
    android:layout_marginBottom="8dp"
    android:layout_marginStart="8dp"
    android:layout_marginEnd="8dp"
    app:layout_behavior="@string/appbar_scrolling_view_behavior"&gt;

    &lt;android.support.v4.view.ViewPager
      android:id="@+id/viewpager"
      android:layout_width="match_parent"
      android:layout_height="match_parent"/&gt;

  &lt;/FrameLayout&gt;

&lt;/android.support.design.widget.CoordinatorLayout&gt;
</pre>