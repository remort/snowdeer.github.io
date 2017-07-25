---
layout: post
title: ViewPager 사용 예제
category: Android
tag: [Android, ViewPager]
---

안드로이드 ViewPager를 활용하는 간단한 예제 코드입니다.

<br>

## Layout 코드
<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;RelativeLayout xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:tools="http://schemas.android.com/tools"
  android:layout_width="match_parent"
  android:layout_height="match_parent"&gt;

  &lt;android.support.v4.view.ViewPager
    android:id="@+id/viewpager"
    android:layout_width="match_parent"
    android:layout_height="match_parent"&gt;

    &lt;android.support.v4.view.PagerTitleStrip
      android:id="@+id/pager_title_strip"
      android:layout_width="match_parent"
      android:layout_height="wrap_content"
      android:paddingBottom="8dp"
      android:paddingTop="8dp"/&gt;

  &lt;/android.support.v4.view.ViewPager&gt;

&lt;/RelativeLayout&gt;</pre>

<br>
그리고 Java 코드는 다음과 같습니다. 이번 예제 코드는 Fragment에서 ViewPager를 사용할
경우의 코드인데, Activity에서 사용할 경우에도 큰 차이는 없습니다.
<br>

## Java 코드
<pre class="prettyprint">package com.lnc.prototype.ui.main;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentPagerAdapter;
import android.support.v4.view.ViewPager;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;

import com.lnc.prototype.R;

public class LocationDataFragment extends Fragment {

  private ViewPager mViewPager;
  private ViewPagerAdapter mViewPagerAdapter;

  @Override
  public View onCreateView(LayoutInflater inflater, ViewGroup container,
      Bundle savedInstanceState) {
    View view = inflater.inflate(R.layout.fragment_location_data,
        container, false);

    mViewPager = (ViewPager) view.findViewById(R.id.viewpager);
    mViewPagerAdapter = new ViewPagerAdapter(getFragmentManager());
    mViewPager.setAdapter(mViewPagerAdapter);

    return view;
  }


  private class ViewPagerAdapter extends FragmentStatePagerAdapter {

    public ViewPagerAdapter(FragmentManager fm) {
      super(fm);
    }

    @Override
    public int getCount() {
      return 2;
    }

    @Override
    public Fragment getItem(int position) {
      Fragment fragment = null;
      switch(position) {
        case 0:
          fragment = new BeaconRadarFragment();
          break;

        case 1:
          fragment = new LocationHistoryFragment();
          break;
      }
      return fragment;
    }

    @Override
    public CharSequence getPageTitle(int position) {
      switch(position) {
        case 0:
          return "Beacon Radar";
        case 1:
          return "Location History";
      }

      return super.getPageTitle(position);
    }
  }

  ;
}
</pre>
