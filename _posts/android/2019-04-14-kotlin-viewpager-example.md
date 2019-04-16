---
layout: post
title: Kotlin ViewPager 예제
category: Android, Kotlin
---

## build.gradle

<pre class="prettyprint">
dependencies {
    implementation 'com.android.support:design:28.0.0'
}
</pre>

<br>

## activity_main.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;LinearLayout
  xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:tools="http://schemas.android.com/tools"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:orientation="vertical"
  tools:context=".MainActivity"&gt;

  &lt;android.support.design.widget.TabLayout
    android:id="@+id/tab"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    app:tabIndicatorColor="@color/colorAccent"
    app:tabSelectedTextColor="@color/colorAccent"
    app:tabTextColor="@color/colorPrimary"/&gt;

  &lt;android.support.v4.view.ViewPager
    android:id="@+id/viewpager"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"/&gt;

&lt;/LinearLayout&gt;
</pre>

<br>

## fragment_page.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;FrameLayout xmlns:android="http://schemas.android.com/apk/res/android"
  android:layout_width="match_parent"
  android:layout_height="match_parent"&gt;

  &lt;TextView
    android:id="@+id/textview"
    android:textStyle="bold"
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    android:text="Hello"
    android:textColor="@android:color/holo_blue_dark"
    android:textSize="32sp"/&gt;

&lt;/FrameLayout&gt;
</pre>

<br>

## BaseFragment.kt

<pre class="prettyprint">
import android.support.v4.app.Fragment

abstract class BaseFragment : Fragment() {
    abstract fun title(): String
}
</pre>

<br>

## FirstPageFragment.kt

<pre class="prettyprint">
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import com.snowdeer.hellokotlin.R

class FirstPageFragment : BaseFragment() {

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        val view = inflater.inflate(R.layout.fragment_page, container, false)
        view.findViewById&lt;TextView&gt;(R.id.textview).text = "First Page"

        return view
    }

    override fun title(): String {
        return "First"
    }
}
</pre>

<br>

## SecondPageFragment.kt

<pre class="prettyprint">
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import com.snowdeer.hellokotlin.R

class SecondPageFragment : BaseFragment() {

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        val view = inflater.inflate(R.layout.fragment_page, container, false)
        view.findViewById&lt;TextView&gt;(R.id.textview).text = "Second Page"

        return view
    }

    override fun title(): String {
        return "Second"
    }
}
</pre>

<br>

## ThirdPageFragment.kt

<pre class="prettyprint">
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import com.snowdeer.hellokotlin.R

class ThirdPageFragment : BaseFragment() {

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?, savedInstanceState: Bundle?): View? {
        val view = inflater.inflate(R.layout.fragment_page, container, false)
        view.findViewById&lt;TextView&gt;(R.id.textview).text = "Third Page"

        return view
    }

    override fun title(): String {
        return "Third"
    }
}
</pre>

<br>

## SamplePagerAdapter.kt

<pre class="prettyprint">
class SamplePagerAdapter : FragmentPagerAdapter {

    private val list: ArrayList&lt;BaseFragment&gt; = ArrayList();

    constructor(fragmentManager: FragmentManager) : super(fragmentManager) {
        list.add(FirstPageFragment())
        list.add(SecondPageFragment())
        list.add(ThirdPageFragment())
    }

    override fun getPageTitle(position: Int): CharSequence? {
        return list[position].title()
    }

    override fun getItem(position: Int): Fragment {
        return list.get(position)
    }

    override fun getCount(): Int {
        return list.size
    }
}
</pre>

<br>

## MainActivity.kt

<pre class="prettyprint">
import android.support.v7.app.AppCompatActivity
import android.os.Bundle
import com.snowdeer.hellokotlin.viewpager.SamplePagerAdapter
import kotlinx.android.synthetic.main.activity_main.*

class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val adapter = SamplePagerAdapter(supportFragmentManager)
        viewpager.adapter = adapter

        tab.setupWithViewPager(viewpager)
    }
}
</pre>