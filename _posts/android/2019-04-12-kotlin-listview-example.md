---
layout: post
title: Kotlin ListView 예제
category: Android, Kotlin
tag: [Android, Kotlin]
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

  &lt;Button
    android:id="@+id/add_log_button"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="Add Log"/&gt;

  &lt;android.support.v7.widget.RecyclerView
    android:id="@+id/recycler_view"
    android:layout_width="match_parent"
    android:layout_height="match_parent"/&gt;

&lt;/LinearLayout&gt;
</pre>

<br>

## item_log.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;android.support.v7.widget.CardView
  xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:app="http://schemas.android.com/apk/res-auto"
  android:layout_width="match_parent"
  android:layout_height="wrap_content"
  android:layout_margin="4dp"
  app:cardCornerRadius="4dp"
  app:cardElevation="8dp"&gt;

  &lt;LinearLayout
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    android:paddingTop="8dp"
    android:paddingBottom="8dp"
    android:paddingStart="8dp"
    android:paddingEnd="8dp"
    android:orientation="vertical"&gt;

    &lt;TextView
      android:id="@+id/message"
      android:layout_width="match_parent"
      android:layout_height="wrap_content"
      android:paddingLeft="8dp"
      android:text="message"
      android:textColor="@android:color/holo_blue_dark"
      android:textSize="14sp"/&gt;

    &lt;TextView
      android:id="@+id/timestamp"
      android:layout_width="match_parent"
      android:layout_height="wrap_content"
      android:gravity="right"
      android:text="timestamp"
      android:textColor="@android:color/darker_gray"
      android:textSize="10sp"/&gt;

  &lt;/LinearLayout&gt;

&lt;/android.support.v7.widget.CardView&gt;
</pre>

<br>

## LogListAdapter.kt

<pre class="prettyprint">
import android.content.Context
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.BaseAdapter
import android.widget.TextView

data class LogItem(val text: String, val timestamp: String)

class LogListAdapter(val ctx: Context) : BaseAdapter() {

    private val list = ArrayList&lt;LogItem&gt;()

    init {
        list.add(LogItem("hello", "2019/04/15 13:00"))
        list.add(LogItem("Nice to meet you", "2019/04/15 14:00"))
        list.add(LogItem("Good bye", "2019/04/15 17:00"))
    }

    override fun getView(position: Int, convertView: View?, parent: ViewGroup?): View {
        var view: View
        val holder: ViewHolder

        if (convertView == null) {
            view = LayoutInflater.from(ctx).inflate(R.layout.item_log, parent, false)
            holder = ViewHolder()
            holder.message = view.findViewById(R.id.message)
            holder.timestamp = view.findViewById(R.id.timestamp)
            view.tag = holder

        } else {
            view = convertView
            holder = convertView.tag as ViewHolder
        }

        val item = list[position]
        holder.message.text = item.text
        holder.timestamp.text = item.timestamp

        return view
    }

    override fun getItem(position: Int): LogItem {
        return list[position]
    }

    override fun getItemId(position: Int): Long {
        return position.toLong()
    }

    override fun getCount(): Int {
        return list.size
    }
}

private class ViewHolder {
    lateinit var message: TextView
    lateinit var timestamp: TextView
}
</pre>

<br>

## MainActivity.kt

<pre class="prettyprint">
import android.support.v7.app.AppCompatActivity
import android.os.Bundle
import kotlinx.android.synthetic.main.activity_main.*

class MainActivity : AppCompatActivity() {

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val adapter = LogListAdapter(applicationContext)
        listview.adapter = adapter
    }
}
</pre>
