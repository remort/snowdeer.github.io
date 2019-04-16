---
layout: post
title: Kotlin RecyclerView 예제
category: Android, Kotlin
tag: [Android, Kotlin]
---

## build.gradle

<pre class="prettyprint">
dependencies {
    implementation 'com.android.support:design:28.0.0'
    implementation 'com.android.support:recyclerview-v7:28.0.0'
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
import android.support.v7.widget.RecyclerView
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import kotlinx.android.synthetic.main.item_log.view.*
import java.text.SimpleDateFormat
import java.util.*

data class LogItem(val text: String, val timestamp: String)

class LogListAdapter(val ctx: Context) : RecyclerView.Adapter&lt;ViewHolder&gt;() {

    private val list: ArrayList&lt;LogItem&gt; = ArrayList()

    fun addLog(message: String) {
        val c = Calendar.getInstance()
        val date = Date(c.timeInMillis)
        val sdf = SimpleDateFormat("yyyy/MM/dd hh:mm:ss", Locale.getDefault())

        list.add(LogItem(message, sdf.format(date)))
        notifyDataSetChanged()
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        val view: View = LayoutInflater.from(ctx).inflate(R.layout.item_log, parent, false)
        return ViewHolder(view)
    }

    override fun getItemCount(): Int {
        return list.size
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        holder?.message?.text = list.get(position)?.text
        holder?.timestamp?.text = list.get(position)?.timestamp
    }
}

class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {
    val message = view.message
    val timestamp = view.timestamp
}
</pre>

<br>

## MainActivity.kt

<pre class="prettyprint">
import android.support.v7.app.AppCompatActivity
import android.os.Bundle
import android.support.v7.widget.LinearLayoutManager
import android.util.Log
import kotlinx.android.synthetic.main.activity_main.*

class MainActivity : AppCompatActivity() {

    var i: Int = 0

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val adapter = LogListAdapter(applicationContext)
        recycler_view.layoutManager = LinearLayoutManager(this)
        recycler_view.adapter = adapter

        add_log_button.setOnClickListener {
            i++
            Log.i("snowdeer", "[snowdeer] Add log button is clicked!!")
            adapter.addLog("Hello $i")
        }
    }
}
</pre>