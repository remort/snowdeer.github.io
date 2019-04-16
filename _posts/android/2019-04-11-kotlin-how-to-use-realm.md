---
layout: post
title: Kotlin Realim 사용하는 방법
category: Android
tag: [Android, Kotlin]
---

## build.gradle(프로젝트)

<pre class="prettyprint">
buildscript {
    ext.kotlin_version = '1.3.21'
    ext.anko_version = '0.10.8'
    repositories {
        google()
        jcenter()

    }
    dependencies {
        classpath 'com.android.tools.build:gradle:3.3.2'
        classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlin_version"
        classpath "io.realm:realm-gradle-plugin:5.10.0"
    }
}
</pre>

<br>

## build.gradle(모듈)

<pre class="prettyprint">
apply plugin: 'com.android.application'

apply plugin: 'kotlin-android'

apply plugin: 'kotlin-android-extensions'

apply plugin: 'kotlin-kapt'

apply plugin: 'realm-android'
</pre>

<br>

## Realm 객체 생성하기

<pre class="prettyprint">
import io.realm.RealmObject
import io.realm.annotations.PrimaryKey

open class Todo(@PrimaryKey var id: Long = 0,
                var title: String = "",
                var date: Long = 0) : RealmObject() {

}
</pre>

<br>

## SnowApplication.kt

<pre class="prettyprint">
import android.app.Application
import io.realm.Realm

class SnowApplication : Application() {
    override fun onCreate() {
        super.onCreate()
        Realm.init(this)
    }
}
</pre>

그리고 `AndroidManifest.xml`에 위 Application을 등록합니다.

<br>

## MainActivity.kt

<pre class="prettyprint">
import android.support.v7.app.AppCompatActivity
import android.os.Bundle
import android.util.Log
import io.realm.Realm
import io.realm.Sort
import io.realm.kotlin.createObject
import io.realm.kotlin.where
import kotlinx.android.synthetic.main.activity_main.*
import org.jetbrains.anko.alert
import org.jetbrains.anko.yesButton
import java.util.*

class MainActivity : AppCompatActivity() {

    private val realm = Realm.getDefaultInstance()
    private val calendar = Calendar.getInstance()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        insert_button.setOnClickListener { insertItem() }
        select_button.setOnClickListener { selectItemList() }
        update_button.setOnClickListener { updateItem(1) }
        delete_button.setOnClickListener { deleteItem(1) }
    }

    override fun onDestroy() {
        super.onDestroy()
        realm.close()
    }

    private fun nextId(): Long {
        val maxId = realm.where&lt;Todo&gt;().max("id")
        if (maxId != null) {
            return maxId.toLong() + 1
        }
        return 0
    }

    private fun insertItem() {
        realm.beginTransaction()

        val newItem = realm.createObject&lt;Todo&gt;(nextId())
        newItem.title = "Title"
        newItem.date = calendar.timeInMillis

        realm.commitTransaction()

        alert("An item is added.") {
            yesButton { }
        }.show()
    }

    private fun selectItemList() {
        val realmResult = realm.where&lt;Todo&gt;().findAll().sort("date", Sort.DESCENDING)

        for (Todo in realmResult) {
            Log.i("snowdeer", "[snowdeer] todo: $Todo")
        }
    }

    private fun updateItem(id: Long) {
        realm.beginTransaction()

        val newItem = realm.where&lt;Todo&gt;().equalTo("id", id).findFirst()!!
        newItem.title = "Title is changed !!!"
        newItem.date = calendar.timeInMillis

        realm.commitTransaction()

        alert("An item($id) is changed.") {
            yesButton { }
        }.show()
    }

    private fun deleteItem(id: Long) {
        realm.beginTransaction()

        val item = realm.where&lt;Todo&gt;().equalTo("id", id).findFirst()!!
        item.deleteFromRealm()

        realm.commitTransaction()

        alert("An item($id) is deleted.") {
            yesButton { }
        }.show()
    }
}
</pre>