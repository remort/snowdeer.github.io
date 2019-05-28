---
layout: post
title: Realm 이용한 Database 사용 예제
category: Android
tag: [Android]
---

## 프로젝트 build.gradle

<pre class="prettyprint">
buildscript {
    ext.kotlin_version = '1.3.31'
    repositories {
        google()
        jcenter()

    }
    dependencies {
        classpath 'com.android.tools.build:gradle:3.4.1'
        classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlin_version"
        classpath "io.realm:realm-gradle-plugin:5.2.0"
    }
}
</pre>

<br>

## 모듈 build.gradle

<pre class="prettyprint">
apply plugin: 'com.android.application'
apply plugin: 'kotlin-android'
apply plugin: 'kotlin-android-extensions'
apply plugin: 'kotlin-kapt'
apply plugin: 'realm-android'

...

dependencies {
    ...
    implementation 'io.realm:android-adapters:2.1.1'
}
</pre>

<br>

## SnowApplication.kt

<pre class="prettyprint">
import android.app.Application
import android.content.Context
import io.realm.Realm
import io.realm.RealmConfiguration

class SnowApplication : Application() {

    companion object {
        private var instance: SnowApplication? = null

        fun context(): Context {
            return instance!!.applicationContext
        }
    }

    init {
        instance = this
    }

    override fun onCreate() {
        super.onCreate()

        Realm.init(this)
        Realm.setDefaultConfiguration(getRealmConfig())
    }

    private fun getRealmConfig(): RealmConfiguration {
        return RealmConfiguration.Builder()
                .deleteRealmIfMigrationNeeded()
                .build()
    }
}
</pre>

<br>

## 데이터 레코드 정의

`open` 키워드를 이용해서 상속을 가능하게 해야 하며, `RealmObject()`를 상속받아야 합니다.

<pre class="prettyprint">
import android.os.Parcel
import android.os.Parcelable
import io.realm.RealmObject
import io.realm.annotations.PrimaryKey

open class RewardDto(
        var habitId: Long = 0,
        var year: Int = 0, var month: Int = 0, var day: Int = 0,
        var percent: Int = 0, var stampType: Int = 0) : RealmObject()

open class HabitDto(
        @PrimaryKey var id: Long = 0,
        var name: String = "", var seq: Long = 0) : RealmObject()
</pre>

<br>

## 데이터 생성, 조회, 수정, 삭제

<pre class="prettyprint">
import com.snowdeer.htracker.model.data.HabitDto
import io.realm.Realm
import io.realm.Sort
import io.realm.kotlin.createObject
import io.realm.kotlin.where

interface OnDatabaseEventListener {
    fun onDatabaseUpdated()
}

class HabitModelManager {

    companion object {
        val instance = HabitModelManager()
    }

    private constructor()

    private val realm = Realm.getDefaultInstance()

    private var onDatabaseEventListener: OnDatabaseEventListener? = null

    fun setOnDatabaseEventListener(l: OnDatabaseEventListener) {
        onDatabaseEventListener = l
    }

    fun notifyDatabaseUpdated() {
        onDatabaseEventListener?.onDatabaseUpdated()
    }

    private fun nextId(): Long {
        val maxId = realm.where&lt;HabitDto&gt;().max("id")
        if (maxId != null) {
            return maxId.toLong() + 1
        }
        return 0
    }

    private fun nextSeq(): Long {
        val maxSeq = realm.where&lt;HabitDto&gt;().max("seq")
        return maxSeq?.toLong()?.plus(1) ?: 0
    }


    fun getList(): ArrayList&lt;HabitDto&gt; {
        val realmResult = realm.where&lt;HabitDto&gt;()
                .findAll()
                .sort("seq", Sort.DESCENDING)

        val list = ArrayList&lt;HabitDto&gt;()
        for (item in realmResult) {
            list.add(item)
        }

        return list
    }

    fun create(name: String) {
        realm.beginTransaction()

        val item = realm.createObject&lt;HabitDto&gt;(nextId())

        item.name = name
        item.seq = nextSeq()

        realm.commitTransaction()

        notifyDatabaseUpdated()
    }

    fun update(id: Long, name: String, seq: Long) {
        realm.beginTransaction()

        val item = realm.where&lt;HabitDto&gt;().equalTo("id", id).findFirst()!!
        item.name = name
        item.seq = seq

        realm.commitTransaction()

        notifyDatabaseUpdated()
    }

    fun deleteWithReward(id: Long) {
        realm.beginTransaction()

        val item = realm.where&lt;HabitDto&gt;().equalTo("id", id).findFirst()!!
        item.deleteFromRealm()

        realm.commitTransaction()

        notifyDatabaseUpdated()
    }
}
</pre>