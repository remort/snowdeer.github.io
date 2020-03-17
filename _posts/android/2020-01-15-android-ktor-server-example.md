---
layout: post
title: Android에서 Ktor을 이용한 웹 서버 구현하기
category: Android
tag: [Android, Kotlin]
---

# Ktor 라이브러리

`ktor` 라이브러리를 사용하기 위해서 먼저 `build.gradle`에 다음과 같은 설정을 해줍니다.

<br>

## build.gradle(프로젝트)

`ktor` 라이브러리 버전을 너무 최신으로 했더니 일부 호환되지 않는 라이브러리들이 있어서 여기서는 버전을 `1.2.5`로 지정했습니다.

<pre class="prettyprint">
buildscript {
    ext.kotlin_version = '1.3.50'
    ext.ktor_version = '1.2.5'

    repositories {
        google()
        jcenter()
    }
    dependencies {
        classpath 'com.android.tools.build:gradle:3.5.0'
        classpath "org.jetbrains.kotlin:kotlin-gradle-plugin:$kotlin_version"
    }
}

allprojects {
    repositories {
        google()
        jcenter()
        maven { url "https://dl.bintray.com/kotlin/ktor" }
    }
}

task clean(type: Delete) {
    delete rootProject.buildDir
}
</pre>

<br>

## build.gradle(모듈)

<pre class="prettyprint">
apply plugin: 'com.android.application'

apply plugin: 'kotlin-android'

apply plugin: 'kotlin-android-extensions'

android {
    compileSdkVersion 29
    buildToolsVersion "29.0.2"
    defaultConfig {
        applicationId "com.snowdeer.webserver"
        minSdkVersion 27
        targetSdkVersion 29
        versionCode 1
        versionName "1.0"
        testInstrumentationRunner "androidx.test.runner.AndroidJUnitRunner"
    }
    buildTypes {
        release {
            minifyEnabled false
            proguardFiles getDefaultProguardFile('proguard-android-optimize.txt'), 'proguard-rules.pro'
        }
    }

    packagingOptions {
        exclude 'META-INF/*'
    }
}

dependencies {
    implementation fileTree(dir: 'libs', include: ['*.jar'])
    implementation "org.jetbrains.kotlin:kotlin-stdlib-jdk7:$kotlin_version"

    implementation "io.ktor:ktor-server-netty:1.2.5"
    implementation "ch.qos.logback:logback-classic:1.2.3"

    implementation 'androidx.appcompat:appcompat:1.0.2'
    implementation 'androidx.core:core-ktx:1.0.2'
    implementation 'androidx.constraintlayout:constraintlayout:1.1.3'
    testImplementation 'junit:junit:4.12'

    androidTestImplementation 'androidx.test:runner:1.1.1'
    androidTestImplementation 'androidx.test.espresso:espresso-core:3.1.1'
}
</pre>

<br>

## Main.kt

<pre class="prettyprint">
package com.snowdeer.webserver

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.os.Environment
import android.os.Environment.DIRECTORY_DOCUMENTS
import android.util.Log
import android.widget.Toast
import androidx.appcompat.app.AppCompatActivity


class MainActivity : AppCompatActivity() {

    companion object {
        private const val PERMISSION_REQUEST_CODE = 100
    }

    private val assetInstaller = AssetInstaller()
    private val router = Router()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        Log.i("snowdeer", "[snowdeer] onCreate()")

        requestPermission()

    }

    private fun requestPermission() {
        if (checkSelfPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
            val permissions = arrayOf(Manifest.permission.WRITE_EXTERNAL_STORAGE)
            requestPermissions(permissions, PERMISSION_REQUEST_CODE)
        } else {
            start()
        }
    }

    override fun onRequestPermissionsResult(
        requestCode: Int, permissions: Array&lt;String&gt;,
        grantResults: IntArray
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)

        when (requestCode) {
            PERMISSION_REQUEST_CODE -> if (grantResults.isNotEmpty() && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
                Toast.makeText(applicationContext, "Permission 완료", Toast.LENGTH_SHORT).show()
                start()
            } else {
                Toast.makeText(applicationContext, "Permission 실패", Toast.LENGTH_SHORT).show()
            }
        }
    }

    private fun start() {
        val parent = Environment.getExternalStoragePublicDirectory(DIRECTORY_DOCUMENTS)

        val staticContentDirectory = "${parent.absolutePath}/static"
        assetInstaller.install(this, "static", staticContentDirectory)

        router.startServer(this, staticContentDirectory, "/static")
    }


}
</pre>

<br>

## AssetInstaller.kt

`assets` 디렉토리에 있는 파일을 단말내 저장소에 복사하는 용도의 클래스입니다.

<pre class="prettyprint">
package com.snowdeer.webserver

import android.content.Context
import android.content.res.AssetManager
import android.util.Log
import java.io.File
import java.io.FileOutputStream

class AssetInstaller {

    fun install(ctx: Context, fromDirectory: String, targetDirectory: String) {
        ctx?.assets?.let {
            recursiveCopy(it, fromDirectory, targetDirectory)
        }
    }

    private fun recursiveCopy(am: AssetManager, src: String, target: String) {
        val files = am.list(src)
        files?.let {
            for (filename in it) {
                val filepath = "$src/$filename"
                val targetPath = "$target/$filename"

                if ((am.list(filepath) == null) || (am.list(filepath)?.size == 0)) {
                    Log.i("snowdeer", "[snowdeer] copy $filepath to $targetPath")
                    copyFile(am, filepath, targetPath)

                } else {
                    Log.i("snowdeer", "[snowdeer] $filepath is a directory.")
                    createDirectory(targetPath)

                    recursiveCopy(am, filepath, targetPath)
                }
            }
        }
    }

    private fun createDirectory(path: String) {
        val directory = File(path)
        if (!directory.exists()) {
            directory.mkdirs()
        }
    }

    private fun copyFile(am: AssetManager, src: String, target: String) {
        val file = File(target)
        if (file.exists()) {
            file.delete()
        }

        val inputStream = am.open(src)
        val outputStream = FileOutputStream(target)

        var read: Int
        val buffer = ByteArray(1024)

        while (true) {
            read = inputStream?.read(buffer)
            if (read == -1) {
                break
            }
            outputStream.write(buffer, 0, read)
        }

        inputStream?.close()
        outputStream.flush()
        outputStream.close()
    }
}
</pre>

<br>

## Router.kt

<pre class="prettyprint">
package com.snowdeer.webserver

import android.content.Context
import android.util.Log
import android.widget.Toast
import io.ktor.application.Application
import io.ktor.application.call
import io.ktor.application.install
import io.ktor.features.CORS
import io.ktor.http.HttpMethod
import io.ktor.response.respondFile
import io.ktor.routing.Routing
import io.ktor.routing.get
import io.ktor.routing.routing
import io.ktor.server.engine.embeddedServer
import io.ktor.server.netty.Netty
import java.io.File

class Router {

    fun startServer(ctx: Context, staticContentDirectory: String, staticContentURL: String) {
        Log.i("snowdeer", "[snowdeer] startServer()")
        Toast.makeText(ctx, "startServer()", Toast.LENGTH_SHORT).show()

        val server = embeddedServer(Netty, port = 8080) {
            installCors(this)

            routing {
                staticContentDirectory?.let {
                    addStaticContentRoute(this, staticContentDirectory, staticContentDirectory, staticContentURL)
                }

                get("/react") {
                    Log.i("snowdeer", "[snowdeer] /react is called.")
                    val file = File("${staticContentDirectory}/web/react.html")
                    call.respondFile(file)
                }

                get("/tetris") {
                    val file = File("${staticContentDirectory}/web/tetris.html")
                    call.respondFile(file)
                }

                get("/maker") {
                    val file = File("${staticContentDirectory}/web/bt_maker.html")
                    call.respondFile(file)
                }

                get("/tree") {
                    val file = File("${staticContentDirectory}/tree/MoveToPoint")
                    call.respondFile(file)
                }
            }
        }
        server.start()
    }

    private fun installCors(server:Application) {
        server.install(CORS) {
            method(HttpMethod.Options)
            method(HttpMethod.Get)
            method(HttpMethod.Post)
            method(HttpMethod.Put)
            method(HttpMethod.Delete)
            method(HttpMethod.Patch)
            anyHost()
        }
    }

    private fun addStaticContentRoute(routing: Routing, path: String, parentPath: String, url: String) {
        val directory = File(path)
        val files = directory.listFiles()

        for (f in files) {
            if (f.isDirectory) {
                val routeUrl = url + f.absolutePath.substring(parentPath.length) + "/{filename}"
                Log.i("snowdeer", "[snowdeer] routeUrl($routeUrl)")

                routing.get(routeUrl) {
                    val fileName = call.parameters["filename"]
                    val file = File("${f.absolutePath}/$fileName")
                    call.respondFile(file)
                }

                addStaticContentRoute(routing, f.absolutePath, parentPath, url)
            }
        }

    }
}
</pre>