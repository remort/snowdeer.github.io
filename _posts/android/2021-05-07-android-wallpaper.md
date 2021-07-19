---
layout: post
title: 간단한 Wallpaper 만들어보기
category: Android
tag: [Android, Kotlin]
---

# Wallpaper

안드로이드에서 간단한 Wallpaper를 만들어보는 예제입니다.

<br>

## AndroidManifest.xml

먼저 다음 `Permission`을 추가해줍니다.
<pre class="prettyprint">
    &lt;uses-permission android:name="android.permission.SET_WALLPAPER" /&gt;
    &lt;uses-feature
            android:name="android.software.live_wallpaper"
            android:required="true"/&gt;
</pre>

그리고 Wallpaper는 `android.service`를 사용하기 때문에 `service`도 등록합니다.

<pre class="prettyprint">
    &lt;service
        android:name=".service.snowdeer.SimpleWallpaperService"
        android:enabled="true"
        android:label="@string/app_name"
        android:permission="android.permission.BIND_WALLPAPER"&gt;
        &lt;intent-filter&gt;
            &lt;action android:name="android.service.wallpaper.WallpaperService" /&gt;
        &lt;/intent-filter&gt;

        &lt;meta-data
            android:name="android.service.wallpaper"
            android:resource="@xml/wallpaper" /&gt;
    &lt;/service&gt;
</pre>

<br>

## BaseWallpaperService.kt

<pre class="prettyprint">
import android.content.Context
import android.service.wallpaper.WallpaperService
import android.view.MotionEvent
import android.view.SurfaceHolder
import android.view.WindowManager

interface InteractiveEngine {
    fun setScreenSize(width: Int, height: Int)
    fun onSurfaceCreated(holder: SurfaceHolder)
    fun onDestory()
}

open abstract class BaseWallpaperService : WallpaperService() {

    abstract fun handleScreenSize(width: Int, height: Int)
    abstract fun handleSurfaceCreated(holder: SurfaceHolder)
    open fun handleTouchEvent(event: MotionEvent) {}

    override fun onCreate() {
        super.onCreate()

        val window = getSystemService(Context.WINDOW_SERVICE) as WindowManager
        val display = window.defaultDisplay
        val width = display.width
        val height = display.height

        handleScreenSize(width, height)
    }

    override fun onCreateEngine(): Engine {
        return WallpaperEngine()
    }

    inner class WallpaperEngine : WallpaperService.Engine() {
        override fun onSurfaceCreated(holder: SurfaceHolder) {
            handleSurfaceCreated(holder)
        }

        override fun onTouchEvent(event: MotionEvent?) {
            event?.let {
                handleTouchEvent(event)
            }
        }
    }
}
</pre>

<br>

## SimpleWallpaper.kt

<pre class="prettyprint">
import android.view.MotionEvent
import android.view.SurfaceHolder
import com.snowdeer.wallpaper.service.BaseWallpaperService

class SimpleWallpaper : BaseWallpaperService() {
    val engine: InteractiveEngine = RandomColorEngine()
    
    companion object {
        const val TAG = "NormalWallpaperService"
    }

    override fun handleScreenSize(width: Int, height: Int) {
        engine.setScreenSize(width, height)
    }

    override fun handleSurfaceCreated(holder: SurfaceHolder) {
        engine.onSurfaceCreated(holder)
    }

    override fun handleTouchEvent(event: MotionEvent) {
        engine.handleTouchEvent(event)
    }

    override fun onDestroy() {
        super.onDestroy()
        engine.onDestory()
    }
}
</pre>

<br>

## RandomColorEngine.kt

<pre class="prettyprint">
import android.graphics.Color
import android.graphics.Paint
import android.os.Handler
import android.os.Looper
import android.view.SurfaceHolder
import com.snowdeer.service.InteractiveEngine
import kotlin.random.Random

class RandomColorEngine : InteractiveEngine {
    private val handler = Handler(Looper.getMainLooper())
    private var isRunning = false

    override fun setScreenSize(width: Int, height: Int) {

    }

    override fun onSurfaceCreated(holder: SurfaceHolder) {
        isRunning = true

        drawCanvas(holder)
    }

    override fun onDestory() {
        isRunning = false
    }

    private fun drawCanvas(holder: SurfaceHolder) {
        if (!isRunning) {
            return
        }

        try {
            val canvas = holder.lockCanvas()

            canvas?.let {
                val paint = Paint().apply {
                    val randomColor = Random.nextInt(16_777_216)
                        .toString(16)
                        .padStart(6, '0')
                    color = Color.parseColor("#$randomColor")
                    style = Paint.Style.FILL
                }
                canvas.drawPaint(paint)
                holder.unlockCanvasAndPost(canvas)

                handler.postDelayed({ drawCanvas(holder) }, 1000)
            }
        } catch (e: Exception) {
            e.printStackTrace()
        }

    }
}
</pre>

<br>

## WallpaperApplier.kt

<pre class="prettyprint">
import android.app.WallpaperManager
import android.content.ComponentName
import android.content.Context
import android.content.Intent

data class WallpaperListItem(val name: String, val resId: Int, val cls: Class<*>)

class WallpaperApplier(val ctx: Context) {
    fun apply(wallpaper: WallpaperListItem) {
        setWallpaperService(wallpaper.cls)
    }

    private fun setWallpaperService(wallpaperService: Class<*>) {
        val intent = Intent(WallpaperManager.ACTION_CHANGE_LIVE_WALLPAPER)
        intent.putExtra(
            WallpaperManager.EXTRA_LIVE_WALLPAPER_COMPONENT,
            ComponentName(ctx, wallpaperService)
        )
        intent.flags = Intent.FLAG_ACTIVITY_NEW_TASK
        ctx.startActivity(intent)
    }
}
</pre>

