---
layout: post
title: 코루틴(Coroutine) Pause/Resume 사용하기
category: Kotlin
tag: [Android, Kotlin]
---

# SAM 변환

<pre class="prettyprint">
import android.os.Bundle
import android.os.Handler
import android.util.Log
import androidx.appcompat.app.AppCompatActivity
import kotlinx.android.synthetic.main.activity_main.*
import kotlinx.coroutines.*
import kotlin.coroutines.Continuation
import kotlin.coroutines.resume

class MainActivity : AppCompatActivity() {

    private var continuation: Continuation&lt;String&gt;? = null

    private var count = 0

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        Log.i("snowdeer", "[snowdeer] onCreate()")

        run_button.setOnClickListener {
            startCoroutine()
        }
        next_button.setOnClickListener {
            continuation?.resume("Resumed")
        }

    }

    private fun startThread() {

    }

    private fun startCoroutine() {
        Log.i("snowdeer", "[snowdeer] startCoroutine()")

        GlobalScope.launch {
            for(i in 0 until 5) {
                refreshCount()
                Thread.sleep(500L)
            }

            pause()

            for(i in 0 until 5) {
                refreshCount()
                Thread.sleep(500L)
            }

            pause()

            for(i in 0 until 5) {
                refreshCount()
                Thread.sleep(500L)
            }

            pause()

            for(i in 0 until 5) {
                refreshCount()
                Thread.sleep(500L)
            }

            pause()

            for(i in 0 until 5) {
                refreshCount()
                Thread.sleep(500L)
            }

            pause()
        }

        Log.i("snowdeer", "[snowdeer] end of startCoroutine()")
    }

    private fun refreshCount() {
        count++
        count_textview.text = "$count"
    }

    private suspend fun pause() = suspendCancellableCoroutine&lt;String&gt; {
        continuation = it
    }
}
</pre>
