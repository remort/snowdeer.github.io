---
layout: post
title: Scale 및 Gesture Handler
category: Android
tag: [Android]
---

화면의 TouchEvent에서 멀티 터치를 이용한 Scale이나 Gesture를 인식하는 Handler 코드 예제입니다.

<pre class="prettyprint">
package com.snowdeer.utils

import android.content.Context
import android.view.GestureDetector
import android.view.MotionEvent
import android.view.ScaleGestureDetector

interface OnScaleGestureEventListener {
    fun onSingleTab(x: Float, y: Float)
    fun onDoubleTab(x: Float, y: Float)
    fun onLongPress(x: Float, y: Float)
    fun onScaleChanged(scaleFactor: Float)
    fun onOffsetChanged(offsetX: Float, offsetY: Float)
}

class ScaleGestureHandler(ctx: Context) {

    private var scaleFactor = 1.0F
    private var offsetX = 0.0F
    private var offsetY = 0.0F
    private var scaledOffsetX = 0.0F
    private var scaledOffsetY = 0.0F
    private var focusX = 0.0F
    private var focusY = 0.0F

    private val scaleGestureDetector: ScaleGestureDetector
    private val gestureDetector: GestureDetector
    var onScaleGestureEventListener: OnScaleGestureEventListener? = null

    init {
        scaleGestureDetector = ScaleGestureDetector(ctx, ScaleListener())
        gestureDetector = GestureDetector(ctx, GestureListener())
    }

    fun handleTouchEvent(event: MotionEvent): Boolean {
        scaleGestureDetector.onTouchEvent(event)

        if (gestureDetector.onTouchEvent(event)) {
            return true
        }

        return false
    }

    fun reset() {
        scaleFactor = 1.0F
        offsetX = 0F
        offsetY = 0F
        scaledOffsetX = 0F
        scaledOffsetY = 0F
        focusX = 0.0F
        focusY = 0.0F
    }

    private inner class ScaleListener : ScaleGestureDetector.SimpleOnScaleGestureListener() {
        override fun onScale(detector: ScaleGestureDetector): Boolean {
            scaleFactor *= detector.scaleFactor
            scaleFactor = Math.max(0.8f, Math.min(scaleFactor, 3.0f))

            focusX = detector.focusX
            focusY = detector.focusY

            onScaleGestureEventListener?.onScaleChanged(scaleFactor)

            offsetX = (scaledOffsetX - focusX) * scaleFactor + focusX
            offsetY = (scaledOffsetY - focusY) * scaleFactor + focusY

            onScaleGestureEventListener?.onOffsetChanged(offsetX, offsetY)

            return true
        }
    }

    private inner class GestureListener : GestureDetector.SimpleOnGestureListener() {
        override fun onSingleTapConfirmed(e: MotionEvent): Boolean {
            onScaleGestureEventListener?.onSingleTab(e.x, e.y)
            return true
        }

        override fun onDoubleTap(e: MotionEvent): Boolean {
            onScaleGestureEventListener?.onDoubleTab(e.x, e.y)
            return true
        }

        override fun onLongPress(e: MotionEvent) {
            onScaleGestureEventListener?.onLongPress(e.x, e.y)
        }

        override fun onScroll(e1: MotionEvent, e2: MotionEvent, distanceX: Float, distanceY: Float): Boolean {
            scaledOffsetX -= distanceX / scaleFactor
            scaledOffsetY -= distanceY / scaleFactor

            offsetX = (scaledOffsetX - focusX) * scaleFactor + focusX
            offsetY = (scaledOffsetY - focusY) * scaleFactor + focusY

            onScaleGestureEventListener?.onOffsetChanged(offsetX, offsetY)

            return true
        }
    }
}
</pre>