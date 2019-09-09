---
layout: post
title: Canvas에 Arc 그리기
category: Android
tag: [Android]
---

<pre class="prettyprint">
package com.snowdeer.animation.sample.component

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.View

private const val THICKNESS = 20F


class CircleIndicatorView(context: Context?, attrs: AttributeSet?) : View(context, attrs) {

    var min = 0
    var max = 1
    var progress = 0

    private val paint = Paint()
    private val erasePaint = Paint()

    init {
        paint.apply {
            color = Color.WHITE
            isAntiAlias = true
            style = Paint.Style.STROKE
            strokeJoin = Paint.Join.ROUND
            strokeCap = Paint.Cap.ROUND
            strokeWidth = THICKNESS
        }
    }

    override fun onDraw(canvas: Canvas?) {
        val centerX = measuredWidth / 2
        val centerY = measuredHeight / 2

        val rectF = RectF(0F, 0F, measuredWidth.toFloat(), measuredHeight.toFloat())

        val path = Path()
        path.arcTo(rectF, 103F, 2F)
        canvas?.drawPath(path, paint)
    }
}
</pre>
