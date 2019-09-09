---
layout: post
title: Canvas에 Pie Chart 그리기
category: Android
tag: [Android]
---

<pre class="prettyprint">
package com.snowdeer.animation.sample.component

import android.content.Context
import android.graphics.*
import android.util.AttributeSet
import android.view.View
import kotlin.math.cos
import kotlin.math.sin

data class ValueItem(var name: String, var value: Float, var color: Int)

class PieChartView(context: Context?, attrs: AttributeSet?) : View(context, attrs) {

    private val WIDTH = 800
    private val HEIGHT = 800

    private var list = ArrayList&lt;ValueItem&gt;()

    fun setValueList(list: ArrayList&lt;ValueItem&gt;) {
        this.list = list
        invalidate()
    }

    override fun onDraw(canvas: Canvas?) {
        drawSlice(canvas)
        drawText(canvas)
    }

    private fun drawSlice(canvas: Canvas?) {
        val total = getTotalSize()
        val dAngle = 360.0F / total

        val centerX = measuredWidth / 2
        val centerY = measuredHeight / 2
        val left = centerX - WIDTH / 2
        val top = centerY - HEIGHT / 2
        val right = centerX + WIDTH / 2
        val bottom = centerY + HEIGHT / 2

        val rectF = RectF(left.toFloat(), top.toFloat(), right.toFloat(), bottom.toFloat())

        var fromAngle = 0.0F
        for (item in list) {
            val paint = Paint()
            paint.color = item.color

            val sweepAngle = item.value * dAngle
            val drawArc = canvas?.drawArc(rectF, fromAngle, sweepAngle, true, paint)

            fromAngle += sweepAngle
        }
    }

    private fun drawText(canvas: Canvas?) {
        val total = getTotalSize()
        val dAngle = 360.0F / total

        val centerX = measuredWidth / 2
        val centerY = measuredHeight / 2
        val left = centerX - WIDTH / 2
        val top = centerY - HEIGHT / 2
        val right = centerX + WIDTH / 2
        val bottom = centerY + HEIGHT / 2

        val rectF = RectF(left.toFloat(), top.toFloat(), right.toFloat(), bottom.toFloat())
        val rect = Rect(left, top, right, bottom)

        var fromAngle = 0.0F
        for (item in list) {
            val text = item.name
            val sweepAngle = item.value * dAngle
            val angle = (fromAngle + (sweepAngle / 2.0F)) * 0.0174532925F

            val paint = Paint()
            paint.color = Color.BLACK
            paint.textSize = 40F
            paint.textAlign = Paint.Align.CENTER

            canvas?.save()

            paint.getTextBounds(text, 0, text.length, rect)
            var x = rectF.centerX() + cos(angle) * (rectF.width() / 4 + rect.width() / 2)
            val y = rectF.centerY() + sin(angle) * (rectF.height() / 4 + rect.width() / 2)

            x -= rect.width() / 2
            canvas?.rotate(
                fromAngle + (sweepAngle / 2), (x + rect.exactCenterX()),
                (y + rect.exactCenterY())
            )
            canvas?.drawText(text, x, y, paint)
            canvas?.restore()

            fromAngle += sweepAngle
        }
    }

    private fun getTotalSize(): Float {
        var sum = 0.0F

        for (item in list) {
            sum += item.value
        }

        return sum
    }
}
</pre>
