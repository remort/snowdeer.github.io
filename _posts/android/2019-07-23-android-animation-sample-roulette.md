---
layout: post
title: 다양한 Animation 샘플(룰렛, Roulette)
category: Android
tag: [Android]
---

## PieChartView.kt

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

<br>

## RouletteFragment.kt

<pre class="prettyprint">
package com.snowdeer.animation.sample.fragment

import android.animation.ObjectAnimator
import android.graphics.Color
import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.core.animation.doOnEnd
import androidx.fragment.app.Fragment
import com.snowdeer.animation.sample.R
import com.snowdeer.animation.sample.component.ValueItem
import kotlinx.android.synthetic.main.fragment_roulette.*
import android.view.animation.DecelerateInterpolator
import kotlinx.android.synthetic.main.fragment_roulette.view.*
import java.util.*
import kotlin.collections.ArrayList


class RouletteFragment : Fragment() {

    private var degree = 0
    private var isAnimating = false

    private val candidateList = arrayListOf(
        ValueItem("snowdeer", 1.0F, Color.parseColor("#FFDECF3F")),
        ValueItem("yang", 1.0F, Color.parseColor("#FFF17CB0")),
        ValueItem("down", 1.0F, Color.parseColor("#FF4D4D4D")),
        ValueItem("ran", 1.0F, Color.parseColor("#FFB2912F")),
        ValueItem("song", 1.0F, Color.parseColor("#FF00B200")),
        ValueItem("john", 1.0F, Color.parseColor("#FFFD4425"))
    )

    private var itemCount = 3

    override fun onCreateView(
        inflater: LayoutInflater, container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View? {

        val view = inflater.inflate(R.layout.fragment_roulette, container, false)

        view.piechart_view.setValueList(getList(itemCount))

        view.add_button.setOnClickListener {
            itemCount++
            if (itemCount >= candidateList.size) {
                itemCount = candidateList.size
            }
            view.piechart_view.setValueList(getList(itemCount))
        }

        view.remove_button.setOnClickListener {
            itemCount--
            if (itemCount <= 1) {
                itemCount = 1
            }
            view.piechart_view.setValueList(getList(itemCount))
        }

        view.rotate_button.setOnClickListener {
            rotate()
        }

        return view
    }

    private fun getList(count: Int): ArrayList&lt;ValueItem&gt; {
        val list = ArrayList&lt;ValueItem&gt;()
        for (i in 0 until count) {
            list.add(this.candidateList[i])
        }
        return list
    }


    private fun rotate() {
        val random= Random()
        if (!isAnimating) {
            isAnimating = true

            val targetDegree = degree + random.nextInt(360) * (random.nextInt(7) + 7)
            val rotateAnimator = ObjectAnimator.ofFloat(piechart_view,
                "rotation", degree.toFloat(), targetDegree.toFloat())
            rotateAnimator.interpolator = DecelerateInterpolator()
            rotateAnimator.duration = 3000
            rotateAnimator.doOnEnd {
                degree = targetDegree
                isAnimating = false
            }
            rotateAnimator.start()
        }
    }
}
</pre>
