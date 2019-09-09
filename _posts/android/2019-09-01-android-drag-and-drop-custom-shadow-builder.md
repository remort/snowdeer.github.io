---
layout: post
title: Drag and Drop 시 Custom ShadowBuilder 사용하기
category: Android
tag: [Android]
---

안드로이드에서 드래그&드랍(Drag & Drop)시 보여주는 반투명 이미지는 `View.ShadowBuilder` 클래스를 통해서 만들 수 있습니다.
자동으로 해당 `View`에서 반투명 이미지를 생성해주는데, 만약 개발자가 원하는 특정 이미지가 있다면 다음 코드를 이용해서
해당 이미지를 드래그시 사용할 수 있습니다.

<pre class="prettyprint">
import android.content.Context
import android.graphics.Bitmap
import android.graphics.Canvas
import android.graphics.Point
import android.graphics.drawable.BitmapDrawable
import android.graphics.drawable.Drawable
import android.view.View

class ImageShadowBuilder : View.DragShadowBuilder() {

    private var shadow: Drawable? = null

    companion object {

        fun fromResource(ctx: Context, resId: Int): View.DragShadowBuilder {
            val builder = ImageShadowBuilder()
            builder.shadow = ctx.resources.getDrawable(resId)

            builder.shadow?.let {
                it.setBounds(0, 0, it.minimumWidth, it.minimumHeight)
            }

            return builder
        }

        fun fromBitmap(ctx: Context, bm: Bitmap): View.DragShadowBuilder {

            val builder = ImageShadowBuilder()
            builder.shadow = BitmapDrawable(ctx.resources, bm)

            builder.shadow?.let {
                it.setBounds(0, 0, it.minimumWidth, it.minimumHeight)
            }

            return builder
        }

    }

    override fun onDrawShadow(canvas: Canvas?) {
        shadow?.draw(canvas)
    }

    override fun onProvideShadowMetrics(outShadowSize: Point?, outShadowTouchPoint: Point?) {
        outShadowSize?.x = shadow?.minimumWidth
        outShadowSize?.y = shadow?.minimumHeight

        outShadowTouchPoint?.x = (outShadowSize?.x ?: 0 / 2)
        outShadowTouchPoint?.y = (outShadowSize?.y ?: 0 / 2)
    }
}
</pre>