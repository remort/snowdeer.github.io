---
layout: post
title: 경로를 따라 그리는 Path Animation
category: Android
tag: [Android]
---

먼저 `View`를 상속받는 캔버스(Canvas)를 가진 클래스를 하나 구현합니다. 그리고 `setPercentage` 메소드를 만들어줍니다. 
클래스 외부에서 애니메이션을 동작시킬 때 필요한 `progress` 관련 메소드입니다.

<pre class="prettyprint">
class PathView(context: Context, attrs: AttributeSet) : View(context, attrs) {

    companion object {
        private const val CORNER_ROUND = 40F
    }

    private val wayPointList = ArrayList&lt;PointF&gt;()

    private val paint = Paint()

    private var progress = 0F
    private var pathLength = 0F

    init {
        paint.apply {
            color = context.getColor(R.color.path_color)
            style = Paint.Style.STROKE
            isAntiAlias = true
            strokeWidth = 8.0F
            strokeCap = Paint.Cap.ROUND
            strokeJoin = Paint.Join.ROUND
        }

        initDummyData()
    }

    private fun initDummyData() {
        wayPointList.add(PointF(190F, 1715F))
        wayPointList.add(PointF(270F, 1715F))
        wayPointList.add(PointF(270F, 650F))
        wayPointList.add(PointF(460F, 650F))
        wayPointList.add(PointF(460F, 500F))
    }

    fun setPath(pointList: ArrayList&lt;Point&gt;) {
        wayPointList.clear()

        for (p in pointList) {
            val x = MapCoordinateConverter.getCanvasXFromWorldX(p.x).toFloat()
            val y = MapCoordinateConverter.getCanvasXFromWorldX(p.y).toFloat()

            wayPointList.add(PointF(x, y))
        }

        invalidate()
    }

    fun setPercentage(percentage: Float) {
        if (percentage < 0.0f || percentage > 1.0f) {
            throw IllegalArgumentException("setPercentage not between 0.0f and 1.0f")
        }

        progress = percentage
        invalidate()
    }

    override fun onDraw(canvas: Canvas?) {
        super.onDraw(canvas)

        drawPath(canvas)
    }

    private fun drawPath(canvas: Canvas?) {
        val p = createPath()

        val measure = PathMeasure(p, false)
        pathLength = measure.length

        val total = pathLength - pathLength * progress
        val pathEffect = DashPathEffect(floatArrayOf(pathLength, pathLength), total)

        val cornerPathEffect = CornerPathEffect(CORNER_ROUND)
        paint.pathEffect = ComposePathEffect(cornerPathEffect, pathEffect)

        canvas?.drawPath(p, paint)
    }

    private fun createPath(): Path {
        val p = Path()

        if(wayPointList.size > 0) {
            p.moveTo(wayPointList[0].x, wayPointList[0].y)
            for (pf in wayPointList) {
                p.lineTo(pf.x, pf.y)
            }
        }

        return p
    }

}
</pre>

위에서 만든 클래스를 외부에서 사용할 때는 다음과 같이 작성하면 됩니다.

<pre class="prettyprint">
class MapView(context: Context, attrs: AttributeSet) : FrameLayout(context, attrs) {

    private val pathView = PathView(context, attrs)

    init {

        val layoutParams = ViewGroup.LayoutParams(ViewGroup.LayoutParams.MATCH_PARENT,
                ViewGroup.LayoutParams.MATCH_PARENT)

        addView(pathView, layoutParams)
    }

    fun start() {
        playPathAnimation(pathView)
    }

    fun setPath(pointList: ArrayList&lt;Point&gt;) {
        pathView.setPath(pointList)
    }

    private fun playPathAnimation(target: View) {
        val anim = ObjectAnimator.ofFloat(target, "percentage", 0.0f, 1.0f)

        anim.duration = 3000
        anim.interpolator = LinearInterpolator()
        anim.start()
    }
}
</pre>