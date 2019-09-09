---
layout: post
title: 다양한 Animation 샘플(크기, 회전, Fade In/Out)
category: Android
tag: [Android]
---

<pre class="prettyprint">
package com.snowdeer.animation.sample.fragment

import android.animation.AnimatorSet
import android.animation.ObjectAnimator
import android.os.Bundle
import android.os.Handler
import android.view.*
import androidx.core.animation.doOnEnd
import androidx.fragment.app.Fragment
import com.snowdeer.animation.sample.R
import kotlinx.android.synthetic.main.fragment_voice_bubble.view.*


class VoiceBubbleFragment : Fragment() {

    private val handler = Handler()

    private var degree = 0
    private var isAnimating = false

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?,
                              savedInstanceState: Bundle?): View? {

        val view = inflater.inflate(R.layout.fragment_voice_bubble, container, false)

        playScaleLoopAnimation(view.voice_indicator_1)

        handler.postDelayed({
            playScaleLoopAnimation(view.voice_indicator_2)

        }, 500)

        view.rotate_button.setOnClickListener {
            rotate(view.voice_layout) {}
        }

        view.show_bubble_button.setOnClickListener {
            if ((degree == 0) || (degree == 180)) {
                rotate(view.voice_layout) { fadeIn(view.question_layout) }
            } else {
                fadeIn(view.question_layout)
            }
        }

        view.hide_bubble_button.setOnClickListener {
            if ((degree == 90) || (degree == 270)) {
                rotate(view.voice_layout) {}
            }
            fadeOut(view.question_layout)
        }

        return view
    }

    private fun playScaleLoopAnimation(target: View) {
        val scaleUpX = ObjectAnimator.ofFloat(target, "scaleX", 2.5f)
        val scaleUpY = ObjectAnimator.ofFloat(target, "scaleY", 2.5f)

        scaleUpX.apply {
            duration = 1000
            repeatMode = ObjectAnimator.REVERSE
            repeatCount = ObjectAnimator.INFINITE
        }

        scaleUpY.apply {
            duration = 1000
            repeatMode = ObjectAnimator.REVERSE
            repeatCount = ObjectAnimator.INFINITE
        }

        val scaleDown = AnimatorSet()
        scaleDown.play(scaleUpX).with(scaleUpY)

        scaleDown.start()
    }

    private fun rotate(target: ViewGroup, nextAnim: () -> Unit) {
        if (!isAnimating) {
            isAnimating = true

            val targetDegree = degree + 90
            val rotateAnimator = ObjectAnimator.ofFloat(target,
                    "rotation", degree.toFloat(), targetDegree.toFloat())

            rotateAnimator.duration = 1000
            rotateAnimator.doOnEnd {
                degree = targetDegree
                isAnimating = false

                nextAnim()

            }
            rotateAnimator.start()
        }
    }

    private fun fadeIn(target: ViewGroup) {
        val animator = ObjectAnimator.ofFloat(target, View.ALPHA, 0F, 1f)
        animator.duration = 1000
        animator.start()
        target.visibility = View.VISIBLE
    }

    private fun fadeOut(target: ViewGroup) {
        val animator = ObjectAnimator.ofFloat(target, View.ALPHA, 1F, 0f)
        animator.duration = 1000
        animator.start()
    }
}
</pre>
