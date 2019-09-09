---
layout: post
title: 다양한 Animation 샘플(Reveal)
category: Android
tag: [Android]
---

<pre class="prettyprint">
package com.snowdeer.animation.sample.fragment

import android.animation.Animator
import android.animation.AnimatorListenerAdapter
import android.animation.AnimatorSet
import android.animation.ObjectAnimator
import android.os.Bundle
import android.os.Handler
import android.view.*
import android.view.animation.AccelerateDecelerateInterpolator
import androidx.fragment.app.Fragment
import com.snowdeer.animation.sample.R
import kotlinx.android.synthetic.main.fragment_reveal_transition.*
import kotlinx.android.synthetic.main.fragment_reveal_transition.view.*
import kotlin.math.hypot


class RevealTransitionFragment : Fragment() {

    private val handler = Handler()

    private var degree = 0
    private var isAnimating = false

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?,
                              savedInstanceState: Bundle?): View? {

        val view = inflater.inflate(R.layout.fragment_reveal_transition, container, false)

        view.reveal_transition_button.setOnClickListener {
            image.visibility = View.GONE

            handler.post {
                val anim = animateRevealColorFromCoordinates(content_main, content_main.width / 2, 0)
                anim.addListener(object : AnimatorListenerAdapter() {
                    override fun onAnimationEnd(animation: Animator) {
                        // TODO
                    }
                })
            }

            handler.postDelayed({
                fadeIn(image)
            }, 800)
        }

        return view
    }

    private fun fadeIn(target: View) {
        val animator = ObjectAnimator.ofFloat(target, View.ALPHA, 0F, 1f)
        animator.duration = 1000
        animator.start()
        target.visibility = View.VISIBLE
    }

    private fun animateRevealColorFromCoordinates(viewRoot: ViewGroup, x: Int, y: Int): Animator {
        val finalRadius = hypot(viewRoot.width.toDouble(), viewRoot.height.toDouble()).toFloat()

        val anim = ViewAnimationUtils.createCircularReveal(viewRoot, x, y, 0f, finalRadius)
        anim.duration = 1500
        anim.interpolator = AccelerateDecelerateInterpolator()
        viewRoot.visibility = View.VISIBLE
        anim.start()
        return anim
    }
}
</pre>
