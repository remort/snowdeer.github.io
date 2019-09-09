---
layout: post
title: 다양한 Animation 샘플(Scene Transition)
category: Android
tag: [Android]
---

<pre class="prettyprint">
package com.snowdeer.animation.sample.fragment

import android.os.Bundle
import android.view.Gravity
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.Fragment
import androidx.transition.*
import com.snowdeer.animation.sample.R
import kotlinx.android.synthetic.main.fragment_scene_change.view.*
import kotlinx.android.synthetic.main.scene1.view.*

class SceneChangeFragment :Fragment() {

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?,
                              savedInstanceState: Bundle?): View? {

        val view = inflater.inflate(R.layout.fragment_scene_change, container, false)

        val scene1 = Scene(view.scene_root!!, view.container)
        val scene2 = Scene.getSceneForLayout(view.scene_root, R.layout.scene2, activity!!)
        val scene3 = Scene.getSceneForLayout(view.scene_root, R.layout.scene3, activity!!)


        view.scene_1_button.setOnClickListener {
            TransitionManager.go(scene1)
        }

        view.scene_2_button.setOnClickListener {
            val set = TransitionSet()
            val slide = Slide(Gravity.LEFT)
            slide.addTarget(R.id.image2)
            set.addTransition(slide)
            set.addTransition(ChangeBounds())
            set.ordering = TransitionSet.ORDERING_TOGETHER
            set.duration = 350
            TransitionManager.go(scene2, set)
        }

        view.scene_3_button.setOnClickListener {
            TransitionManager.go(scene3)
        }

        return view
    }
}
</pre>
