---
layout: post
title: Kotlin에서 Swing을 이용한 Desktop GUI App 만들기
category: Kotlin
tag: [Kotlin]
---

## 간단한 GUI Application 구현하기

<pre class="prettyprint">
package com.snowdeer

import java.awt.BorderLayout
import java.awt.Dimension
import java.lang.Thread.sleep
import javax.swing.JFrame
import javax.swing.JScrollPane
import javax.swing.JTextArea

fun main(args: Array&lt;String&gt;) {
    val textArea = JTextArea()
    textArea.text = "Hello, SnowDeer"
    val scrollPane = JScrollPane(textArea)

    val frame = JFrame("Hello, SnowDeer")
    frame.contentPane.add(scrollPane, BorderLayout.CENTER)
    frame.defaultCloseOperation = JFrame.EXIT_ON_CLOSE
    frame.size = Dimension(600, 400)
    frame.setLocationRelativeTo(null)
    frame.isVisible = true
}
</pre>
