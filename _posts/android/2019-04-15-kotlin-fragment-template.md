---
layout: post
title: Kotlin Fragment 템플릿
category: Android
tag: [Android, Kotlin]
---

팩토리 메소드(`newInstance`)로 Fragment를 생성하는 예제 코드입니다.

<pre class="prettyprint">
import android.os.Bundle
import android.support.v4.app.Fragment
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup

private const val ARG_TITLE = "TITLE"

class TodoListFragment : Fragment() {

    private var title: String? = ""

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        arguments?.let {
            title = it.getString(ARG_TITLE)
        }
    }

    companion object {
        fun newInstance(title: String): TodoListFragment {
            return TodoListFragment().apply {
                arguments = Bundle().apply {
                    putString(ARG_TITLE, title)
                }
            }
        }
    }

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?,
                              savedInstanceState: Bundle?): View? {
        return inflater.inflate(R.layout.fragment_todolist, container, false)

    }
}
</pre>