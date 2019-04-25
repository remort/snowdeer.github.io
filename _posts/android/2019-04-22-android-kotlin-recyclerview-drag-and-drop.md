---
layout: post
title: Kotlin Drap & Drop 지원하는 RecyclerView (이미지 부분 눌러서 드래그)
category: Android, Kotlin
tag: [Android, Kotlin]
---

## item_todo.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;RelativeLayout xmlns:android="http://schemas.android.com/apk/res/android"
  android:layout_width="match_parent"
  android:layout_height="wrap_content"&gt;

  &lt;ImageView
    android:id="@+id/drag_handle"
    android:layout_width="40dp"
    android:layout_height="40dp"
    android:layout_marginEnd="20dp"
    android:layout_alignParentEnd="true"
    android:layout_centerVertical="true"
    android:src="@drawable/drag_handle"/&gt;

  &lt;TextView
    android:id="@+id/text"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:layout_marginTop="15dp"
    android:layout_marginBottom="15dp"
    android:layout_marginStart="10dp"
    android:layout_marginEnd="20dp"
    android:layout_centerVertical="true"
    android:layout_toStartOf="@id/drag_handle"
    android:textAppearance="@style/TextAppearance.AppCompat.Body2"
    android:textSize="18sp"/&gt;

&lt;/RelativeLayout&gt;
</pre>

<br>

## ItemTouchHelperViewHolder.kt

<pre class="prettyprint">
interface ItemTouchHelperViewHolder {
    fun onItemSelected()
    fun onItemClear()
}
</pre>

<br>

## ItemTouItemTouchHelperAdapter.kt

<pre class="prettyprint">
interface ItemTouchHelperAdapter {
    fun onItemMove(fromPosition: Int, toPosition: Int): Boolean
    fun onItemRemove(position: Int)
}
</pre>

<br>

## 

## OnRecyclerAdapterEventListener.kt
<pre class="prettyprint">
interface OnRecyclerAdapterEventListener {
    fun onItemClicked(position: Int)
    fun onItemLongClicked(position: Int)
    fun onDragStarted(viewHolder: RecyclerView.ViewHolder)
}
</pre>

<br>

## SimpleItemTouchHelperCallback.kt

<pre class="prettyprint">
import android.graphics.Canvas
import android.support.v7.widget.GridLayoutManager
import android.support.v7.widget.RecyclerView
import android.support.v7.widget.helper.ItemTouchHelper

class SimpleItemTouchHelperCallback(val adapter: ItemTouchHelperAdapter) :
        ItemTouchHelper.Callback() {
    private val MAX_ALPHA = 1.0f

    override fun isItemViewSwipeEnabled(): Boolean {
        return false
    }

    override fun isLongPressDragEnabled(): Boolean {
        return false
    }


    override fun getMovementFlags(recyclerView: RecyclerView,
                                  viewHolder: RecyclerView.ViewHolder): Int {
        var dragFlags: Int
        var swipeFlags: Int

        if (recyclerView.layoutManager is GridLayoutManager) {
            dragFlags = ItemTouchHelper.UP or ItemTouchHelper.DOWN or
                    ItemTouchHelper.LEFT or ItemTouchHelper.RIGHT
            swipeFlags = 0
        } else {
            dragFlags = ItemTouchHelper.UP or ItemTouchHelper.DOWN
            swipeFlags = ItemTouchHelper.LEFT or ItemTouchHelper.RIGHT
        }

        return makeMovementFlags(dragFlags, swipeFlags)
    }

    override fun onMove(recyclerView: RecyclerView,
                        source: RecyclerView.ViewHolder,
                        target: RecyclerView.ViewHolder): Boolean {
        if (source.itemViewType != target.itemViewType) {
            return false
        }

        adapter.onItemMove(source.adapterPosition, target.adapterPosition)
        return true
    }

    override fun onSwiped(viewHolder: RecyclerView.ViewHolder, position: Int) {
        adapter.onItemRemove(viewHolder.adapterPosition)
    }

    override fun onChildDraw(c: Canvas, recyclerView: RecyclerView,
                             viewHolder: RecyclerView.ViewHolder,
                             dX: Float, dY: Float, actionState: Int,
                             isCurrentlyActive: Boolean) {

        if (actionState == ItemTouchHelper.ACTION_STATE_SWIPE) {
            val alpha = MAX_ALPHA - Math.abs(dX) / viewHolder.itemView.width
            viewHolder.itemView.alpha = alpha
            viewHolder.itemView.translationX = dX
        } else {
            super.onChildDraw(c, recyclerView, viewHolder, dX, dY, actionState, isCurrentlyActive)
        }
    }

    override fun onSelectedChanged(viewHolder: RecyclerView.ViewHolder?,
                                   actionState: Int) {
        if (actionState == ItemTouchHelper.ACTION_STATE_IDLE) {
            if (viewHolder is ItemTouchHelperViewHolder) {
                viewHolder.onItemSelected()
            }
        }

        super.onSelectedChanged(viewHolder, actionState)
    }

    override fun clearView(recyclerView: RecyclerView, viewHolder:
    RecyclerView.ViewHolder) {
        super.clearView(recyclerView, viewHolder)

        viewHolder.itemView.alpha = MAX_ALPHA

        if (viewHolder is ItemTouchHelperViewHolder) {
            viewHolder.onItemClear()
        }
    }
}
</pre>

<br>

## TodoListAdapter.kt

<pre class="prettyprint">
import android.content.Context
import android.graphics.Paint
import android.support.v7.widget.RecyclerView
import android.view.*
import android.widget.ImageView
import android.widget.TextView
import com.ran.todolist.R
import com.ran.todolist.common.TodoInfo
import com.ran.todolist.model.ModelManager
import com.ran.todolist.model.OnTodoInfoEventListener
import com.ran.todolist.utils.Log
import com.ran.todolist.utils.recyclerview.ItemTouchHelperAdapter
import com.ran.todolist.utils.recyclerview.OnRecyclerAdapterEventListener
import kotlinx.android.synthetic.main.item_todo.view.*
import java.util.*
import kotlin.collections.ArrayList

class TodoListAdapter(private val ctx: Context, private val categoryId: Long) :
        RecyclerView.Adapter&lt;ViewHolder&gt;(), OnTodoInfoEventListener,
        ItemTouchHelperAdapter {

    private var list: ArrayList&lt;TodoInfo&gt; = ArrayList()

    private var onEventListener: OnRecyclerAdapterEventListener? = null

    fun setOnRecyclerAdapterEventListener(l: OnRecyclerAdapterEventListener) {
        onEventListener = l
    }

    init {
        ModelManager.instance.addOnTodoInfoEventListener(this)
        refresh()
    }

    private fun refresh() {
        list = ModelManager.instance.getList(categoryId)

        for (i in 0 until list.size) {
            Log.i("snowdeer] ${list[i]}")
        }

        notifyDataSetChanged()
    }

    override fun onCreateViewHolder(parent: ViewGroup, position: Int): ViewHolder {
        val view: View = LayoutInflater.from(ctx).inflate(R.layout.item_todo, parent, false)
        return ViewHolder(view)
    }

    fun getItem(position: Int): TodoInfo {
        return list[position]
    }

    override fun getItemCount(): Int {
        return list.size
    }

    override fun onModelUpdated(info: TodoInfo) {
        refresh()
    }

    override fun onBindViewHolder(viewHolder: ViewHolder, position: Int) {
        viewHolder.text.text = list[position].text
        viewHolder.text.paintFlags = when (list[position].done) {
            true -> Paint.STRIKE_THRU_TEXT_FLAG
            else -> 0
        }

        val info = list[position]
        viewHolder.text.setOnLongClickListener {
            Log.i("onItemLongClicked: $position")
            onEventListener?.onItemLongClicked(position)
            true
        }

        viewHolder.handle.setOnTouchListener(View.OnTouchListener { _, event ->
            if (event.action == MotionEvent.ACTION_DOWN) {
                onEventListener?.onDragStarted(viewHolder);
            }
            false;
        })
    }

    override fun onItemMove(fromPosition: Int, toPosition: Int): Boolean {
        swap(fromPosition, toPosition)
        return true
    }

    override fun onItemRemove(position: Int) {
        ModelManager.instance.deleteTodoInfo(list[position])
    }

    private fun swap(from: Int, to: Int) {
        ModelManager.instance.swap(list[from], list[to])
        Collections.swap(list, from, to)
        notifyItemMoved(from, to)
    }

}

class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {
    val text: TextView = view.text
    val handle: ImageView = view.drag_handle
}
</pre>

<br>

## TodoListFragment.kt

<pre class="prettyprint">
import android.os.Bundle
import android.support.v4.app.Fragment
import android.support.v7.widget.LinearLayoutManager
import android.support.v7.widget.RecyclerView
import android.support.v7.widget.helper.ItemTouchHelper
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import com.ran.todolist.R
import com.ran.todolist.utils.recyclerview.OnRecyclerAdapterEventListener
import com.ran.todolist.utils.recyclerview.SimpleItemTouchHelperCallback
import kotlinx.android.synthetic.main.fragment_todolist.view.*

private const val ARG_TAB_KEY = "ARG_TAB_KEY"

class TodoListFragment : Fragment(), OnRecyclerAdapterEventListener {

    private var categoryId = 0L
    private lateinit var itemTouchHelper: ItemTouchHelper
    private val adapter by lazy {
        TodoListAdapter(activity!!.applicationContext, categoryId)
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        arguments?.let {
            categoryId = it.getLong(ARG_TAB_KEY)
        }
    }

    companion object {
        fun newInstance(categoryId: Long): TodoListFragment {
            return TodoListFragment().apply {
                arguments = Bundle().apply {
                    putLong(ARG_TAB_KEY, categoryId)
                }
            }
        }
    }

    override fun onCreateView(inflater: LayoutInflater, container: ViewGroup?,
                              savedInstanceState: Bundle?): View? {
        val view = inflater.inflate(R.layout.fragment_todolist, container, false)

        view.recycler_view.layoutManager = LinearLayoutManager(activity)
        view.recycler_view.adapter = adapter

        adapter.setOnRecyclerAdapterEventListener(this)
        val callback = SimpleItemTouchHelperCallback(adapter)
        itemTouchHelper = ItemTouchHelper(callback)
        itemTouchHelper.attachToRecyclerView(view.recycler_view)

        return view
    }

    override fun onItemClicked(position: Int) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun onItemLongClicked(position: Int) {
        val fragment = TodoDeleteFragment()

        fragment.currentItemId = adapter.getItem(position)?.id
        fragment.show(activity?.supportFragmentManager, "TodoInsertFragment")
    }

    override fun onDragStarted(viewHolder: RecyclerView.ViewHolder) {
        itemTouchHelper.startDrag(viewHolder)
    }
}
</pre>
