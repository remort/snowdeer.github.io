---
layout: post
title: 커스텀 달력 만들어보기
category: Android
tag: [Android]
---

## DayListBuilder.kt

<pre class="prettyprint">
import java.util.*
import kotlin.collections.ArrayList

data class DayInfo(
        val year: Int, val month: Int, val day: Int,
        val timestamp: Long, val inMonth: Boolean) {

    fun isToday(): Boolean {
        val c = Calendar.getInstance()
        val todayYear = c.get(Calendar.YEAR)
        val todayMonth = c.get(Calendar.MONTH)
        val todayDay = c.get(Calendar.DAY_OF_MONTH)

        return (todayYear == year) && (todayMonth == month) && (todayDay == day)
    }
}

class DayListBuilder {

    companion object {
        val instance = DayListBuilder()
    }

    private constructor()

    fun getDayList(year: Int, month: Int): ArrayList&lt;DayInfo&gt; {
        val list = ArrayList&lt;DayInfo&gt;()

        val c = Calendar.getInstance()

        c.set(Calendar.YEAR, year)
        c.set(Calendar.MONTH, month - 1)
        c.set(Calendar.DAY_OF_MONTH, 1)
        c.set(Calendar.HOUR_OF_DAY, 3)
        c.set(Calendar.MINUTE, 0)
        c.set(Calendar.SECOND, 0)

        // Add days of previous Month
        val prevMonthDays = c.get(Calendar.DAY_OF_WEEK) - Calendar.SUNDAY
        c.add(Calendar.DAY_OF_YEAR, -prevMonthDays)
        for (day in 0 until prevMonthDays) {
            list.add(DayInfo(c.get(Calendar.YEAR), c.get(Calendar.MONTH),
                    c.get(Calendar.DAY_OF_MONTH), c.timeInMillis, false))
            c.add(Calendar.DAY_OF_YEAR, 1)
        }

        // Add days of target Month
        for (day in 1..c.getActualMaximum(Calendar.DAY_OF_MONTH)) {
            c.set(Calendar.DAY_OF_MONTH, day)
            list.add(DayInfo(c.get(Calendar.YEAR), c.get(Calendar.MONTH),
                    c.get(Calendar.DAY_OF_MONTH), c.timeInMillis, false))
        }

        // Add days of next Month
        val nextMonthDays = Calendar.SATURDAY - c.get(Calendar.DAY_OF_WEEK)
        for (day in 1..nextMonthDays) {
            c.add(Calendar.DAY_OF_YEAR, 1)
            list.add(DayInfo(c.get(Calendar.YEAR), c.get(Calendar.MONTH),
                    c.get(Calendar.DAY_OF_MONTH), c.timeInMillis, false))
        }

        return list
    }
}
</pre>

<br>

## CalendarAdapter.kt

<pre class="prettyprint">
import android.content.Context
import android.graphics.Color
import android.support.v7.app.AlertDialog
import android.support.v7.widget.RecyclerView
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import com.snowdeer.htracker.R
import com.snowdeer.htracker.model.OnDatabaseEventListener
import com.snowdeer.htracker.model.RewardModelManager
import com.snowdeer.htracker.model.data.HabitDto
import kotlinx.android.synthetic.main.item_calendar.view.*
import java.util.*
import kotlin.collections.ArrayList

class CalendarAdapter(
        private val ctx: Context, private val habit: HabitDto?
        , private var year: Int, private var month: Int
) : RecyclerView.Adapter&lt;ViewHolder&gt;(), OnDatabaseEventListener {

    private var list = ArrayList&lt;DayInfo&gt;()

    init {
        refresh()
        RewardModelManager.instance.setOnDatabaseEventListener(this)
    }

    private fun refresh() {
        list = DayListBuilder.instance.getDayList(year, month)
        notifyDataSetChanged()
    }

    fun refresh(year: Int, month: Int) {
        this.year = year
        this.month = month
        refresh()
    }

    override fun onDatabaseUpdated() {

    }

    fun getItem(position: Int): DayInfo {
        return list[position]
    }

    override fun getItemCount(): Int {
        return list.size
    }

    override fun onCreateViewHolder(parent: ViewGroup, position: Int): ViewHolder {
        val view: View = LayoutInflater.from(ctx).inflate(R.layout.item_calendar, parent, false)
        return ViewHolder(view)
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        val item = getItem(holder.adapterPosition)

        holder.day.text = item.day.toString()
        holder.day.setTextColor(getDayColor(item))

        if (existStamp(item.year, item.month, item.day)) {
            holder.stamp.visibility = View.VISIBLE
        } else {
            holder.stamp.visibility = View.INVISIBLE
        }

        if (item.isToday()) {
            holder.background.setBackgroundColor(ctx.getColor(R.color.today_background))
        } else {
            holder.background.setBackgroundColor(ctx.getColor(R.color.transparent))
        }

        holder.background.setOnClickListener {
            if (habit != null) {
                if (item.isToday()) {
                    if (existStamp(item.year, item.month, item.day)) {
                        RewardModelManager.instance.delete(habit.id, item.year, item.month, item.day)
                    } else {
                        RewardModelManager.instance.create(habit.id, item.year, item.month, item.day)
                    }
                    notifyItemChanged(holder.adapterPosition)
                } else {
                    showForceHabitResultDialog(holder.adapterPosition, habit.id, item.year, item.month, item.day)
                }
            }
        }
    }

    private fun existStamp(year: Int, month: Int, day: Int): Boolean {
        if (habit == null) {
            return false
        }

        return RewardModelManager.instance.exist(habit.id, year, month, day)
    }

    private fun getDayColor(info: DayInfo): Int {
        val c = Calendar.getInstance()
        c.set(Calendar.YEAR, info.year)
        c.set(Calendar.MONTH, info.month)
        c.set(Calendar.DAY_OF_MONTH, info.day)

        if (info.inMonth) {
            when (c.get(Calendar.DAY_OF_WEEK)) {
                Calendar.SUNDAY -> return ctx.getColor(R.color.in_month_sunday)
                Calendar.SATURDAY -> return ctx.getColor(R.color.in_month_saturday)
            }
            return Color.BLACK
        } else {
            when (c.get(Calendar.DAY_OF_WEEK)) {
                Calendar.SUNDAY -> return ctx.getColor(R.color.out_month_sunday)
                Calendar.SATURDAY -> return ctx.getColor(R.color.out_month_saturday)
            }

            return Color.GRAY
        }
        return Color.BLACK
    }

    private fun showForceHabitResultDialog(pos: Int, habitId: Long, year: Int, month: Int, day: Int) {
        AlertDialog.Builder(ctx)
                .setTitle("It is not Today. Do you want to edit the result?")
                .setPositiveButton("Yes") { _, _ ->
                    if (existStamp(year, month, day)) {
                        RewardModelManager.instance.delete(habitId, year, month, day)
                    } else {
                        RewardModelManager.instance.create(habitId, year, month, day)
                    }
                    notifyItemChanged(pos)
                }
                .setNegativeButton("No") { _, _ ->
                }
                .show()
    }
}

class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {
    val background = view.item_layout!!
    val day = view.day!!
    val stamp = view.stamp!!
}
</pre>

<br>

## fragment_calendar.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;LinearLayout
  xmlns:android="http://schemas.android.com/apk/res/android"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:orientation="vertical"&gt;

  &lt;LinearLayout
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:orientation="vertical"&gt;

    &lt;TextView
      android:id="@+id/month"
      android:textStyle="bold"
      android:layout_width="wrap_content"
      android:layout_height="wrap_content"
      android:layout_marginTop="8dp"
      android:layout_marginStart="8dp"
      android:layout_gravity="center_vertical"
      android:text="5"
      android:textColor="@color/black"
      android:textSize="34sp"/&gt;

    &lt;TextView
      android:id="@+id/year"
      android:layout_width="wrap_content"
      android:layout_height="wrap_content"
      android:layout_marginStart="8dp"
      android:layout_gravity="center_vertical"
      android:text="2019"
      android:textColor="@color/darkergray"
      android:textSize="16sp"/&gt;
  &lt;/LinearLayout&gt;

  &lt;android.support.v7.widget.RecyclerView
    android:id="@+id/recycler_view"
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    android:layout_margin="8dp"/&gt;

&lt;/LinearLayout&gt;
</pre>

<br>

## CalendarFragment.kt

<pre class="prettyprint">
import android.os.Bundle
import android.support.v4.app.Fragment
import android.support.v7.widget.GridLayoutManager
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import com.snowdeer.htracker.R
import com.snowdeer.htracker.model.data.HabitDto
import com.snowdeer.utils.Log
import kotlinx.android.synthetic.main.fragment_calendar.view.*
import java.util.*

private const val ARG_YEAR = "ARG_YEAR"
private const val ARG_MONTH = "ARG_MONTH"
private const val ARG_HABIT = "ARG_HABIT"

class CalendarFragment : Fragment() {

    private var year: Int = 2019
    private var month: Int = 1
    private var habit: HabitDto? = null

    private lateinit var adapter: CalendarAdapter

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        arguments?.let {
            year = it.getInt(ARG_YEAR)
            month = it.getInt(ARG_MONTH)
            habit = it.getParcelable(ARG_HABIT)
        }

        Log.i("onCreate: $habit")
    }

    companion object {
        fun newInstanceWithMonthOffset(habit: HabitDto?, offset: Int): CalendarFragment {
            val c = Calendar.getInstance()
            c.add(Calendar.MONTH, offset)

            Log.i("newInstanceWithMonthOffset: $habit")
            return newInstance(habit, c.get(Calendar.YEAR), c.get(Calendar.MONTH) + 1)
        }

        private fun newInstance(habit: HabitDto?, year: Int, month: Int): CalendarFragment {
            return CalendarFragment().apply {
                arguments = Bundle().apply {
                    putInt(ARG_YEAR, year)
                    putInt(ARG_MONTH, month)
                    putParcelable(ARG_HABIT, habit)
                }
            }
        }
    }

    override fun onCreateView(
            inflater: LayoutInflater, container: ViewGroup?,
            savedInstanceState: Bundle?): View? {
        val view = inflater.inflate(R.layout.fragment_calendar, container, false)

        view.year.text = year.toString()
        view.month.text = month.toString()

        Log.i("onCreateView: $habit")
        adapter = CalendarAdapter(activity!!, habit, year, month)
        view.recycler_view.layoutManager = GridLayoutManager(activity, 7)
        view.recycler_view.adapter = adapter

        return view
    }
}
</pre>