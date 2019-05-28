---
layout: post
title: Directory Picker 구현해보기
category: Android
tag: [Android]
---

팝업 형태의 다이얼로그(Dialog)로 실행하는 Directory Picker 입니다.

## dialog_directory_picker.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:app="http://schemas.android.com/apk/res-auto"
  xmlns:tools="http://schemas.android.com/tools"
  android:layout_width="wrap_content"
  android:layout_height="wrap_content"
  android:orientation="vertical"&gt;

  &lt;FrameLayout
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:background="@color/colorPrimaryDark"&gt;

    &lt;TextView
      android:textStyle="bold"
      android:layout_width="match_parent"
      android:layout_height="wrap_content"
      android:layout_marginTop="8dp"
      android:layout_marginBottom="8dp"
      android:layout_marginLeft="12dp"
      android:text="@string/directory_picker_title"
      android:textColor="@color/colorDialogTitle"
      android:textSize="24sp"/&gt;

  &lt;/FrameLayout&gt;

  &lt;FrameLayout
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:background="@color/colorCurrentPathBackground"&gt;

    &lt;TextView
      android:id="@+id/current_path"
      android:layout_width="match_parent"
      android:layout_height="wrap_content"
      android:layout_marginTop="4dp"
      android:layout_marginBottom="4dp"
      android:layout_marginLeft="12dp"
      android:text="current directory path"
      android:textColor="@color/colorCurrentPathText"
      android:textSize="16dp"/&gt;

  &lt;/FrameLayout&gt;

  &lt;android.support.v7.widget.RecyclerView
    android:id="@+id/recycler_view"
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    android:layout_marginTop="16dp"
    android:minHeight="320dp"/&gt;

&lt;/LinearLayout&gt;
</pre>

<br>

## DirectoryListAdapter.kt

<pre class="prettyprint">
import android.content.Context
import android.os.Environment
import android.support.v7.widget.RecyclerView
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import com.snowdeer.ftpserver.R
import kotlinx.android.synthetic.main.item_directory.view.*
import java.io.File

interface OnDirectoryListEventListener {
    fun onDirectoryChanged(path: String)
}

data class DirectoryItem(val name: String, val path: String)

class DirectoryListAdapter(private val ctx: Context) : RecyclerView.Adapter&lt;ViewHolder&gt;() {

    private val list = ArrayList&lt;DirectoryItem&gt;()
    private val root = Environment.getExternalStorageDirectory().absolutePath
    var currentPath: String = ""

    var onDirectoryListEventListener: OnDirectoryListEventListener? = null

    fun refresh(path: String) {
        list.clear()

        val file = File(path)
        currentPath = file.path
        onDirectoryListEventListener?.onDirectoryChanged(path)

        addParentDirectory(file)

        if (!file.exists()) {
            notifyDataSetChanged()
            return
        }

        val files = file.listFiles()
        files?.let {
            for (f in it) {
                if (f.isHidden) {
                    continue
                }

                if (f.isDirectory) {
                    list.add(DirectoryItem(f.name, f.absolutePath))
                }
            }
        }

        notifyDataSetChanged()
    }

    private fun addParentDirectory(file: File) {
        val path = file.path
        if ((path == "/") || (path == root)) {
            return
        }

        list.add(DirectoryItem("..", file.parentFile.path))
    }

    override fun onCreateViewHolder(parent: ViewGroup, position: Int): ViewHolder {
        val view: View = LayoutInflater.from(ctx).inflate(R.layout.item_directory, parent, false)
        return ViewHolder(view)
    }

    fun getItem(position: Int): DirectoryItem {
        return list[position]
    }

    override fun getItemCount(): Int {
        return list.size
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        val item = getItem(holder.adapterPosition)

        holder.name.text = item.name

        holder.layout_item.setOnClickListener {
            refresh(item.path)
        }
    }
}

class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {
    val layout_item = view.layout_item!!
    val name = view.name!!
}
</pre>

<br>

## DirectoryPickerDialog.kt

<pre class="prettyprint">
import android.app.Dialog
import android.os.Bundle
import android.support.v4.app.DialogFragment
import android.support.v4.app.FragmentActivity
import android.support.v7.app.AlertDialog
import android.support.v7.widget.LinearLayoutManager
import android.view.LayoutInflater
import android.widget.Toast
import com.snowdeer.ftpserver.R
import com.snowdeer.ftpserver.SnowPreference
import kotlinx.android.synthetic.main.dialog_directory_picker.view.*
import kotlinx.android.synthetic.main.item_directory.view.*
import java.io.File
import java.util.*

interface OnDirectoryPickerEventListener {
    fun onDirectorySelected(path: String)
}

class DirectoryPickerDialog : DialogFragment() {

    private lateinit var adapter: DirectoryListAdapter

    var onDirectoryPickerEventListener: OnDirectoryPickerEventListener? = null

    override fun onCreateDialog(savedInstanceState: Bundle?): Dialog {
        val builder = AlertDialog.Builder(Objects.requireNonNull&lt;FragmentActivity&gt;(activity))
        val inflater = activity!!.layoutInflater
        val view = inflater.inflate(R.layout.dialog_directory_picker, null)

        adapter = DirectoryListAdapter(activity!!)
        adapter.onDirectoryListEventListener = object : OnDirectoryListEventListener {
            override fun onDirectoryChanged(path: String) {
                view.current_path.text = path
            }
        }

        view.recycler_view.layoutManager = LinearLayoutManager(activity)
        view.recycler_view.adapter = adapter
        adapter.refresh(SnowPreference.getDirectory(activity!!))

        builder.setView(view)
                .setPositiveButton(getString(R.string.directory_picker_ok), null)
                .setNeutralButton(getString(R.string.directory_picker_new_directory), null)

        val dialog = builder.create()

        dialog.setOnShowListener {
            dialog.getButton(AlertDialog.BUTTON_POSITIVE).setOnClickListener {
                onDirectoryPickerEventListener?.onDirectorySelected(adapter.currentPath)
                dialog.dismiss()
            }

            dialog.getButton(AlertDialog.BUTTON_NEUTRAL).setOnClickListener {
                showNewDirectoryDialog(adapter.currentPath)
            }
        }

        return dialog
    }

    private fun showNewDirectoryDialog(path: String) {
        val view = LayoutInflater.from(activity).inflate(R.layout.dialog_new_directory, null)

        AlertDialog.Builder(activity!!)
                .setTitle(getString(R.string.directory_picker_new_directory))
                .setView(view)
                .setPositiveButton("Ok") { _, _ ->
                    val newPath = path + "/" + view.name.text.toString()
                    if (createDirectory(newPath)) {
                        adapter.refresh(path)
                    } else {
                        Toast.makeText(activity!!, getString(R.string.new_directory_failed), Toast.LENGTH_SHORT).show()
                    }

                }
                .show()
    }

    private fun createDirectory(path: String): Boolean {
        val file = File(path)

        if (file.exists()) {
            return false
        }

        return file.mkdir()
    }
}
</pre>