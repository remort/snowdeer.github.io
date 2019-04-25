---
layout: post
title: Kotlin Parcelable 인터페이스 구현 예제
category: Android, Kotlin
tag: [Android, Kotlin]
---

## TodoInfo.kt

<pre class="prettyprint">
import android.os.Parcel
import android.os.Parcelable
import io.realm.RealmObject
import io.realm.annotations.PrimaryKey

open class TodoInfo(
        @PrimaryKey
        var id: Long = 0,
        var categoryId: Long = 0,
        var text: String = "",
        var date: Long = 0,
        var done: Boolean = false,
        var seq: Long = 0,
        var isNotification: Boolean = false
) : RealmObject(), Model, Parcelable {
    constructor(categoryId: Long, text: String) : this(0, categoryId, text, 0, false, 0, false)

    private constructor(p: Parcel) : this(
            p.readLong(), p.readLong(), p.readString(), p.readLong(),
            p.readInt() == 1, p.readLong(), p.readInt() == 1
    )

    override fun writeToParcel(dest: Parcel?, flags: Int) {
        dest?.writeLong(id)
        dest?.writeLong(categoryId)
        dest?.writeString(text)
        dest?.writeLong(date)
        dest?.writeInt(if (done) 1 else 0)
        dest?.writeLong(seq)
        dest?.writeInt(if (isNotification) 1 else 0)
    }

    override fun describeContents(): Int {
        return 0
    }

    companion object CREATOR : Parcelable.Creator&lt;TodoInfo&gt; {
        override fun createFromParcel(parcel: Parcel): TodoInfo {
            return TodoInfo(parcel)
        }

        override fun newArray(size: Int): Array&lt;TodoInfo?&gt; {
            return arrayOfNulls(size)
        }
    }
}
</pre>

<br>

## 활용 예제(Fragment의 newInstance 메소드 매개변수로 Parcelable 전달하기)

<pre class="prettyprint">
class SampleFragment() : DialogFragment() {

    private lateinit var todoInfo : TodoInfo

    companion object {
        private const val ARG_TODO_INFO = "ARG_TODO_INFO"

        fun newInstance(todoInfo: TodoInfo): SampleFragment {
            return SampleFragment().apply {
                arguments = Bundle().apply {
                    putParcelable(ARG_TODO_INFO, todoInfo)
                }
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        arguments?.let {
            todoInfo = it.getParcelable(ARG_TODO_INFO)
        }
    }
}
</pre>