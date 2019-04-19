---
layout: post
title: Kotlin TabLayout에 CustomView 적용하는 예제
category: Android
tag: [Android, Kotlin]
---

## item_selected_tab.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
  android:layout_width="wrap_content"
  android:layout_height="wrap_content"
  android:orientation="horizontal"&gt;

  &lt;TextView
    android:id="@+id/title"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:layout_marginEnd="10dp"
    android:layout_gravity="center"
    android:maxWidth="@dimen/tab_max_width"
    android:minWidth="20dp"
    android:singleLine="true"
    android:textColor="@color/textColorPrimary"
    android:textSize="16sp"/&gt;

  &lt;Button
    android:id="@+id/remove_button"
    android:layout_width="24dp"
    android:layout_height="24dp"
    android:layout_gravity="center"
    android:background="@drawable/btn_remove"/&gt;
  
&lt;/LinearLayout&gt;
</pre>

<br>

## MainActivity.kt

<pre class="prettyprint">
class MainActivity : AppCompatActivity(), TabLayout.OnTabSelectedListener {

    private val tabLayout: TabLayout by lazy {
        findViewById&lt;TabLayout&gt;(R.id.tabs)
    }

    private val adapter: CategoryPagerAdapter by lazy {
        CategoryPagerAdapter(supportFragmentManager)
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        val toolbar = findViewById&lt;Toolbar&gt;(R.id.toolbar)

        setSupportActionBar(toolbar)

        viewpager.adapter = adapter
        tabLayout.setupWithViewPager(viewpager)
        tabLayout.addOnTabSelectedListener(this)

        add_tab.setOnClickListener {
            showEditCategoryNameDialog(null)
        }
    }

    override fun onTabReselected(tab: TabLayout.Tab?) {
        TODO("not implemented") //To change body of created functions use File | Settings | File Templates.
    }

    override fun onTabUnselected(tab: TabLayout.Tab?) {
        tab?.customView = null
    }

    override fun onTabSelected(tab: TabLayout.Tab?) {
        val view = LayoutInflater.from(applicationContext).inflate(R.layout.item_selected_tab, null)
        todoInsertFragment.currentTabPosition = tab?.position?.toLong() ?: 0

        val tabInfo = adapter.getTabInfo(tab?.position ?: 0)
        view.title.text = tabInfo?.name

        view.title.setOnLongClickListener(object : View.OnLongClickListener {
            override fun onLongClick(v: View?): Boolean {
                showEditCategoryNameDialog(tabInfo)
                return true
            }

        })

        view.remove_button.setOnClickListener {
            if (tabInfo != null) showDeleteCategoryDialog(tabInfo)
        }

        tab?.customView = view
    }

    private fun showEditCategoryNameDialog(tabInfo: TabInfo?) {
        val dialogLayout = LayoutInflater.from(applicationContext).inflate(R.layout.dialog_edit_name, null)
        val editText = dialogLayout.findViewById&lt;EditText&gt;(R.id.editText)
        editText.setText(tabInfo?.name ?: "")

        AlertDialog.Builder(this)
                .setTitle("이름을 입력하세요.")
                .setView(dialogLayout)
                .setPositiveButton("OK") { _, _ ->
                    if (tabInfo != null) {
                        ModelManager.instance.updateTabInfo(tabInfo.id, editText.text.toString())
                    } else {
                        ModelManager.instance.addTabInfo(editText.text.toString())
                        selectLastTab()
                    }
                }
                .show()
    }

    private fun showDeleteCategoryDialog(tabInfo: TabInfo) {
        AlertDialog.Builder(this)
                .setTitle("Category 삭제")
                .setMessage("${tabInfo.name} 항목을 정말 삭제하시겠습니까?")
                .setPositiveButton("확인") { _, _ ->
                    ModelManager.instance.deleteTabInfo(tabInfo.id)
                    selectNextTab(tabLayout.selectedTabPosition)
                }
                .show()
    }
}
</pre>