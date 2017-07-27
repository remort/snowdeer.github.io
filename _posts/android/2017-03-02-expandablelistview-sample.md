---
layout: post
title: ExpandableListView 예제
category: Android
tag: [Android, UX]
---

Android에서 다음 그림과 같이 Depth가 존재하는 ListView를 만들고 싶을 때는 어떻게 해야 할까요?
여러 가지 방법이 있긴 한데, 가장 간편하게 만들 수 있는 방법으로는 ExpandableListView를
사용하는 방법이 있습니다. 이 UI 컴포넌트는 Android SDK에 기본으로 포함되어 있습니다.

![image -fullwidth](/assets/2017-03-02-expandablelistview-sample/01.png)

위와 같은 화면을 구현하려면 크게 다음과 같은 요소들을 구현해주면 됩니다.

<ul>
 	<li class="clear">첫 번째 Depth의 ItemView</li>
 	<li class="clear">두 번째 Depth의 ItemView</li>
 	<li class="clear">ExpandableListView의 Adapter</li>
</ul>
<br>

# item_view_setting_parent.xml

<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
  android:id="@+id/layout_background"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:padding="15dp"
  android:orientation="horizontal"&gt;

  &lt;ImageView
    android:id="@+id/icon"
    android:layout_width="32dp"
    android:layout_height="32dp"
    android:layout_marginLeft="16dp"
    android:src="@mipmap/ic_launcher" /&gt;

  &lt;TextView
    android:id="@+id/name"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:layout_marginLeft="16dp"
    android:layout_gravity="center"
    android:text="name"
    android:textColor="@color/black"
    android:textSize="22sp" /&gt;

&lt;/LinearLayout&gt;</pre>
<br>

# item_view_setting_child.xml

<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
  android:id="@+id/layout_background"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:padding="15dp"
  android:orientation="horizontal"&gt;

  &lt;TextView
    android:id="@+id/name"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:layout_marginLeft="80dp"
    android:layout_gravity="center"
    android:text="name"
    android:textColor="@color/black"
    android:textSize="18sp" /&gt;

  &lt;Switch
    android:id="@+id/data_switch"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:layout_marginRight="18dp"
    android:layout_gravity="right"
    android:checked="true" /&gt;

&lt;/LinearLayout&gt;</pre>
<br>

# fragment_device_setting.xml

<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;android.support.design.widget.CoordinatorLayout xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:app="http://schemas.android.com/apk/res-auto"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:fitsSystemWindows="false"&gt;

  &lt;android.support.design.widget.AppBarLayout
    android:id="@+id/app_bar_layout"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:background="@color/background"
    android:fitsSystemWindows="true"&gt;

    &lt;RelativeLayout
      android:layout_width="match_parent"
      android:layout_height="wrap_content"&gt;

      &lt;TextView
        android:layout_width="wrap_content"
        android:layout_height="wrap_content"
        android:layout_margin="12dp"
        android:layout_centerInParent="true"
        android:text="Device Setting"
        android:textColor="@color/white"
        android:textSize="24sp" /&gt;

    &lt;/RelativeLayout&gt;

  &lt;/android.support.design.widget.AppBarLayout&gt;

  &lt;ExpandableListView
    android:id="@+id/expandable_listview"
    android:layout_width="match_parent"
    android:layout_height="match_parent"
    android:groupIndicator="@null"
    app:layout_behavior="@string/appbar_scrolling_view_behavior" /&gt;

&lt;/android.support.design.widget.CoordinatorLayout&gt;
</pre>
<br>

# DeviceSettingExpandableAdapter.java

<pre class="prettyprint">import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.BaseExpandableListAdapter;
import android.widget.ImageView;
import android.widget.Switch;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.HashMap;

public class DeviceSettingExpandableAdapter extends BaseExpandableListAdapter {

  private Context mContext;
  private ArrayList<SettingParentItem> mParentList;
  private HashMap<String, ArrayList<SettingChildItem>> mChildHashMap;

  public DeviceSettingExpandableAdapter(Context context) {
    this.mContext = context;

    initList();
  }

  private void initList() {
    mParentList = new ArrayList<>();
    mParentList.add(
        new SettingParentItem(SettingParentItem.KEY_MYPHONE,
            R.drawable.icon_setting_myphone, "My Phone"));
    mParentList.add(
        new SettingParentItem(SettingParentItem.KEY_SHEALTH,
            R.drawable.icon_setting_shealth, "SHealth"));

    mChildHashMap = new HashMap<>();

    ArrayList<SettingChildItem> childList = new ArrayList<>();
    childList.add(new SettingChildItem(DEVICE_DATA_TYPE.TYPE_MYPHONE_APP_USAGE,
        "앱 사용 시간", ""));
    childList.add(new SettingChildItem(DEVICE_DATA_TYPE.TYPE_MYPHONE_LOCATION,
        "위치 정보", ""));
    childList.add(new SettingChildItem(DEVICE_DATA_TYPE.TYPE_MYPHONE_ACTIVITY,
        "액티비티 정보", ""));
    childList.add(new SettingChildItem(DEVICE_DATA_TYPE.TYPE_MYPHONE_WEATHER,
        "날씨", ""));
    childList.add(new SettingChildItem(DEVICE_DATA_TYPE.TYPE_MYPHONE_EVENT,
        "스마트폰 이벤트", ""));
    mChildHashMap.put(mParentList.get(0).key + "", childList);

    childList = new ArrayList<>();
    childList.add(new SettingChildItem(DEVICE_DATA_TYPE.TYPE_SHEALTH_STEP_COUNT,
        "걸음 수", ""));
    childList.add(new SettingChildItem(DEVICE_DATA_TYPE.TYPE_SHEALTH_SLEEP,
        "수면 정보", ""));
    childList.add(new SettingChildItem(DEVICE_DATA_TYPE.TYPE_SHEALTH_HEART_RATE,
        "심박수", ""));
    mChildHashMap.put(mParentList.get(1).key + "", childList);
  }

  @Override
  public String getGroup(int groupPosition) {
    return mParentList.get(groupPosition).key + "";
  }

  @Override
  public int getGroupCount() {
    return mParentList.size();
  }

  @Override
  public long getGroupId(int groupPosition) {
    return groupPosition;
  }

  // ParentListView
  @Override
  public View getGroupView(int groupPosition, boolean isExpanded, View convertView,
      ViewGroup parent) {
    ParentItemHolder holder = null;
    View row = convertView;

    if(row == null) {
      LayoutInflater inflator = (LayoutInflater) mContext
          .getSystemService(Context.LAYOUT_INFLATER_SERVICE);
      row = inflator.inflate(R.layout.item_view_setting_parent, null);

      holder = new ParentItemHolder();

      holder.icon = (ImageView) row.findViewById(R.id.icon);
      holder.name = (TextView) row.findViewById(R.id.name);

      row.setTag(holder);
    } else {
      holder = (ParentItemHolder) row.getTag();
    }

    final SettingParentItem item = mParentList.get(groupPosition);
    if(item != null) {
      holder.icon.setImageResource(item.resId);
      holder.name.setText(item.name);
    }

    return row;
  }

  //ChildListView
  @Override
  public SettingChildItem getChild(int groupPosition, int childPosition) {
    return this.mChildHashMap.get(this.mParentList.get(groupPosition).key + "")
        .get(childPosition);

  }

  @Override
  public int getChildrenCount(int groupPosition) {
    return this.mChildHashMap.get(this.mParentList.get(groupPosition).key + "")
        .size();

  }

  @Override
  public long getChildId(int groupPosition, int childPosition) {
    return childPosition;
  }

  @Override
  public View getChildView(int groupPosition, int childPosition, boolean isLastChild,
      View convertView, ViewGroup parent) {
    ChildItemHolder holder = null;
    View row = convertView;

    if(row == null) {
      LayoutInflater inflator = (LayoutInflater) mContext
          .getSystemService(Context.LAYOUT_INFLATER_SERVICE);
      row = inflator.inflate(R.layout.item_view_setting_child, null);

      holder = new ChildItemHolder();

      holder.name = (TextView) row.findViewById(R.id.name);

      row.setTag(holder);
    } else {
      holder = (ChildItemHolder) row.getTag();
    }

    SettingChildItem item = getChild(groupPosition, childPosition);

    if(item != null) {
      holder.name.setText(item.name);
    }

    return row;
  }

  @Override
  public boolean hasStableIds() { return true; }

  @Override
  public boolean isChildSelectable(int groupPosition, int childPosition) { return true; }


  class ParentItemHolder {

    ImageView icon;
    TextView name;
  }

  ;

  class ChildItemHolder {

    TextView name;
    Switch _switch;
  }

  ;
}
</pre>
