---
layout: post
title: Custom ListView 예제
category: Android
tag: [Android, UX]
---

Android에서 ListView 샘플입니다. ListView는 제일 많이 사용하는 UI 컴포넌트 중
하나이다보니 코드 재활용이 빈번합니다. 그래서 여기 템플릿으로 활용할 수 있는 예제를
하나 포스팅해봅니다. 참고로 GridView도 똑같은 방법으로 사용하면 됩니다.

ListView의 스크롤 속도를 부드럽고 빠르게 하기 위해서는 디자인 패턴 중
[ViewHolder Pattern](https://developer.android.com/training/improving-layouts/smooth-scrolling.html)을
사용합니다. Android에서 XML에서 리소스를 읽어오는 findViewById 함수는 시간이 많이 걸리는 함수입니다.
ViewHolder 패턴을 사용하면 최초에 한 번 findViewById 함수로 찾은 리소스를 Holder
안의 변수에 맵핑을 해놓아서 다음 번에 찾을 때 훨씬 빠르게 찾을 수 있습니다.
(Cache와 같은 원리입니다.)

아래는 ViewHolder 패턴을 적용한 예제입니다.
<br>
## item_view_log.xml
<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
  android:id="@+id/layout_background"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:paddingLeft="15dp"
  android:paddingRight="15dp"
  android:orientation="vertical"&gt;

  &lt;TextView
    android:id="@+id/timestamp"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="timestamp"
    android:textColor="@color/ics_theme_darkblue"
    android:textSize="14sp" /&gt;

  &lt;TextView
    android:id="@+id/desc"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="Description"
    android:textColor="@color/darkgray"
    android:textSize="14sp" /&gt;

&lt;/LinearLayout&gt;</pre>
<br>
## LogListAdapter.java
<pre class="prettyprint">public class LogListAdapter extends BaseAdapter {

  private final Context mContext;
  private ArrayList&lt;BaseInfo&gt; mList;

  public LogListAdapter(Context context) {
    mContext = context;
  }

  public void refresh(ArrayList&lt;BaseInfo&gt; list) {
    mList = list;
    notifyDataSetChanged();
  }

  @Override
  public int getCount() {
    if(mList == null) {
      return 0;
    }

    return mList.size();
  }

  @Override
  public BaseInfo getItem(int position) {
    if(mList == null) {
      return null;
    }

    return mList.get(position);
  }

  @Override
  public long getItemId(int position) {
    if(mList == null) {
      return 0;
    }

    return position;
  }

  @Override
  public View getView(int position, View convertView, ViewGroup parent) {
    ItemHolder holder = null;
    View row = convertView;

    if(row == null) {
      LayoutInflater inflator = (LayoutInflater) mContext
          .getSystemService(Context.LAYOUT_INFLATER_SERVICE);
      row = inflator.inflate(R.layout.item_view_log, null);

      holder = new ItemHolder();

      holder.timestamp = (TextView) row.findViewById(R.id.timestamp);
      holder.desc = (TextView) row.findViewById(R.id.desc);

      row.setTag(holder);
    } else {
      holder = (ItemHolder) row.getTag();
    }

    final BaseInfo item = getItem(position);
    if(item != null) {
      if(item.getTimestamp() == 0) {
        holder.timestamp.setVisibility(View.GONE);
      } else {
        String timestamp = SnowTimeUtil
            .getTimeAsString(STRING_FORMAT.TIMESTAMP_FORMAT, item.getTimestamp());
        holder.timestamp.setText(timestamp);
        holder.timestamp.setVisibility(View.VISIBLE);
      }

      holder.desc.setText(item.toString());
    }

    return row;
  }

  class ItemHolder {

    TextView timestamp;
    TextView desc;
  }

  ;
}</pre>
<br>

그 다음에는 ListView나 GridView를 배치해놓고 setAdapter() 메소드를 이용해서 위의 Adapter를 세팅해주면 됩니다.
