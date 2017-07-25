---
layout: post
title: Butter Knife 라이브러리 사용법
category: Android
tag: [Android, Open Source]
---

안드로이드 GUI 개발을 하다보면, 가장 귀찮은 것 중 하나가 화면에 컴포넌트를 하나
추가하는 것입니다.

예를 들어 화면에 버튼을 추가한다고 하면 다음과 같은 작업들을 거쳐야 합니다.
<ul>
 	<li>XML Layout에 버튼을 추가한다.</li>
 	<li>해당 Activity(Fragment)에서 그 컴포넌트에 해당하는 변수를 추가한다.
ex) private Button button;</li>
 	<li>findViewById를 통해 그 컴포넌트를 변수에 할당한다.
ex) button = (Button)findViewById(R.id.button)</li>
 	<li>그 버튼에 setOnClickListener를 통해 이벤트를 등록한다.</li>
</ul>
컴포넌트가 몇 개 안 될 경우는 큰 문제가 안되는데, 컴포넌트가 많아질수록
위 코드들은 감당이 안 될 정도로 많아지고 복잡해집니다. 이럴 때 'Butter Knife' 라이브러리를
사용하면 코드량이 상당히 줄어듭니다. (사실, 이런 식으로 별거 아닌 코드를 Wrapping 하는
  외부 라이브러리를 쓰는 걸 선호하진 않지만 Butter Knife는 생각보다 괜찮은 것 같아서
  최근에 조금씩 사용을 해보고 있습니다.)

간단한 예제 코드로 알아보도록 하겠습니다. 더 자세한 내용은
[공식 홈페이지를 참조](http://jakewharton.github.io/butterknife/)하시면 됩니다.
버전에 따라 문법이 조금씩 바뀌고 있는데, 여기서는 현재 최신 버전인 8.5.1 기준으로 사용해보겠습니다.

<br>
## gradle 세팅
<pre class="prettyprint">dependencies {
    ...
    compile 'com.jakewharton:butterknife:8.5.1'
    annotationProcessor 'com.jakewharton:butterknife-compiler:8.5.1'
    ...
}
</pre>
<br>
## activity_main.xml
<pre class="prettyprint">&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
  xmlns:tools="http://schemas.android.com/tools"
  android:id="@+id/activity_main"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:paddingTop="@dimen/activity_vertical_margin"
  android:paddingBottom="@dimen/activity_vertical_margin"
  android:paddingLeft="@dimen/activity_horizontal_margin"
  android:paddingRight="@dimen/activity_horizontal_margin"
  android:orientation="vertical"
  tools:context="com.snowdeer.butterknife.sample.MainActivity"&gt;

  &lt;TextView
    android:id="@+id/title"
    android:textStyle="bold"
    android:layout_width="wrap_content"
    android:layout_height="wrap_content"
    android:text="Title"
    android:textSize="32sp" /&gt;

  &lt;Button
    android:id="@+id/button"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="Click Me!" /&gt;

  &lt;ImageView
    android:id="@+id/imageview"
    android:layout_width="match_parent"
    android:layout_height="match_parent" /&gt;

&lt;/LinearLayout&gt;</pre>
<br>
## strings.xml
<pre class="prettyprint">&lt;resources&gt;
  &lt;string name="app_name"&gt;ButterKnifeSample&lt;/string&gt;
  &lt;string name="title"&gt;Hello. Butter Knife&lt;/string&gt;
&lt;/resources&gt;</pre>
<br>
## MainActivity.java
여기서 가장 중요한 부분은 onCreate() 함수 내의 다음 코드입니다.
<pre class="prettyprint">ButterKnife.bind(this);</pre>
위 코드를 호출해주어야 각 View들이 바인딩(Binding)이 됩니다.

<br>
<pre class="prettyprint">import android.graphics.drawable.Drawable;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.ImageView;
import android.widget.TextView;
import butterknife.BindDrawable;
import butterknife.BindString;
import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.OnClick;

public class MainActivity extends AppCompatActivity {

  @BindView(R.id.title)
  TextView titleView;
  @BindView(R.id.imageview)
  ImageView imageView;
  @BindString(R.string.title)
  String title;
  @BindDrawable(R.mipmap.ic_launcher)
  Drawable drawable;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    ButterKnife.bind(this);
  }

  @OnClick(R.id.button)
  void onButtonClicked() {
    titleView.setText(title);
    imageView.setImageDrawable(drawable);
  }

  ;
}</pre>
<br>
## Fragment 에서의 예제
<pre class="prettyprint">public class FancyFragment extends Fragment {

  @BindView(R.id.button1)
  Button button1;
  @BindView(R.id.button2)
  Button button2;

  @Override
  public View onCreateView(LayoutInflater inflater, ViewGroup container,
      Bundle savedInstanceState) {
    View view = inflater.inflate(R.layout.fancy_fragment, container, false);
    ButterKnife.bind(this, view);
    // TODO Use fields...
    return view;
  }
}</pre>
<br>
## ListView 등에서 사용하는 Adapter 및 ViewHolder 예제
<pre class="prettyprint">public class MyAdapter extends BaseAdapter {

  @Override
  public View getView(int position, View view, ViewGroup parent) {
    ViewHolder holder;
    if(view != null) {
      holder = (ViewHolder) view.getTag();
    } else {
      view = inflater.inflate(R.layout.whatever, parent, false);
      holder = new ViewHolder(view);
      view.setTag(holder);
    }

    holder.name.setText("John Doe");
    // etc...

    return view;
  }

  static class ViewHolder {

    @BindView(R.id.title)
    TextView name;
    @BindView(R.id.job_title)
    TextView jobTitle;

    public ViewHolder(View view) {
      ButterKnife.bind(this, view);
    }
  }
}</pre>
<br>
## 이벤트 등록 예제
<pre class="prettyprint">@OnClick(R.id.submit)
public void submit(View view) {
  // TODO submit data to server...
}

@OnClick(R.id.submit)
public void submit() {
  // TODO submit data to server...
}

@OnClick(R.id.submit)
public void sayHi(Button button) {
  button.setText("Hello!");
}

@OnClick({R.id.door1, R.id.door2, R.id.door3})
public void pickDoor(DoorView door) {
  if(door.hasPrizeBehind()) {
    Toast.makeText(this, "You win!", LENGTH_SHORT).show();
  } else {
    Toast.makeText(this, "Try again", LENGTH_SHORT).show();
  }
}</pre>
