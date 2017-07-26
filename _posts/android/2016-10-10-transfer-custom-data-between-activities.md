---
layout: post
title: Activity간 데이터 전달하기 (사용자 정의 클래스)
category: Android
tag: [Android]
---

Activity간 사용자 정의 클래스 데이터를 전달하는 예제 코드입니다.
Activity간 데이터를 이동시킬 때는 그 데이터를 Serializable(또는 Parcelable) 해주어야
데이터가 전달됩니다.

데이터를 Serializable 화 하는 건 다음과 같이 Serializable 인터페이스를 구현해주는
것으로 충분합니다.

<br>

## MyObject.java

<pre class="prettyprint">class MyObject implements Serializable {

  int nID;
  String strName;

  MyObject(int _id, String _Name) {
    nID = _id;
    strName = _Name;
  }
};</pre>
<br>

Activity 코드들은 각각 다음과 같습니다.
<br>

## firstActivity.java

<pre class="prettyprint">import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

public class firstActivity extends Activity {

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main);

    Button btnTest = (Button) findViewById(R.id.btnLaunch2ndActivity);
    btnTest.setOnClickListener(myButtonClick);
  }

  Button.OnClickListener myButtonClick = new Button.OnClickListener() {
    public void onClick(View v) {
      Intent intent = new Intent(firstActivity.this, secondActivity.class);

      MyObject obj = new MyObject(314, "SnowDeer");
      intent.putExtra("StringData_1", "첫번째 String 데이터");
      intent.putExtra("StringData_2", "두번째 String 데이터");
      intent.putExtra("ObjectData", obj);

      startActivity(intent);
    }
  };
}
</pre>
<br>

## secondActivity.java

<pre class="prettyprint">import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

public class secondActivity extends Activity {

  @Override
  public void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.main_2);

    TextView tvView = (TextView) findViewById(R.id.tvTextView);

    Intent intent = getIntent();
    if(intent != null) {
      String strText = "";
      MyObject obj;

      obj = (MyObject) intent.getSerializableExtra("ObjectData");

      strText = "StringData_1 : " + intent.getStringExtra("StringData_1") + "\n" +
          "StringData_2 : " + intent.getStringExtra("StringData_2") + "\n" +
          "MyObject.ID : " + obj.nID + "\n" +
          "MyObject.Name : " + obj.strName;

      tvView.setText(strText);
    }
  }

  Button.OnClickListener myButtonClick = new Button.OnClickListener() {
    public void onClick(View v) {
      Intent intent = new Intent(secondActivity.this, firstActivity.class);
      startActivity(intent);
    }
  };
}</pre>
