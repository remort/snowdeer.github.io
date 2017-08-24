---
layout: post
title: onSaveInstanceState 활용하기 (Activity 데이터 유지)
category: Android

tag: [Android]
---
`onSaveInstanceState()` 메소드를 이용하면 Activity가 종료될 때 데이터를 저장할 수 있습니다.
일반적으로 사용자가 정상적인 행동으로 Activity를 종료할 때는 해당 이벤트를 미리 감지하고 
그에 맞는 대처를 해줄 수가 있지만, 실제로는 다양한 상황에서 Activity가 종료되는 현상이 발생할 수
있습니다.

<br>

# Activity가 종료되는 경우

* 사용자가 '뒤로 가기(Back)' 버튼을 눌러 Activity를 종료한 경우
* Activity가 백그라운드에 있을 때 시스템 메모리가 부족해진 경우(OS가 강제 종료시킴)
* 언어 설정을 변경할 때
* 화면을 가로/세로 회전할 때
* 폰트 크기나 폰트를 변경했을 때

<br>

# onSaveInstanceState 예제

<pre class="prettyprint">
package snowdeer.customviewexample;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

  static final String KEY_DATA = "KEY_DATA";
  TextView textView;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    textView = (TextView) findViewById(R.id.text_view);

    if (savedInstanceState != null) {
      String data = savedInstanceState.getString(KEY_DATA);
      textView.setText(data);
    }
  }

  @Override
  protected void onSaveInstanceState(Bundle outState) {
    super.onSaveInstanceState(outState);

    String data = textView.getText().toString();
    outState.putStringArrayList(KEY_DATA, data);
  }
}
</pre>

<br>

## 화면 회전시 라이프 사이클

화면을 회전할 때 발생하는 이벤트들은 다음과 같습니다. 화면을 회전할 때마다 Activity가 종료되고
새로 만들어지기 때문에 라이프 사이클을 잘 이해하고 대처하는 것이 좋을 것 같습니다.

onPause() → onSaveInstanceState() → onStop() → onDestory() → onCreate() → onStart() → onResume()

<br>

# Fragment에서의 onSaveInstanceState

Activity에서와 마찬가지로 Fragment에서도 `onSaveInstanceState` 메소드를 이용해서 기존 데이터를
유지하도록 할 수 있습니다.

Fragment 종료시 onPause() → onSaveInstanceState() 순서로 호출됩니다.

<br>

## Fragment 시작시 라이프 사이클

onAttach() → onCreate() → onCreateView() → onActivityCreated() → onStart() → onResume()

<br>

## Fragment 전환시 라이프 사이클

onPause() → onSaveInstanceState() → onStop() → onDestoryView() → onDestory() → onDetach() → onAttach() → onCreate() → onCreateView() → onActivityCreated() → onStart() → onResume()