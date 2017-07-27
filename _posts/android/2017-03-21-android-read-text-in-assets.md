---
layout: post
title: Assets의 텍스트 파일 불러오기
category: Android
tag: [Android, Assets]
---

Assets 폴더에 들어있는 텍스트 파일의 내용을 String으로 읽는 코드입니다.

Android Studio 기준으로 Assets 폴더는 `/app/src/main/assets`에 위치하며,
네비게이션 뷰를 `Project`로 설정한 다음 폴더를 추가하면 됩니다.

본 예제는 텍스트 파일을 읽어오는 코드이지만, 다른 파일들에도 적용할 수 있습니다.

예제 코드는 다음과 같습니다.

<br>

# PolicyActivity.java

<pre class="prettyprint">import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.text.Html;
import android.widget.TextView;
import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.OnClick;
import com.lnc.cuppadata.R;
import com.lnc.cuppadata.gui.account.RegistrationActivity;
import java.io.BufferedReader;
import java.io.InputStreamReader;

public class PolicyActivity extends AppCompatActivity {

  @BindView(R.id.policy_view)
  protected TextView policyView;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_policy);

    ButterKnife.bind(this);

    loadPolicy();
  }

  @OnClick(R.id.ok)
  protected void onOkButtonPressed() {
    Intent intent = new Intent(PolicyActivity.this, RegistrationActivity.class);
    startActivity(intent);

    finish();
  }

  @OnClick(R.id.cancel)
  protected void onCancelButtonPressed() {
    finish();
  }

  private void loadPolicy() {
    try {
      String policy = readFromAssets("policy.txt");

      policyView.setText(Html.fromHtml(policy));
    } catch(Exception e) {
      e.printStackTrace();
    }
  }

  private String readFromAssets(String filename) throws Exception {
    BufferedReader reader = new BufferedReader(new InputStreamReader(getAssets().open(filename)));

    StringBuilder sb = new StringBuilder();
    String line = reader.readLine();
    while(line != null) {
      sb.append(line);
      line = reader.readLine();
    }
    reader.close();
    return sb.toString();
  }
}</pre>
