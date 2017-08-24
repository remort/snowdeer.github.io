---
layout: post
title: Main Thread가 강제 종료되지 않도록 하는 방법
category: Android
tag: [Android, Thread]
---
Main Thread가 강제 종료되지 않도록 Exception 처리를 해줄 수 있는 방법이 있습니다.
하지만, ANR(Application Not Responding) 등을 막을 수는 없고, 애초에 강제 종료될 상황을
만들지 않는 것이 더 중요한 것 같습니다.

# 예제

<pre class="prettyprint">
public class MainActivity extends AppCompatActivity {

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    Thread.currentThread().setUncaughtExceptionHandler(new UncaughtExceptionHandler() {
      @Override
      public void uncaughtException(Thread t, Throwable e) {
        Log.i("snowdeer", "Uncaught Exception !!");
        e.printStackTrace();
      }
    });

    ArrayList&lt;String&gt; list = null;
    int count = list.size();
  }
}
</pre>

`UncaughtExceptionHandler`를 먼저 등록하고 뒷 부분에 강제로 오류가 발생하는 코드를
넣었습니다.

<br>

## 응용 - 오류 메세지를 파일에 저장

여기서 조금 더 응용해서 오류 메세지가 발생했을 때 파일에 저장하는 코드를 작성해보도록 하겠습니다.
여기서는 파일에 저장을 했지만, 서버에 전송을 하는 등으로 추가 응용해서 활용 가능합니다.

<pre class="prettyprint">
import android.Manifest.permission;
import android.content.Context;
import android.content.pm.PackageManager;
import android.support.annotation.NonNull;
import android.support.v7.app.AlertDialog;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Toast;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.lang.Thread.UncaughtExceptionHandler;

public class MainActivity extends AppCompatActivity {

  Thread.UncaughtExceptionHandler defaultExceptionHandler = Thread
      .getDefaultUncaughtExceptionHandler();
  static final String ERROR_LOG_FILE_NAME = "error_log.txt";

  FileOutputStream fos;
  FileInputStream fis;
  BufferedReader br;

  static final int PERMISSION_REQUEST_CODE = 100;

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    setContentView(R.layout.activity_main);

    if (checkSelfPermission(permission.WRITE_EXTERNAL_STORAGE)
        != PackageManager.PERMISSION_GRANTED) {
      String[] permissions = new String[]{permission.WRITE_EXTERNAL_STORAGE};
      requestPermissions(permissions, PERMISSION_REQUEST_CODE);
    }

    findViewById(R.id.btn_error).setOnClickListener(new OnClickListener() {
      @Override
      public void onClick(View v) {
        throw new RuntimeException();
      }
    });

    Thread.setDefaultUncaughtExceptionHandler(new UncaughtExceptionHandler() {
      @Override
      public void uncaughtException(Thread t, Throwable e) {
        e.printStackTrace();

        try {
          fos = openFileOutput(ERROR_LOG_FILE_NAME, Context.MODE_PRIVATE);
          fos.write(e.getClass().getName().getBytes());
          fos.close();
        } catch (Exception ex) {
          ex.printStackTrace();
        }

        defaultExceptionHandler.uncaughtException(t, e);
      }
    });

    File file = getFileStreamPath(ERROR_LOG_FILE_NAME);

    if (file.exists()) {
      StringBuilder sb = new StringBuilder("Previous Error:\n");
      try {
        fis = new FileInputStream(file);
        br = new BufferedReader(new InputStreamReader(fis));
        String line;

        while ((line = br.readLine()) != null) {
          sb.append(line).append("\n");
        }
        br.close();
        fis.close();

        Log.i("snowdeer", sb.toString());

        new AlertDialog.Builder(MainActivity.this, R.style.Theme_AppCompat)
            .setMessage(sb.toString())
            .setPositiveButton("Ok", null)
            .show();

        deleteFile(ERROR_LOG_FILE_NAME);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  @Override
  public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
      @NonNull int[] grantResults) {
    super.onRequestPermissionsResult(requestCode, permissions, grantResults);

    switch (requestCode) {
      case PERMISSION_REQUEST_CODE:
        Toast.makeText(getApplicationContext(), "Permission 완료", Toast.LENGTH_SHORT).show();
        break;
    }
  }
}
</pre>