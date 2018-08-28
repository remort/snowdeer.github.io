---
layout: post
title: Message Handler 구현 예제 (Looper 활용)
category: Android
tag: [Android]
---
# Message Handler 구현 예제

`Thread`를 상속받아 구현했으며, Thread 내부에서 `Handler`를 생성하기 위해서 `Looper`를 이용한 예제입니다.

<br>

## MessageHandler.java

<pre class="prettyprint">
import android.os.Handler;
import android.os.Looper;
import android.os.Message;

public abstract class MessageHandler extends Thread {

  Handler mHandler;

  @Override
  public void init() {
    this.start();
  }

  @Override
  public void fin() {
    this.interrupt();
  }

  @Override
  public void run() {
    Looper.prepare();

    mHandler = new Handler() {
      @Override
      public void handleMessage(Message msg) {
        handle(msg);
      }
    };

    Looper.loop();
  }

  public abstract void handle(Message msg);

  public synchronized void sendMessage(Message msg) {
    mHandler.sendMessage(msg);
  }
}
</pre>

<br>

## 사용 방법

추상 클래스이기 때문에 상속받아서 구체화된 클래스를 만들어 사용하면 됩니다. 사용 예제는 다음과 같습니다.

<pre class="prettyprint">
public class SampleComponent {

  public o() {
    messageHandler = new MessageHandler() {
      @Override
      public void handle(Message msg) {
        handleMessage(msg);
      }
    };
  }

  public void handleMessage(Message msg) {
    switch (msg.what) {
      case MESSAGE.DEBUG_ON:
        // TODO 
        break;
      case MESSAGE.DEBUG_OFF:
        // TODO
        break;
    }
  }
}
</pre>