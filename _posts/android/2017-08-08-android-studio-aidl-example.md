---
layout: post
title: 모듈간 AIDL 통신 예제
category: Android
tag: [Android, Service]
---

안드로이드 스튜디오(Android Studio)에서 모듈간 AIDL 통신을 하는 예제 코드입니다. 기존에 Eclipse에서 프로젝트간 AIDL 통신을 할 때에 비해 약간의 차이점이 있긴 했습니다. (예를 들어 Android Studio에서는 `aidl` 전용 폴더를 만들어서 따로 관리한다던지 등)

여기서는 화면이 존재하지 않는 서비스(Service) 모듈과 별도의 어플리케이션간 AIDL 통신을 하는 코드를 다뤄 보겠습니다.

<br>

# 3개의 모듈 생성

먼저 안드로이드 스튜디오에서 3개의 모듈을 생성합니다. 각각의 모듈의 역할은 다음과 같습니다.

* app : 실제 Activity 등이 있는 어플리케이션
* sdk : AIDL 통신을 쉽게 할 수 있도록 도와주는 라이브러리
* service : GUI가 존재하지 않는 서비스

여기서 `sdk` 모듈은 따로 존재하지 않아도 됩니다. 하지만, AIDL 통신 부분은 처음 접하는 사람들에게는 다소 어려움이 존재하기 때문에 AIDL을 전혀 모르는 사람들도 쉽게 사용할 수 있도록 Wrapper 클래스화 시키는 것이 좋은 것 같습니다.

<br>

# 종속성 설정

각 모듈간 종속성은 다음과 같습니다.

* `app` 모듈은 `sdk` 모듈을 참조합니다.
* `service` 모듈은 `sdk` 모듈을 참조합니다.

즉, `app` 모듈과 `service` 모듈의 `build.gradle` 파일에 다음 코드를 추가해줍니다.

<pre class="prettyprint">
dependencies {
    ...
    compile project(':snowsdk');
}
</pre>

<br>

# AIDL 파일 생성

`sdk` 모듈의 `New` 메뉴에서 'Folder > AIDL'을 선택하면 AIDL 폴더가 만들어집니다. 

![Image](/assets/2017-08-08-android-studio-aidl-example/01.png)

그리고 그 안에 패키지 이름까지 지정해서 하위 폴더를 만듭니다.

![Image](/assets/2017-08-08-android-studio-aidl-example/02.png)

<br>

다음은 예제로 만들어 본 aidl 파일입니다. 어플리케이션에서 서비스로 메세지를 전송하는 `IServiceInterface.aidl`과 그 반대 방향으로 콜백(Callback) 이벤트를 알려주는 용도의 `IServiceCallback.aidl` 파일 2개를 만들었습니다.

## IServiceInterface.aidl

<pre class="prettyprint">
package snowdeer.sdk;
import snowdeer.sdk.IServiceCallback;

interface IServiceInterface {
    boolean registerCallback(in IServiceCallback cb);
    boolean unregisterCallback(in IServiceCallback cb);

    void sendMessage(int what, in Bundle bundle);
}
</pre>

<br>

## IServiceCallback.aidl

<pre class="prettyprint">
package snowdeer.sdk;

oneway interface IServiceCallback {
    void onMessageReceived(int what, in Bundle bundle);
}
</pre>

<br>

# Message 상수값 정의

굳이 이벤트 메세지의 상수값을 정의할 필요없이 필요한 API 개수만큼 aidl 코드안에 정의하는 편이 더 좋을 때도 있습니다. 

다만, API를 앞으로 빈번하게 수정해야 할 경우가 있다면 메세지 상수값을 정의해놓고 향후 상수값만 추가하면서 업데이트해나가는 편이 더 수월하더군요.

## SNOW_EVENT_MESSAGE.java

<pre class="prettyprint">
package snowdeer.sdk.constant;

public class SNOW_EVENT_MESSAGE {
  public static final int SAY_HELLO = 101;
  public static final int SAY_GOOD_BYE= 102;
}

</pre>

<br>

## SNOW_RESPONSE_MESSAGE.java

<pre class="prettyprint">
package snowdeer.sdk.constant;

public class SNOW_RESPONSE_MESSAGE {
  // TODO
}
</pre>

<br>

# Java 소스 구현

이제 위에서 만든 aidl 파일들을 이용해서 쉽게 통신할 수 있도록 인터페이스를 만들도록 합니다. 

가급적 클래스(class)가 아닌 인터페이스(interface)를 사용하는 편이 더 좋기 때문에 인터페이스로 만들도록 하겠습니다.

## SnowInterface.java

<pre class="prettyprint">
package snowdeer.sdk;

public interface SnowInterface {
  public interface OnEventListener {
    void onHelloRequested();
  }

  public void setOnEventListener(OnEventListener listener);

  void hello();
  void goodBye();
}
</pre>

<br>

## SnowInterfaceImpl.java

<pre class="prettyprint">
package snowdeer.sdk.impl;

import android.os.Bundle;
import android.os.RemoteException;
import android.util.Log;
import snowdeer.sdk.IServiceCallback;
import snowdeer.sdk.IServiceInterface;
import snowdeer.sdk.SnowInterface;
import snowdeer.sdk.constant.SNOW_EVENT_MESSAGE;

public class SnowInterfaceImpl implements SnowInterface {

  OnEventListener mOnEventListener;
  IServiceInterface mServiceInterface;

  public void setOnEventListener(OnEventListener listener) {
    mOnEventListener = listener;
  }

  public SnowInterfaceImpl(IServiceInterface iface) {
    mServiceInterface = iface;
  }

  public IServiceCallback getServiceCallback() {
    return mServiceCallback;
  }

  IServiceCallback.Stub mServiceCallback = new IServiceCallback.Stub() {

    @Override
    public void onMessageReceived(int what, Bundle bundle) throws RemoteException {
      switch (what) {
        // TODO
      }
    }
  };

  @Override
  public void hello() {
    Log.i("snowdeer", "[snowdeer] Hello!");
    try {
      Bundle bundle = new Bundle();
      mServiceInterface.sendMessage(SNOW_EVENT_MESSAGE.SAY_HELLO, bundle);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  @Override
  public void goodBye() {
    Log.i("snowdeer", "[snowdeer] Good Bye!");
    try {
      Bundle bundle = new Bundle();
      mServiceInterface.sendMessage(SNOW_EVENT_MESSAGE.SAY_GOOD_BYE, bundle);
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
</pre>

<br>

# ServiceProvider 구현

위에서 `SnowInterface.java` 와 `SnowInterfaceImpl.java` 코드를 만들었다면 이 인스턴스를 획득할 수 있는 ServiceProvider가 필요합니다. 아래와 같이 구현할 수 있습니다.

## SnowServiceProvider.java

<pre class="prettyprint">
package snowdeer.sdk;

import android.content.Context;
import snowdeer.sdk.impl.SnowServiceProviderImpl;

public final class SnowServiceProvider {
  private SnowServiceProvider() {}

  public interface OnServiceEventListener {
    void onConnected(SnowInterface iface);
    void onDisconnected();
  }

  public void setOnServiceEventListener(OnServiceEventListener listener) {
    SnowServiceProviderImpl.setOnServiceEventListener(listener);
  }

  public static boolean connect(Context context) {
    return SnowServiceProviderImpl.connect(context);
  }

  public static void disconnect(Context context) {
    SnowServiceProviderImpl.disconnect(context);
  }
}
</pre>

<br>

## SnowServiceProviderImpl.java

<pre class="prettyprint">
package snowdeer.sdk.impl;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.IBinder;
import android.os.RemoteException;
import snowdeer.sdk.IServiceInterface;
import snowdeer.sdk.SnowServiceProvider.OnServiceEventListener;

public class SnowServiceProviderImpl {
  static final String SERVICE_NAME = "snowdeer.service";
  static final String PACKAGE_NAME = "snowdeer.service";

  static IServiceInterface mIServiceInterface;
  static OnServiceEventListener mOnServiceEventListener;
  static SnowInterfaceImpl mServiceInterfaceImpl;
  static boolean isConnected = false;

  public static void setOnServiceEventListener(OnServiceEventListener listener) {
    mOnServiceEventListener = listener;
  }

  public static boolean connect(Context context) {
    if(isConnected) return false;

    Intent intent = new Intent(SERVICE_NAME);
    intent.setPackage(PACKAGE_NAME);
    return context.bindService(intent, mServiceConnection, Context.BIND_AUTO_CREATE);
  }

  public static void disconnect(Context context) {
    if(!isConnected) return;

    if(mServiceConnection != null) {
      context.unbindService(mServiceConnection);
    }

    isConnected = false;
    if(mOnServiceEventListener != null) {
      mOnServiceEventListener.onDisconnected();
    }
  }

  static final ServiceConnection mServiceConnection = new ServiceConnection() {
    @Override
    public void onServiceConnected(ComponentName name, IBinder service) {
      mIServiceInterface = IServiceInterface.Stub.asInterface(service);
      if(mServiceInterfaceImpl == null) {
        mServiceInterfaceImpl = new SnowInterfaceImpl(mIServiceInterface);
      }

      try {
        mIServiceInterface.registerCallback(mServiceInterfaceImpl.getServiceCallback());
      }
      catch (RemoteException e) {
        e.printStackTrace();
        return;
      }

      if(mOnServiceEventListener != null) {
        mOnServiceEventListener.onConnected(mServiceInterfaceImpl);
      }
    }

    @Override
    public void onServiceDisconnected(ComponentName name) {
      try {
        mIServiceInterface.unregisterCallback(mServiceInterfaceImpl.getServiceCallback());
      }
      catch (RemoteException e) {
        e.printStackTrace();
      }

      if(mOnServiceEventListener != null) {
        mOnServiceEventListener.onDisconnected();
      }

      mServiceInterfaceImpl = null;
      mIServiceInterface = null;
      isConnected = false;
    }
  };
}
</pre>

<br>

이제 SDK 쪽은 끝이 났습니다. AIDL 통신을 하는 서비스 쪽 코드를 살펴보도록 하겠습니다.

# 서비스 구현

## AndroidManifest.xml

<pre class="prettyprint">
&lt;?xml version="1.0" encoding="utf-8"?&gt;
&lt;manifest xmlns:android="http://schemas.android.com/apk/res/android"
  package="snowdeer.service"&gt;

  &lt;application
    android:allowBackup="true"
    android:icon="@mipmap/ic_launcher"
    android:label="@string/app_name"
    android:roundIcon="@mipmap/ic_launcher_round"
    android:supportsRtl="true"
    android:theme="@style/AppTheme"&gt;
    &lt;service
      android:enabled="true"
      android:exported="true"
      android:name=".SnowService"&gt;
      &lt;intent-filter&gt;
        &lt;action android:name="snowdeer.service" /&gt;
        &lt;category android:name="android.intent.category.DEFAULT" /&gt;
      &lt;/intent-filter&gt;
    &lt;/service&gt;
  &lt;/application&gt;

&lt;/manifest&gt;
</pre>

<br>

## SnowService.java

<pre class="prettyprint">
package snowdeer.service;

import android.app.Service;
import android.content.Intent;
import android.os.Bundle;
import android.os.IBinder;
import android.os.RemoteException;
import android.support.annotation.IntDef;
import snowdeer.sdk.IServiceCallback;
import snowdeer.sdk.IServiceInterface;

public class SnowService extends Service {

  public SnowService() {
  }

  @Override
  public void onCreate() {
    super.onCreate();
  }

  @Override
  public IBinder onBind(Intent intent) {
    return mBinder;
  }

  @Override
  public boolean onUnbind(Intent intent) {
    return super.onUnbind(intent);
  }

  @Override
  public int onStartCommand(Intent intent, int flags, int startId) {
    return super.onStartCommand(intent, flags, startId);
  }

  @Override
  public void onDestroy() {
    super.onDestroy();
  }

  final IServiceInterface.Stub mBinder = new IServiceInterface.Stub() {

    @Override
    public boolean registerCallback(IServiceCallback cb) throws RemoteException {
      return EventCallbackHandler.getInstance().registerCallback(cb);
    }

    @Override
    public boolean unregisterCallback(IServiceCallback cb) throws RemoteException {
      return EventCallbackHandler.getInstance().unregisterCallback(cb);
    }

    @Override
    public void sendMessage(int what, Bundle bundle) throws RemoteException {
      switch (what) {
        // TODO
      }
    }
  };
}
</pre>

<br>

## EventCallbackHandler.java

<pre class="prettyprint">
package snowdeer.service;

import android.os.Bundle;
import android.os.RemoteCallbackList;
import snowdeer.sdk.IServiceCallback;

public class EventCallbackHandler {

  static EventCallbackHandler mInstance = new EventCallbackHandler();

  private EventCallbackHandler() {}

  public static EventCallbackHandler getInstance() {
    return mInstance;
  }

  final RemoteCallbackList&lt;IServiceCallback&gt; mIServiceCallbackList = new RemoteCallbackList<>();

  public boolean registerCallback(IServiceCallback cb) {
    return mIServiceCallbackList.register(cb);
  }

  public boolean unregisterCallback(IServiceCallback cb) {
    return mIServiceCallbackList.unregister(cb);
  }

  public void sendCallbackEvent(int what, Bundle bundle) {
    int receiverCount = mIServiceCallbackList.beginBroadcast();
    for (int i = 0; i < receiverCount; i++) {
      try {
        IServiceCallback cb = mIServiceCallbackList.getBroadcastItem(i);

        cb.onMessageReceived(what, bundle);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    mIServiceCallbackList.finishBroadcast();
  }
}
</pre>

<br>