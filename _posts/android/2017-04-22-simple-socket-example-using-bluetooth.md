---
layout: post
title: 블루투스 기반 소켓 통신 예제
category: Android
tag: [Android, Bluetooth]
---

안드로이드에서 블루투스(Bluetooth)를 이용하여 간단하게 단말간 통신할 수 있는 코드를 포스팅해봅니다.

단말 2개가 필요하며, 한 쪽 단말은 서버, 한 쪽 단말은 클라이언트가 됩니다.
또한, 두 단말간의 페어링(Pairing) 단계는 이미 되었다고 가정하고 건너뛰도록 하겠습니다.

<br>

# 권한(Permission) 추가

블루투스 통신을 사용하기 위해서는 아래 두 권한이 필요합니다. `AndroidManifest.xml` 파일에 추가해주시면 됩니다.
<pre class="prettyprint">&lt;uses-permission android:name="android.permission.BLUETOOTH_ADMIN" /&gt;
&lt;uses-permission android:name="android.permission.BLUETOOTH" /&gt;

</pre>

<br>

# 상수값 설정

<pre class="prettyprint">public class SnowConstant {
  public static final UUID BLUETOOTH_UUID_SECURE = UUID.fromString( "fa87c0d0-afac-11de-8a39-0800200c9a66" );
  public static final UUID BLUETOOTH_UUID_INSECURE = UUID.fromString( "8ce255c0-200a-11e0-ac64-0800200c9a66" );
}</pre>

<br>

# 서버

## fragment_btserver.xml

<pre class="prettyprint">&lt;LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:orientation="vertical"&gt;

  &lt;Button
    android:id="@+id/start_server"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="Start Server" /&gt;

  &lt;Button
    android:id="@+id/stop_server"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="Stop Server" /&gt;

  &lt;Button
    android:id="@+id/send_welcome"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="Send Welcome Message" /&gt;

  &lt;TextView
    android:id="@+id/logview"
    android:layout_width="match_parent"
    android:layout_height="match_parent" /&gt;

&lt;/LinearLayout&gt;</pre>
<br>

## BTServerFragment.java

<pre class="prettyprint">public class BTServerFragment extends Fragment {

  Handler handler = new Handler();
  TextView logView;
  StringBuilder sbLog = new StringBuilder();

  BluetoothServer btServer = new BluetoothServer();

  public BTServerFragment() {
    // Required empty public constructor
  }


  @Override
  public View onCreateView(LayoutInflater inflater, ViewGroup container,
      Bundle savedInstanceState) {
    // Inflate the layout for this fragment
    View view = inflater.inflate(R.layout.fragment_btserver, container, false);

    logView = (TextView) view.findViewById(R.id.logview);
    btServer.setOnEventLogListener(new OnEventLogListener() {
      @Override
      public void onLog(String message) {
        log(message);
      }
    });

    view.findViewById(R.id.start_server).setOnClickListener(mOnClickListener);
    view.findViewById(R.id.stop_server).setOnClickListener(mOnClickListener);
    view.findViewById(R.id.send_welcome).setOnClickListener(mOnClickListener);

    return view;
  }

  private View.OnClickListener mOnClickListener = new View.OnClickListener() {

    @Override
    public void onClick(View v) {
      switch (v.getId()) {
        case R.id.start_server:
          btServer.startServer();
          break;
        case R.id.stop_server:
          btServer.stopServer();
          break;
        case R.id.send_welcome:
          btServer.sendWelcome();
          break;
      }
    }
  };

  void log(String message) {
    sbLog.append(message + "\n");

    if (logView != null) {
      handler.post(new Runnable() {
        @Override
        public void run() {
          logView.setText(sbLog.toString());
        }
      });
    }

    Log.i("", "[snowdeer] " + message);
  }
}</pre>
<br>

## BluetoothServer.java

<pre class="prettyprint">public class BluetoothServer {

  BluetoothAdapter btAdapter = BluetoothAdapter.getDefaultAdapter();
  BluetoothServerSocket acceptSocket;

  AcceptThread acceptThread;

  DataInputStream is = null;
  DataOutputStream os = null;

  public interface OnEventLogListener {

    void onLog(String message);
  }

  OnEventLogListener mOnEventLogListener = null;

  public void setOnEventLogListener(OnEventLogListener listener) {
    mOnEventLogListener = listener;
  }

  void printLog(String message) {
    if (mOnEventLogListener != null) {
      mOnEventLogListener.onLog(message);
    }
  }

  public void startServer() {
    stopServer();

    printLog("Start Server.");
    acceptThread = new AcceptThread();
    acceptThread.start();
  }

  public void stopServer() {
    if (acceptThread == null) {
      return;
    }

    try {
      acceptThread.stopThread();
      acceptThread.join(1000);
      acceptThread.interrupt();
    } catch (Exception e) {
      e.printStackTrace();
    }

    printLog("Stop Server.");
  }

  class AcceptThread extends Thread {

    boolean isRunning = false;

    AcceptThread() {
      try {
        acceptSocket = null;
        acceptSocket = btAdapter.listenUsingRfcommWithServiceRecord("SnowDeerBluetoothSample",
            SnowConstant.BLUETOOTH_UUID_INSECURE);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    public void run() {
      BluetoothSocket socket;

      isRunning = true;

      while (isRunning) {
        try {
          printLog("Waiting clients...");
          socket = acceptSocket.accept();
        } catch (Exception e) {
          e.printStackTrace();
          break;
        }

        if (socket != null) {
          printLog("Client is connected.");

          CommunicationThread commThread = new CommunicationThread(socket);
          commThread.start();
        }
      }
    }

    public void stopThread() {
      isRunning = false;
      try {
        acceptSocket.close();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  class CommunicationThread extends Thread {

    final BluetoothSocket socket;
    boolean isRunning = false;

    CommunicationThread(BluetoothSocket socket) {
      this.socket = socket;
    }

    public void run() {
      isRunning = true;

      try {
        is = new DataInputStream(socket.getInputStream());
        os = new DataOutputStream(socket.getOutputStream());
      } catch (Exception e) {
        e.printStackTrace();
      }

      while (isRunning) {
        try {
          String message = is.readUTF();
          printLog(message);
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    }

    public void stopThread() {
      isRunning = false;
      try {
        is.close();
        os.close();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  public void sendWelcome() {
    if (os == null) {
      return;
    }

    try {
      os.writeUTF("Welcome");
      os.flush();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}</pre>
<br>

# 클라이언트

## fragment_btclient.xml

<pre class="prettyprint">&lt;LinearLayout xmlns:android="http://schemas.android.com/apk/res/android"
  android:layout_width="match_parent"
  android:layout_height="match_parent"
  android:orientation="vertical"&gt;

  &lt;Button
    android:id="@+id/search_devices"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="Search Devices" /&gt;

  &lt;Button
    android:id="@+id/connect_to_server"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="Connect to Server" /&gt;

  &lt;Button
    android:id="@+id/disconnect_from_server"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="Disconnect from Server" /&gt;

  &lt;Button
    android:id="@+id/send_hello"
    android:layout_width="match_parent"
    android:layout_height="wrap_content"
    android:text="Send Hello Message" /&gt;

  &lt;TextView
    android:id="@+id/logview"
    android:layout_width="match_parent"
    android:layout_height="match_parent" /&gt;

&lt;/LinearLayout&gt;</pre>
<br>

## BTClientFragment.java

<pre class="prettyprint">public class BTClientFragment extends Fragment {

  Handler handler = new Handler();
  TextView logView;
  StringBuilder sbLog = new StringBuilder();

  BluetoothClient btClient = new BluetoothClient();

  public BTClientFragment() {
    // Required empty public constructor
  }


  @Override
  public View onCreateView(LayoutInflater inflater, ViewGroup container,
      Bundle savedInstanceState) {
    View view =  inflater.inflate(R.layout.fragment_btclient, container, false);

    logView = (TextView) view.findViewById(R.id.logview);
    btClient.setOnEventLogListener(new OnEventLogListener() {
      @Override
      public void onLog(String message) {
        log(message);
      }
    });

    view.findViewById(R.id.search_devices).setOnClickListener(mOnClickListener);
    view.findViewById(R.id.connect_to_server).setOnClickListener(mOnClickListener);
    view.findViewById(R.id.disconnect_from_server).setOnClickListener(mOnClickListener);
    view.findViewById(R.id.send_hello).setOnClickListener(mOnClickListener);

    return view;
  }

  private View.OnClickListener mOnClickListener = new View.OnClickListener() {

    @Override
    public void onClick(View v) {
      switch (v.getId()) {
        case R.id.search_devices:
          btClient.searchPairedDevice();
          break;
        case R.id.connect_to_server:
          btClient.connectToServer();
          break;
        case R.id.disconnect_from_server:
          btClient.disconnectFromServer();
          break;
        case R.id.send_hello:
          btClient.sendHello();
          break;
      }
    }
  };

  void log(String message) {
    sbLog.append(message + "\n");

    if (logView != null) {
      handler.post(new Runnable() {
        @Override
        public void run() {
          logView.setText(sbLog.toString());
        }
      });
    }

    Log.i("", "[snowdeer] " + message);
  }
}</pre>
<br>

## BluetoothClient.java

<pre class="prettyprint">public class BluetoothClient {

  BluetoothAdapter btAdapter = BluetoothAdapter.getDefaultAdapter();
  BluetoothDevice mTargetDevice = null;

  DataInputStream is = null;
  DataOutputStream os = null;

  ClientThread clientThread;

  public interface OnEventLogListener {

    void onLog(String message);
  }

  OnEventLogListener mOnEventLogListener = null;

  public void setOnEventLogListener(OnEventLogListener listener) {
    mOnEventLogListener = listener;
  }

  void printLog(String message) {
    if (mOnEventLogListener != null) {
      mOnEventLogListener.onLog(message);
    }
  }

  public void searchPairedDevice() {
    Set&lt;BluetoothDevice&gt; pairedDevices = btAdapter.getBondedDevices();
    printLog("paired Devices count : " + pairedDevices.size());

    if (pairedDevices.size() &gt; 0) {
      for (BluetoothDevice deivce : pairedDevices) {
        printLog(deivce.getName() + " / " + deivce.getAddress());

        mTargetDevice = deivce;

        printLog(deivce.getName() + "(" + deivce.getAddress() + ") is found !!!");
      }
    }
  }

  public void connectToServer() {
    disconnectFromServer();

    if (mTargetDevice == null) {
      return;
    }

    printLog("Connect to Server.");
    clientThread = new ClientThread(mTargetDevice);
    clientThread.start();
  }

  public void disconnectFromServer() {
    if (clientThread == null) {
      return;
    }

    try {
      clientThread.stopThread();
      clientThread.join(1000);
      clientThread.interrupt();
    } catch (Exception e) {
      e.printStackTrace();
    }

    printLog("Disconnect from Server.");
  }


  class ClientThread extends Thread {

    BluetoothSocket socket;
    BluetoothDevice mDevice;

    ClientThread(BluetoothDevice device) {
      try {
        mDevice = device;

        //socket = null;

        socket = device.createRfcommSocketToServiceRecord(SnowConstant.BLUETOOTH_UUID_INSECURE);
      } catch (Exception e) {
        e.printStackTrace();
      }
    }

    public void run() {
      btAdapter.cancelDiscovery();

      while (true) {
        try {
          printLog("try to connect to Server...");
          socket.connect();
        } catch (Exception e) {
          e.printStackTrace();
          try {
            socket.close();
          } catch (Exception ex) {
            ex.printStackTrace();
          }
          break;
        }

        if (socket != null) {
          printLog("Connected !!!");

          try {
            is = new DataInputStream(socket.getInputStream());
            os = new DataOutputStream(socket.getOutputStream());

          } catch (Exception e) {
            e.printStackTrace();
          }

          while (true) {
            try {
              String message = is.readUTF();
              printLog(message);
            } catch (Exception e) {
              e.printStackTrace();
            }
          }
        }

        try {
          sleep(1000);
        } catch (Exception e) {
          e.printStackTrace();
        }
      }
    }

    public void stopThread() {
      try {
        socket.close();
        is.close();
        os.close();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }
  }

  public void sendHello() {
    if (os == null) {
      return;
    }

    try {
      os.writeUTF("Hello");
      os.flush();
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}</pre>
