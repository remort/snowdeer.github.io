---
layout: post
title: Network 상태 모니터링
category: Android
tag: [Android]
---

안드로이드 버전에 따라 모니터링 방법이 조금씩 다르기 때문에 코드 내에서 분기를 태워줍니다.

## AndroidManifest.xml

먼저 `AndroidManifest.xml` 파일에 `permission`을 추가해줍니다.

<pre class="prettyprint">
  &lt;uses-permission android:name="android.permission.INTERNET"/&gt;
  &lt;uses-permission android:name="android.permission.WRITE_EXTERNAL_STORAGE"/&gt;
  &lt;uses-permission android:name="android.permission.ACCESS_WIFI_STATE"/&gt;
  &lt;uses-permission android:name="android.permission.ACCESS_NETWORK_STATE"/&gt;
</pre>

<br>

## WifiConnectivityMonitor.kt

<pre class="prettyprint">
import android.annotation.TargetApi
import android.arch.lifecycle.LiveData
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Context.CONNECTIVITY_SERVICE
import android.content.Intent
import android.content.IntentFilter
import android.net.ConnectivityManager
import android.net.Network
import android.net.NetworkInfo
import android.net.NetworkRequest
import android.os.Build

class WifiConnectivityMonitor(private val ctx: Context) : LiveData&lt;Boolean&gt;() {

    private var connectivityManager: ConnectivityManager = ctx.getSystemService(CONNECTIVITY_SERVICE) as ConnectivityManager

    private lateinit var connectivityManagerCallback: ConnectivityManager.NetworkCallback

    override fun onActive() {
        super.onActive()
        updateConnection()
        when {
            Build.VERSION.SDK_INT >= Build.VERSION_CODES.N ->
                connectivityManager.registerDefaultNetworkCallback(getConnectivityManagerCallback())
            Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP -> lollipopNetworkAvailableRequest()
            else -> {
                if (Build.VERSION.SDK_INT < Build.VERSION_CODES.LOLLIPOP) {
                    ctx.registerReceiver(networkReceiver, IntentFilter("android.net.conn.CONNECTIVITY_CHANGE"))
                }
            }
        }
    }

    override fun onInactive() {
        super.onInactive()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {
            connectivityManager.unregisterNetworkCallback(connectivityManagerCallback)
        } else {
            ctx.unregisterReceiver(networkReceiver)
        }
    }

    @TargetApi(Build.VERSION_CODES.LOLLIPOP)
    private fun lollipopNetworkAvailableRequest() {
        val builder = NetworkRequest.Builder()
                .addTransportType(android.net.NetworkCapabilities.TRANSPORT_CELLULAR)
                .addTransportType(android.net.NetworkCapabilities.TRANSPORT_WIFI)
        connectivityManager.registerNetworkCallback(builder.build(), getConnectivityManagerCallback())
    }

    private fun getConnectivityManagerCallback(): ConnectivityManager.NetworkCallback {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.LOLLIPOP) {

            connectivityManagerCallback = object : ConnectivityManager.NetworkCallback() {
                override fun onAvailable(network: Network?) {
                    postValue(true)
                }

                override fun onLost(network: Network?) {
                    postValue(false)
                }
            }
            return connectivityManagerCallback
        } else {
            throw IllegalAccessError("Should not happened")
        }
    }

    private val networkReceiver = object : BroadcastReceiver() {
        override fun onReceive(context: Context, intent: Intent) {
            updateConnection()
        }
    }

    private fun updateConnection() {
        val activeNetwork: NetworkInfo? = connectivityManager.activeNetworkInfo
        postValue(activeNetwork?.isConnected == true)
    }
}
</pre>

<br>

## WifiConnectivityMonitor 클래스 사용 예시

<pre class="prettyprint">
private fun initWifiMonitor() {
    val connectivityMonitor = WifiConnectivityMonitor(this)
        connectivityMonitor.observe(this, Observer { isConnected ->
            isConnected?.let {
                if (it) {
                    ip_address.text = getWifiIPAddress()
                    port.visibility = View.VISIBLE
                } else {
                    ip_address.text = getString(R.string.wifi_not_available)
                    port.visibility = View.GONE
                }
            }
        })
}

private fun getWifiIPAddress(): String {
    val wm = getSystemService(Service.WIFI_SERVICE) as WifiManager
    return Formatter.formatIpAddress(wm.connectionInfo.ipAddress)
}
</pre>