---
layout: post
title: Geocoder를 이용한 주소 및 위도/경도 변환 예제
category: Android
tag: [Android]
---

안드로이드에서는 위도/경도를 이용해서 주소값을 획득하거나 반대로 주소값을 이용해서 위도/경도를 획득할 수 있는 `Geocoder`라는 클래스를 제공하고 있습니다.

에제 코드는 다음과 같습니다.

먼저 인터넷이 되어야 하기 때문에 `AndroidManifest.xml`에 다음 권한을 추가합니다.

# Permission 추가

<pre class="prettyprint">
  &lt;uses-permission android:name="android.permission.ACCESS_FINE_LOCATION" /&gt;
  &lt;uses-permission android:name="android.permission.INTERNET" /&gt;
</pre>


<br>

# GeocodeUtil.java

<pre class="prettyprint">
package snowdeer.utils;

import android.content.Context;
import android.location.Address;
import android.location.Geocoder;
import java.util.ArrayList;
import java.util.List;

public class GeoCodeUtil {
  final Geocoder geocoder;

  public static class GeoLocation {

    double latitude;
    double longitude;

    public GeoLocation(double latitude, double longitude) {
      this.latitude = latitude;
      this.longitude = longitude;
    }
  }

  public GeoCodeUtil(Context context) {
    geocoder = new Geocoder(context);
  }

  public ArrayList&lt;GeoLocation&gt; getGeoLocationListUsingAddress(String address) {
    ArrayList&lt;GeoLocation&gt; resultList = new ArrayList&lt;&gt;();


    try {
      List&lt;Address&gt; list = geocoder.getFromLocationName(address, 10);

      for (Address addr : list) {
        resultList.add(new GeoLocation(addr.getLatitude(), addr.getLongitude());
      }
    } catch (Exception e) {
      e.printStackTrace();
    }

    return resultList;
  }

  public ArrayList&lt;String&gt; getAddressListUsingGeolocation(GeoLocation location) {
    ArrayList&lt;String&gt; resultList = new ArrayList&lt;&gt;();

    try {
      List&lt;Address&gt; list = geocoder.getFromLocation(location.latitude, location.longitude, 10);

      for (Address addr : list) {
        resultList.add(addr.toString());
      }
    } catch (Exception e) {
      e.printStackTrace();
    }

    return resultList;
  }
}
</pre>