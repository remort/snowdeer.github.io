---
layout: post
title: OkHttp를 활용한 GET, POST
category: Android
tag: [Android, 네트워크, Open Source]
---

[OkHttp](http://square.github.io/okhttp/)는 HTTP 및 HTTP/2 통신을 보다 쉽게 할 수 있도록
다양한 기능을 제공해주는 Android 및 Java 용 라이브러리입니다. 오픈 소스로 되어 있으며
[GitHub에서 소스 확인 및 다운로드](https://github.com/square/okhttp)할 수 있습니다.

<br>

# Gradle 설정

build.gradle에 다음 라인을 추가해줍니다.

~~~
compile 'com.squareup.okhttp3:okhttp:3.6.0'
~~~

<br>

# GET 예제

<pre class="prettyprint">public boolean getUserInfo(final Context context) {

  try {
    OkHttpClient client = new OkHttpClient();

    String url = SERVER_CONFIGURATION.ADDRESS + ":" +
        SERVER_CONFIGURATION.PORT + "/v1/userinfo";

    Request request = new Request.Builder()
        .addHeader("Authorization", "TEST AUTH")
        .url(url)
        .build();
    Response response = client.newCall(request)
        .execute();

    String result = response.body().string();

    Gson gson = new Gson();
    UserInfo info = gson.fromJson(result, UserInfo.class);

    Log.i("id: " + info.id);
    Log.i("name: " + info.name);

    return true;
  } catch(Exception e) {
    e.printStackTrace();
  }

  return false;
}</pre>
<br>

# POST 예제

<pre class="prettyprint">private boolean updatetMetaInfo(JsonItemMetaInfo metaInfo) {

  try {
    OkHttpClient client = new OkHttpClient();

    String url = SERVER_CONFIGURATION.ADDRESS + ":" +
        SERVER_CONFIGURATION.PORT + "/v1/updateMetaInfo";

    Gson gson = new Gson();
    String json = gson.toJson(metaInfo);

    Request request = new Request.Builder()
        .url(url)
        .post(RequestBody.create(MediaType.parse("application/json"), json))
        .build();

    Response response = client.newCall(request).execute();

    Log.i("request : " + request.toString());
    Log.i("Response : " + response.toString());

    return true;
  } catch(Exception e) {
    e.printStackTrace();
  }

  return false;
}</pre>
<br>

# PUT 예제

<pre class="prettyprint">public boolean registerAppToken(JsonToken token) {
  try {
    OkHttpClient client = new OkHttpClient();

    String url = SERVER_CONFIGURATION.ADDRESS + ":" +
        SERVER_CONFIGURATION.PORT + "/v1/registerAppToken";

    Gson gson = new Gson();
    String json = gson.toJson(token);

    Request request = new Request.Builder()
        .addHeader("key", "Content-Type")
        .addHeader("value", "application/json")
        .addHeader("description", "")
        .url(url)
        .put(RequestBody.create(MediaType.parse("application/json"), json))
        .build();

    Response response = client.newCall(request).execute();

    Log.i("request : " + request.toString());
    Log.i("Response : " + response.toString());

    return true;
  } catch(Exception e) {
    e.printStackTrace();
  }

  return false;
}</pre>
