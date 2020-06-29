---
layout: post
title: Flutter Network Image 예제
category: Flutter

tag: [Flutter]
---

## Flutter Network Image

네트워크로 이미지 받으려면 먼저 Permission을 부여해줘야 합니다.

MacOS 기준으로 `macos/Runner/DebugProfile.entitlements` 파일에 권한을 줄 수 있습니다.

<pre class="prettyprint">
    &lt;key&gt;com.apple.security.network.client&lt;/key&gt;
    &lt;true/&gt;
</pre>

안드로이드 같은 경우는 `AndroidManifest.xml` 파일에 인터넷 권한을 부여하면 됩니다.

<br>

## 예제 코드

<pre class="prettyprint">
import 'package:flutter/material.dart';

void main() => runApp(MyApp());

class MyApp extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Network Image Example',
      home: Scaffold(
        appBar: AppBar(
          title: Text('Network Image Example'),
        ),
        body: NetworkImageWidget(),
      ),
    );
  }
}

class NetworkImageWidget extends StatelessWidget {
  final url = 'https://snowdeer.github.io/public/img/hello_page.jpg';

  @override
  Widget build(BuildContext context) {
    return Center(
      child: Image.network(url),
    );
  }
}
</pre>