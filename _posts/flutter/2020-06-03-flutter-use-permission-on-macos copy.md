---
layout: post
title: MacOS에서 Sandbox Permission 추가하기
category: Flutter

tag: [Flutter]
---

MacOS 프로그램들은 기본적으로 Sandbox에서 동작하고 있습니다. 따라서 네트워크나 공유 자원 등에 접근할 때 
Permission을 획득해야 사용할 수 있는 경우가 있습니다.

대표적인 예로 이미지를 네트워크를 통해 가져오는 `Image.network` 함수를 사용할 경우 다음 오류가 발생합니다.

~~~
SocketException: Connection failed (OS Error: Operation not permitted, errno = 1)
~~~

이 경우 `macos/Runner/DebugProfile.entitlements` 파일에 다음 권한을 추가해주면 됩니다.

<pre class="prettyprint">
    &lt;key&gt;com.apple.security.network.client&lt;/key&gt;
    &lt;true/&gt;
</pre>

<br>

## MacOS의 Permission 리스트

MacOS에서 요구하는 Permission 리스트는 [여기](https://developer.apple.com/documentation/bundleresources/entitlements/app_sandbox)에서 확인하실 수
있습니다. 

또한 런타임 중에 확인해야 하는 `Hardened Runtime` 리스트는 [여기](https://developer.apple.com/documentation/security/hardened_runtime#3111190)에서 
확인 가능합니다.