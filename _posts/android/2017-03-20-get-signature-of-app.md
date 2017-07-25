---
layout: post
title: App에 사용된 Signature 알기
category: Android
tag: [Android]
---

App에 사용된 Signature를 획득하는 방법은 다음과 같습니다.

<br>
## 터미널 명령어
~~~
keytool -list -printcert -jarfile [파일 이름]
~~~

<br>
예를 들면, 파일 이름이 'app-release.apk'라고 하면 다음과 같이 입력하면 됩니다.
~~~
keytool -list -printcert -jarfile app-release.apk
~~~

![image -fullwidth]({{ site.baseurl }}/assets/2017-03-20-get-signature-of-app/01.png)
