---
layout: post
title: Google API Key 등록 및 사용하기
category: Android
tag: [google, api key]
---

Google Awarenes API를 간단히 사용해보려고 합니다.
하지만, Awareness API를 쓰기 위해서는 
Google로부터 Google API Key를 등록 및 승인받아야 합니다.

그 방법을 한 번 포스팅해보도록하겠습니다.

일단 [Google Developer Console](https://console.developers.google.com/)에 접속합니다.

![image -fullwidth]({{ site.baseurl }}/assets/2016-12-04-android-how-to-use-google-api-key/1.png)

<br>

여기에서 왼쪽 편의 메뉴에서 '라이브러리'를 선택합니다.

![image -fullwidth]({{ site.baseurl }}/assets/2016-12-04-android-how-to-use-google-api-key/2.png)

그리고 원하는 API를 선택해줍니다.
저 같은 경우는 Awareness API를 사용할 것이기 때문에 Awareness API를 선택했습니다.

검색을 이용하면 좀 더 편리하게 API를 찾을 수 있습니다.


<br>

![image -fullwidth]({{ site.baseurl }}/assets/2016-12-04-android-how-to-use-google-api-key/3.png)

화면 위쪽에 있는 '사용 설정' 글씨를 클릭해줍니다. (버튼이 아니라서 처음엔 찾기 힘들 수도 있습니다;;)

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2016-12-04-android-how-to-use-google-api-key/4.png)

화면 위쪽에 '사용자 인증 정보 만들기' 버튼을 선택해줍니다.

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2016-12-04-android-how-to-use-google-api-key/5.png)

'어떤 사용자 정보가 필요한가요' 버튼을 눌러줍니다.

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2016-12-04-android-how-to-use-google-api-key/6.png)

API 키가 생성됩니다. 여기에서 '키 제한하기'를 눌러서 키 사용의 제한을 걸어두도록 하겠습니다.

<br>

![image -fullwidth]({{ site.baseurl }}/assets/2016-12-04-android-how-to-use-google-api-key/7.png)

저는 Android 용으로 사용하기 위해서 'Android 앱'을 선택했습니다.
각자 용도로 선택하시면 됩니다.

'Android 앱'을 선택하면 
Android App의 패키지 이름과 Signed Key의 SHA1 값을 요구합니다.
Signed Key의 SHA1은 화면에 있는 명령어를 커맨드 라인에서 입력하여 얻을 수 있습니다.
'mystore.keystore' 대신 자신의 key 이름을 입력하면 됩니다.

<pre class="prettyprint" style="font-size:0.7em;">
signingConfigs{
keytool -list -v -keystore mystore.keystore
</pre>

저 같은 경우는 다음과 같은 출력 결과가 나왔고,
이 중에서 SHA1 값만 있으면 됩니다.

![image -fullwidth]({{ site.baseurl }}/assets/2016-12-04-android-how-to-use-google-api-key/8.png)

<br>

App의 패키지 이름과 Signed Key의 SHA1 값을 획득하면, 위의 화면에서
'패키지 이름 및 지문 추가' 버튼을 눌러서 각 값을 입력해주면 됩니다.

이제, API Key 획득 및 인증 작업은 끝났습니다.  
본격적인 App 개발을 하시면 됩니다. ^^;
