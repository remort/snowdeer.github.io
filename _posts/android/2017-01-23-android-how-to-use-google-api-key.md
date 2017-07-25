---
layout: post
title: Google API Key 등록하기
category: Android
tag: [Android, Google API]
---

Google Awareness API를 이용한 간단한 프로그램을 개발할 일이 생겼습니다.
하지만, Awareness API를 쓰기 위해서는 Google로부터 Google API Key를 등록 및 승인받아야만
사용이 가능했습니다. 그래서 여기서는 Google API Key를 등록하는 방법을 포스팅해보록 하겠습니다.

<br>
## Developer Console 접속
일단 [구글 개발자 콘솔](https://console.developers.google.com/)에 접속한다.

![image -fullwidth]({{ site.baseurl }}/assets/2017-02-23-android-how-to-use-google-api-key/01.png)

여기에서 왼쪽 편의 메뉴에서 `라이브러리`를 선택합니다.

![image -fullwidth]({{ site.baseurl }}/assets/2017-02-23-android-how-to-use-google-api-key/02.png)

그리고 원하는 API를 선택합니다. 저는 Awareness API를 사용할 것이기 때문에 Awareness API를 선택했습니다.
API 종류가 많기 때문에 검색을 이용하면 좀 더 편리하게 API를 찾을 수 있습니다.

![image -fullwidth]({{ site.baseurl }}/assets/2017-02-23-android-how-to-use-google-api-key/03.png)

화면 가운데 위쪽에 있는 `사용 설정` 글씨를 클릭해줍니다. 버튼 형태가 아니라서 찾기가 어려웠네요.

![image -fullwidth]({{ site.baseurl }}/assets/2017-02-23-android-how-to-use-google-api-key/04.png)

화면 오른편 위쪽에 `사용자 인증 정보 만들기` 버튼을 클릭합니다.

![image -fullwidth]({{ site.baseurl }}/assets/2017-02-23-android-how-to-use-google-api-key/05.png)

`어떤 사용자 정보가 필요한가요` 버튼을 눌러줍니다.

![image -fullwidth]({{ site.baseurl }}/assets/2017-02-23-android-how-to-use-google-api-key/06.png)

드디어 API 키가 생성되었다. 여기에서 `키 제한하기`를 눌러서 키 사용의 제한을 걸어두도록 합니다.

![image -fullwidth]({{ site.baseurl }}/assets/2017-02-23-android-how-to-use-google-api-key/07.png)

저는 Android 용으로 사용하기 위해서 `Android 앱`을 선택했습니다.

`Android 앱`을 선택하면 Android App의 패키지 이름과 Signed Key의 SHA1 값을 요구합니다.
Signed Key의 SHA1은 화면에 있는 명령어를 커맨드 라인에서 입력하여 얻을 수 있습니다. 
`mystore.keystore` 대신 자신의 key 이름을 입력하면 됩니다.
<br>
<pre class="prettyprint">signingConfigs{
keytool -list -v -keystore mystore.keystore
</pre>
그러면 다음과 같은 출력 결과가 나오고, 이 중에서 SHA1 값만 있으면 됩니다.

![image -fullwidth]({{ site.baseurl }}/assets/2017-02-23-android-how-to-use-google-api-key/08.png)

App의 패키지 이름과 Signed Key의 SHA1 값을 획득하면, 위의 화면에서
`패키지 이름 및 지문 추가` 버튼을 눌러서 각 값을 입력해주면 됩니다.
