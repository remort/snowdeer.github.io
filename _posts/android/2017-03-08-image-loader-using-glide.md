---
layout: post
title: URL 주소를 이용하여 ImageView에 이미지 채우기 (Glide 활용)
category: Android
tag: [Android, UX]
---

## URL 주소를 이용한 이미지 로딩
이미지의 URL 주소를 알고 있을 때, 그 이미지를 ImageView에 그리는 방법은 다음과 같습니다.
<ul>
 	<li>이미지를 다운로드한다.</li>
 	<li>다운로드한 이미지를 Bitmap 로딩하여 ImageView에 채워 넣는다.</li>
</ul>
이 경우, 구현해야 할 코드의 양도 꽤 많지만, 성능 문제를 위해 여러 가지 기법을 적용해야 합니다. 예를 들면, 다음과 같습니다.
<ul>
 	<li>ListView 등과 같이 여러 장의 이미지를 동시에 다운로드 할 수 있는 Thread 환경 구현</li>
 	<li>한 번 다운로드 받은 이미지는 다시 받지 않도록, 그리고 성능을 위해 Image Cache 구현</li>
 	<li>Thread로 다운로드하는 동안 화면에는 ProgressBar를 표시</li>
</ul>

<br>
기본적으로 위의 요소들이 있지만, 하나 하나를 파고들면 고려해볼 것이 상당히 많습니다.
예를 들면 Image Cache에 무한대의 이미지를 저장할 수 없기 때문에 Cache의 리소스 관리도 필요하고,
이미지 다운로드 및 로딩이 끝났을 때 어느 ImageView에 이미지를 그릴 것인지 관리하는
ImageView의 레퍼런스 저장도 필요합니다. 또한 이러한 것들은 WeakReference로 관리하여
해당 ImageView가 메모리에 여전히 남아 있는지 GC(Garbage Collector)로 청소가 끝났는지도
체크해야 합니다.

예전에 프로젝트를 진행하면서 직접 구현한 적이 있었는데, 신경쓸게 너무 많아서 고생을
한 적이 있습니다. 물론, 그만큼 공부는 많이 되긴 했습니다.

<br>
## Glide
이러한 것들을 한 꺼번에 해소해주기 위해서 오픈 소스 활용을 추천합니다. 오픈 소스는
유명한 것들이 많이 있는데, 5년 정도 전만 하더라도 저는
[Android Universal Image Loader](https://github.com/nostra13/Android-Universal-Image-Loader)를
애용하였습니다. 지금도 무난히 사용하기에 충분히 괜찮은 라이브러리입니다.
하지만, 지금은 [Glide](https://github.com/bumptech/glide) 라이브러리를 사용해보려고 합니다.
Glide는 구글이 인수한 Bump라는 회사에서 사용한 이미지 로딩 라이브러리입니다.

<br.
## 사용 방법
Gradle에 다음 코드를 추가해줍니다.
<pre class="prettyprint">dependencies{
    compile'com.github.bumptech.glide:glide:3.7.0'
    ...
}</pre>

<br>
그리고 별도로 XML 코드를 건드릴 필요 없고, 다음과 같은 Java 코드로 ImageView에 이미지를 불러오게 할 수 있습니다.
<pre class="prettyprint">Glide
    .with(mContext)
    .load(item.image)
    .centerCrop()
    //.placeholder(R.drawable.loading_spinner)
    .crossFade()
    .into(holder.image);
</pre>
이렇게만 하면 끝입니다. 더 자세한 사용법은 [여기에서 확인](https://github.com/bumptech/glide)할 수 있습니다.
