---
layout: post
title: Chrome 브라우저 프록시(Proxy) 설정
category: Linux
tag: [Linux]
---
# Proxy Setting for Ubuntu Chrome

크롬 브라우저는 기본적으로 Ubuntu 시스템에 설정된 Proxy 설정을 사용합니다. 즉, 브라우저 메뉴에서 Proxy 설정 항목에 가더라도 시스템 Proxy 설정을 따른다는 설명만 나오고 설정할 수 있는 것이 없습니다.

보통은 큰 문제없이 시스템 프록시 설정값을 따라 크롬이 실행되는 경우가 많지만, 가끔씩 (뭐가 꼬인건지 몰라도) 크롬 브라우저가 시스템 프록시 설정값을 사용하지 않는 경우가 있습니다. 이 경우는 다음과 같은 방법을 통해 수동 설정을 해줄 수 있습니다.

<br>

## 터미널에서 사용하는 경우

터미널에서 사용하는 경우 다음과 같은 명령을 이용할 수 있습니다.

<pre class="prettyprint">
$ google-chrome --proxy-server="proxy_address:port"
</pre>

`alias`를 이용하면 좀 더 편리하게 사용할 수도 있습니다. (부팅시마다 리셋되기 때문에 `.bashrc`나 `.profile`에 기록해둡시다.)

<pre class="prettyprint">
$ alias google-chrome='google-chrome --proxy-server="proxy_address:port"'
</pre>

<br>

## 데스크탑에서 사용하는 경우

하지만, 대부분은 데스크탑에서 크롬을 실행할 가능성이 높기 때문에 데스크탑 바로가기 설정을 바꿔줘야 합니다.

<pre class="prettyprint">
$ sudo nano /usr/share/applications/google-chrome.desktop
</pre>

여기에서 `Exec`로 실행하는 부분을 찾아서 전부 `--proxy-server="proxy_address:port"` 옵션을 달아주면 됩니다.

