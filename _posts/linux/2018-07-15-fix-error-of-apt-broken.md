---
layout: post
title: apt-get -f install 오류 메시지나는 경우
category: Linux
tag: [리눅스, Ubuntu]
---
# apt 명령어 실행시 apt-get -f install 오류 발생하는 경우

`apt` 명령어를 실행할 때마다 아래와 같은 오류 메시지가 계속 발생하는 경우가 있을 수 있습니다.

~~~
You might want to run 'apt-get -f install' to correct these.
The following packages have unmet dependencies:
 pcl : Depends: libvtk6.2 but it is not installed
       Depends: libvtk6.2-qt but it is not installed
E: Unmet dependencies. Try using -f.
~~~

이 경우는 다음 명령어로 해결할 수 있습니다.

<pre class="prettyprint"
sudo apt --fix-broken install
sudo apt-get update
sudo apt-get upgrade
</pre>