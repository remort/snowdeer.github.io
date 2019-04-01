---
layout: post
title: Ubuntu 18.04에 Libre Office 수동으로 설치하는 방법

category: Linux
permalink: /mac-os/:year/:month/:day/:title/
tag: [리눅스]
---
## Ubuntu 18.04에 Libre Office 수동으로 설치하는 방법

<pre class="prettyprint">
sudo add-apt-repository ppa:libreoffice/ppa
sudo apt-get update
sudo apt-get install libreoffice
</pre>

<br>

## 삭제 방법

<pre class="prettyprint">
sudo apt-get remove libreoffice-core
sudo apt-get remove --purge libreoffice-core
</pre>