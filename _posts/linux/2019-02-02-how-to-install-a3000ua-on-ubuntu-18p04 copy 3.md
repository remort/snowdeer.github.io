---
layout: post
title: ipTIME A3000UA 리눅스 드라이버 설치 방법 

category: Linux
permalink: /mac-os/:year/:month/:day/:title/
tag: [리눅스]
---
# ipTIME A3000UA 리눅스 드라이버 설치 방법 

ipTIME의 A3000UA는 USB 무선랜카드입니다. Windows에는 드라이버 설치 파일을 제공하는데, 리눅스용은 아쉽게도 지원하지 않기 때문에 수동으로 드라이버를 설치해야 합니다.

다행히 해당 제품 내부에는 Realtek의 8812BU 라는 칩셋을 사용하기 때문에 해당 칩셋의 드라이버를 설치하면 됩니다.

<br>

## 설치 명령어

<pre class="prettyprint">
sudo apt update

sudo apt install dkms bc git

git clone https://github.com/cilynx/rtl88x2BU_WiFi_linux_v5.3.1_27678.20180430_COEX20180427-5959

sudo dkms add ./rtl88x2BU_WiFi_linux_v5.3.1_27678.20180430_COEX20180427-5959

sudo dkms install -m rtl88x2bu -v 5.3.1

sudo modprobe 88x2bu
</pre>

참고 : https://askubuntu.com/questions/1018375/how-do-i-install-driver-for-rtl88x2bu