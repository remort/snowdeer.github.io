---
layout: post
title: ftp 서버 설치 방법
category: Linux
tag: [리눅스]
---
# ftp 서버 설치 방법

<pre class="prettyprint">
sudo apt install vsftpd
</pre>

<br>

## 설정

`/etc/vsftpd.conf` 파일을 내용을 수정하면 됩니다.

예를 들어 초기 접속 디렉토리는 

~~~
local_root=/home/snowdeer/ftp_home
~~~

과 같이 파일 끝에 추가해주면 됩니다.

익명 접속 허용 설정은

~~~
anonymous_enable=YES
anon_upload_enable=YES
anon_other_write_enable=YES
anon_world_readable_only=NO
anon_umask=0022
~~~

등과 같이 할 수 있으며, 파일 수정 후 `sudo service vsftpd restart` 해주어야 반영이 됩니다.