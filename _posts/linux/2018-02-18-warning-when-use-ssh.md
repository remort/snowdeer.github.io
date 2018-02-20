---
layout: post
title: ssh 접속시 WARNING - REMOTE HOST IDENTIFICATION HAS CHANGED! 발생하는 경우
category: Linux
tag: [Linux, ssh]
---
#  WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED!

<pre class="prettyprint">
$ ssh snowdeer@13.14.15.16

 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 
 @ WARNING: REMOTE HOST IDENTIFICATION HAS CHANGED! @ @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ 
 
 IT IS POSSIBLE THAT SOMEONE IS DOING SOMETHING NASTY! 
 Someone could be eavesdropping on you right now (man-in-the-middle attack)! 
 It is also possible that a host key has just been changed. 
 The fingerprint for the RSA key sent by the remote host is
 SHA256:abcdefghijklmnopqrstuvwxyz.
 Please contact your system administrator. 
 Add correct host key in /home/snowdeer/.ssh/known_hosts to get rid of this message. 
 Offending RSA key in /home/snowdeer/.ssh/known_hosts:3 
   remove with:
   ssh-keygen -f "/home/snowdeer/.ssh/known_hosts" -R 13.14.15.16
   ECDSA host key for 13.14.15.16 has changed and you have requested strict checking.
 Host key verification failed.
</pre>

이 오류는 기존에 접속한 IP Address의 서버와 공개키를 교환한 상태에서 같은 주소로 다른 서버에 접속했기 때문에 발생하는 오류입니다.

이 경우는 다음 명령어를 이용해서 해결할 수 있습니다.

<pre class="prettyprint">
ssh-keygen -R 13.14.15.16
</pre>