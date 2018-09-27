---
layout: post
title: 어느 날 갑자기 외장 디스크가 Read Only가 된 경우 해결법
category: Linux
tag: [리눅스]
---
# 어느 날 갑자기 외장 디스크가 Read Only가 된 경우 해결법

어느 날 갑자기 외부 저장 매체가 Read Only 상태가 되어버렸을 때의 현상 수정하는 방법입니다. 아마 불필요하거나 손상된 파일이 많이 생긴 바람에 이런 현상이 생긴 것 같습니다. 

터미널에서 다음과 같이 명령을 수행합니다.

<br>

## 경로 확인

<pre class="prettyprint">
$ sudo df -Th
Filesystem     Type      Size  Used Avail Use% Mounted on
udev           devtmpfs  7.8G     0  7.8G   0% /dev
tmpfs          tmpfs     1.6G  1.9M  1.6G   1% /run
...
/dev/sdd1      vfat      932G   36G  896G   4% /media/snowdeer/PORTABLESSD
</pre>

여기에서 외장 디스크의 `Filesystem` 이름과 `Mounted` 되어있는 경로 이름을 확인합니다.

<br>

## Unmout

<pre class="prettyprint">
$ sudo umount /media/snowdeer/PORTABLESSD
</pre>

<br>

## 불필요한 파일 정리

<pre class="prettyprint">
$ sudo dosfsck -a /dev/sdd1

fsck.fat 4.1 (2017-01-24)
0x41: Dirty bit is set. Fs was not properly unmounted and some data may be corrupt.
 Automatically removing dirty bit.
/.Trash-1000/files/res
 Start does point to root directory. Deleting dir. 
/.Trash-1000/files/java
 Start does point to root directory. Deleting dir. 
Reclaimed 185 unused clusters (6062080 bytes) in 179 chains.
Free cluster summary wrong (29345789 vs. really 29345582)
  Auto-correcting.
Performing changes.
/dev/sdd1: 440012 files, 1170741/30516323 clusters
</pre>

<br>

## 불필요한 파일 확인

위 명령어를 수행하면 외장 디스크 루트 디렉토리 안에 수 많은 `*.REC` 파일들이 생겼음을 확인할 수 있습니다.
해당 파일들을 전부 지우고 파일 탐색 브라우저를 종료했다가 다시 실행하면 Read Only 상태가 해제되었음을 확인할 수 있습니다.