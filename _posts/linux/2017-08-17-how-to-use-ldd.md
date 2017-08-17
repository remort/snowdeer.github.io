---
layout: post
title: Ldd 사용법
category: Linux
tag: [Linux 명령어]
---
# List Dynamic Dependencies`

`ldd` 명령어는 프로그램이 사용하고 있는 공유 라이브러리(Shared Library) 리스트를 출력하는 명령어입니다. 'List Dynamic Dependencies'의 약자입니다.

<pre class="prettyprint">
? ldd /usr/sbin/apache2
	linux-vdso.so.1 =>  (0x00007fff85bff000)
	libpcre.so.3 => /lib/libpcre.so.3 (0x00007ff366142000)
	libaprutil-1.so.0 => /usr/lib/libaprutil-1.so.0 (0x00007ff365f1f000)
	libapr-1.so.0 => /usr/lib/libapr-1.so.0 (0x00007ff365ce8000)
	libpthread.so.0 => /lib/libpthread.so.0 (0x00007ff365acb000)
	libc.so.6 => /lib/libc.so.6 (0x00007ff365745000)
	libuuid.so.1 => /lib/libuuid.so.1 (0x00007ff36553f000)
	librt.so.1 => /lib/librt.so.1 (0x00007ff365337000)
	libcrypt.so.1 => /lib/libcrypt.so.1 (0x00007ff3650fe000)
	libdl.so.2 => /lib/libdl.so.2 (0x00007ff364ef9000)
	libexpat.so.1 => /lib/libexpat.so.1 (0x00007ff364cd0000)
	/lib64/ld-linux-x86-64.so.2 (0x00007ff3665f2000)
</pre>

특히 `-d` 옵션을 함께 사용하면 해당 프로그램이 사용하고 있는 공유 라이브러리 중 '*.so' 파일의 존재 여부까지 확인할 수 있습니다. 만약 해당 파일이 없다면 라이브러리 이름 옆에 '==> not found' 라는 메세지가 출력되어서 현재 설치가 필요한 라이브러리 리스트를 확인할 수 있습니다.

<br>

# 사용 예제

~~~
ldd -d snowdeer | grep "not"
~~~
