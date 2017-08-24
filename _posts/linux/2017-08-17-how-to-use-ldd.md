---
layout: post
title: Ldd 사용법
category: Linux
tag: [리눅스 명령어]
---
# 실행 파일은 있는데 필요한 라이브러리가 없는 경우

실행 파일은 있는데 필요한 라이브러리가 없는 경우 다음과 같은 오류 메세지를 출력합니다.

~~~
./snowdeer: error while loading shared libraries: libglog.so.0: cannot open shared object file: No such file or directory
~~~

위 오류 메세지는 `libglog.so.0` 이라는 동적 라이브러리가 없다는 메세지입니다. 이 경우 해당 라이브러리를 시스템 폴더(주로 '/usr/lib' 아래)에 복사해주면 해결이 됩니다.

다만, 프로그램을 실행했을 때 오류 메세지로는 필요한 라이브러리를 1개씩밖에 확인이 안되게 때문에 많이 번거롭습니다. 이 경우 `ldd` 명령어를 이용하면 필요한 라이브러리 리스트를 모두 확인할 수 있습니다.

<br>

# List Dynamic Dependencies`

`ldd` 명령어는 프로그램이 사용하고 있는 공유 라이브러리(Shared Library) 리스트를 출력합니다. 'List Dynamic Dependencies'의 약자입니다.

<pre class="prettyprint">
? ldd ./snowdeer
	/usr/lib/arm-linux-gnueabihf/libarmmem.so (0xb6f3f000)
        libpthread.so.0 => /lib/arm-linux-gnueabihf/libpthread.so.0 (0xb6f04000)
        libglog.so.0 => not found
        libprotobuf.so.9 => not found
        libboost_system.so.1.55.0 => /usr/lib/arm-linux-gnueabihf/libboost_system.so.1.55.0 (0xb6ef1000)
        libcaffe.so.1.0.0 => not found
        libasound.so.2 => /usr/lib/arm-linux-gnueabihf/libasound.so.2 (0xb6e17000)
        libopencv_core.so.2.4 => /usr/lib/arm-linux-gnueabihf/libopencv_core.so.2.4 (0xb6bfe000)
        libopencv_highgui.so.2.4 => /usr/lib/arm-linux-gnueabihf/libopencv_highgui.so.2.4 (0xb6bad000)
        libopencv_objdetect.so.2.4 => /usr/lib/arm-linux-gnueabihf/libopencv_objdetect.so.2.4 (0xb6b32000)
        libopencv_imgproc.so.2.4 => /usr/lib/arm-linux-gnueabihf/libopencv_imgproc.so.2.4 (0xb68e3000)
        libopencv_video.so.2.4 => /usr/lib/arm-linux-gnueabihf/libopencv_video.so.2.4 (0xb6889000)
        libwiringPi.so => /usr/lib/libwiringPi.so (0xb686b000)
        ...
</pre>

위와 같이 `snowdeer`라는 파일이 사용하는 라이브러리 리스트가 출력이 되며, 라이브러리 이름 옆에 위치가 출력이 됩니다. 만약 라이브러리가 없다면 이름 옆에 `=> not found` 라는 메세지가 출력됩니다. 즉, 필요한 라이브러리들을 확인해서 시스템 폴더 아래 복사를 해주면 됩니다.

<br>

# 사용 예제

~~~
ldd ./snowdeer | grep "not"
~~~

다만 `ldd` 명령어로도 필요한 라이브러리들이 한 번에 안 나오는 경우도 종종 있는 것 같습니다. 그럴 때는 필요한 라이브러리들을 먼저 설치한다음 다시 `ldd` 명령어로 확인할 수 있습니다.