---
layout: post
title: 라즈베리파이에서 오디오 녹음하기
category: Python
tag: [Python, 라즈베리파이]
---
# 라즈베리파이에서 오디오 파일로 녹음하기

## pyalsaaudio 모듈 설치

라즈베리파이(Raspberry PI)에서 Python을 이용해서 오디오 녹음을 하는 예제입니다.

라즈베리파이도 기본적으로 Linux 기반이다보니 'ALSA' 오디오 라이브러리를 사용합니다. Python에서도 마찬가지로 `pyalsaaudio`라는 모듈을 설치해야 오디오 장비에 접근할 수 있습니다.

터미널에서 다음 명령어로 `pyalsaaudio` 모듈을 설치합니다.

~~~
sudo pip3 install pyalsaaudio
~~~

만약 `pyalsaaudio` 설치 도중에

~~~
alsaaudio.c:28:28: fatal error: alsa/asoundlib.h: No such file or directory
     #include <alsa/asoundlib.h>
                                ^
    compilation terminated.
    error: command 'arm-linux-gnueabihf-gcc' failed with exit status 1
~~~

위와 같은 오류 메세지가 발생했을 경우에는 다음 명령어를 이용해서 `libasound2-dev`를 먼저 설치해줘야 합니다.

~~~
sudo apt-get install libasound2-dev
~~~

<br>

## 오디오 녹음하는 Python 예제 코드

<pre class="prettyprint">
from __future__ import print_function

import sys
import time
import getopt
import alsaaudio


def usage():
    print('usage: recordtest.py [-d &lt;device&gt;] &lt;file&gt;', file=sys.stderr)
    sys.exit(2)


if __name__ == '__main__':

    device = 'default'

    opts, args = getopt.getopt(sys.argv[1:], 'd:')
    for o, a in opts:
        if o == '-d':
            device = a

    if not args:
        usage()

    f = open(args[0], 'wb')

    inp = alsaaudio.PCM(alsaaudio.PCM_CAPTURE, alsaaudio.PCM_NONBLOCK, device=device)

    inp.setchannels(1)
    inp.setrate(44100)
    inp.setformat(alsaaudio.PCM_FORMAT_S16_LE)
    inp.setperiodsize(160)

    loops = 1000000
    while loops > 0:
        loops -= 1
        l, data = inp.read()

        if l:
            f.write(data)
            time.sleep(.001)
</pre>

<br>

## aplay로 녹음된 파일 확인

`aplay`로 정상적으로 녹음이 되었는지 확인합니다.

~~~
aplay -t raw -c 1 -f S16_LE -r 44100 test.raw
~~~
