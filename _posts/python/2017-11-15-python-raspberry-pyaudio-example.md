---
layout: post
title: 라즈베리파이에서의 pyAudio 모듈 사용 예제
category: Python
tag: [Python, 라즈베리파이]
---
# 라즈베리파이에서 pyAudio 모듈 사용하기

라즈베리파이에서 `pyAudio` 모듈을 사용해서 오디오 녹음을 하고 재생하는 예제 코드입니다. 오디오 파일 녹음과 재생은 Streaming 형태로 이루어지며, 각각 `wav` 파일로 저장하고 재생합니다.

<br>

## pyAudio 모듈 설치

먼저 터미널에서 다음 명령어를 입력해서 `pyAudio`를 설치합니다.

~~~
sudo apt install python3-pyaudio
~~~

<br>

## pyaudio_recorder.py

<pre class="prettyprint">
import pyaudio
import wave

CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
RECORD_SECONDS = 5
WAVE_OUTPUT_FILENAME = "output.wav"

p = pyaudio.PyAudio()

stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

print("Start to record the audio.")

frames = []

for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)

print("Recording is finished.")

stream.stop_stream()
stream.close()
p.terminate()

wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
wf.setnchannels(CHANNELS)
wf.setsampwidth(p.get_sample_size(FORMAT))
wf.setframerate(RATE)
wf.writeframes(b''.join(frames))
wf.close()
</pre>

<br>

## pyaudio_player.py

<pre class="prettyprint">
import pyaudio
import wave
import sys

CHUNK = 1024

if len(sys.argv) < 2:
    print("Plays a wave file.\n\nUsage: %s filename.wav" % sys.argv[0])
    sys.exit(-1)

wf = wave.open(sys.argv[1], 'rb')

p = pyaudio.PyAudio()

stream = p.open(format=p.get_format_from_width(wf.getsampwidth()),
                channels=wf.getnchannels(),
                rate=wf.getframerate(),
                output=True)

data = wf.readframes(CHUNK)

while data != '':
    stream.write(data)
    data = wf.readframes(CHUNK)

stream.stop_stream()
stream.close()

p.terminate()
</pre>