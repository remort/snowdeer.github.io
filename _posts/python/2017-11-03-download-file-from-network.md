---
layout: post
title: 네트워크 상의 파일 다운로드
category: Python
tag: [Python]
---

Python 3.x 기반의 코드입니다. 인터넷 등 네트워크 상에 있는 파일을 다운로드하는 샘플 코드입니다.

<br>

# 예제 코드

## urlretrieve() 함수를 이용하여 바로 파일에 저장

<pre class="prettyprint">
import urllib.request

url = "http://snowdeer.github.io/public/img/hello_page.jpg"
filename = "snowdeer.jpg"

urllib.request.urlretrieve(url, filename)
print("Saving image is successful.")
</pre>

<br>

## urlopen() 함수를 이용하여 메모리에 저장한 다음 파일에 저장

<pre class="prettyprint">
import urllib.request

url = "http://snowdeer.github.io/public/img/hello_page.jpg"
filename = "snowdeer.jpg"

image = urllib.request.urlopen(url).read()

with open(filename, mode="wb") as f:
    f.write(image)
    print("Saving image is successful.")
</pre>