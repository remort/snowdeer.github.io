---
layout: post
title: HTML 문서 본문 가져오기
category: Python
tag: [Python]
---

Python 3.x 기반의 코드입니다. 웹페이지의 HTML 본문을 가져오는 코드입니다.

<br>

# 예제 코드

<pre class="prettyprint">
import urllib.request

url = "http://snowdeer.github.io/"

res = urllib.request.urlopen(url)
data = res.read()

text = data.decode("UTF-8")
print(text)
</pre>