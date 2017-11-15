---
layout: post
title: 기상청 날씨 정보를 HTML 형태로 가져오기
category: Python
tag: [Python]
---

Python 3.x 기반의 코드입니다. 기상청 페이지의 HTML 데이터를 가져오는 예제입니다.
중간에 `stnId`라는 변수가 나오는데, 지역 코드입니다.

지역 | 지역 코드
--- | ---
전국 | 108
서울, 경기도 | 109
강원도 | 105
충청북도 | 131
충청남도 | 133
경상북도 | 143
전라북도 | 146
전라남도 | 156
경상남도 | 159
제주도 | 184

<br>

# 예제 코드

<pre class="prettyprint">
import urllib.request
import urllib.parse

REST_API = "http://www.kma.go.kr/weather/forecast/mid-term-rss3.jsp"

values = {
    'stnId': '108'
}
params = urllib.parse.urlencode(values)

url = REST_API + "?" + params

data = urllib.request.urlopen(url).read()

text = data.decode("UTF-8")
print(text)
</pre>