---
layout: post
title: BeautifulSoup 라이브러리 활용 웹페이지 파싱(Parsing)
category: Python
tag: [Python]
---

Python 3.x 기반의 코드입니다.

# BeautifulSoup 라이브러리 설치

`pip`를 이용해서 설치합니다. 콘솔창에서 다음 명령어를 입력합니다.

~~~
> pip install beautifulsoup4
~~~

<br>

# 예제 코드

## 기본적인 사용법

<pre class="prettyprint">
from bs4 import BeautifulSoup

html = &quot;&quot;&quot;
&lt;html&gt;
    &lt;body&gt;
        &lt;h1&gt;Hello, BeautifulSoup&lt;/h1&gt;
        &lt;p&gt;This is a example.&lt;/p&gt;
        &lt;p&gt;BeautifulSoup helps to scrap web page easily.&lt;/p&gt;
    &lt;/body&gt;
</html>
&quot;&quot;&quot;

soup = BeautifulSoup(html, "html.parser")

h1 = soup.html.body.h1
p1 = soup.html.body.p
p2 = p1.next_sibling.next_sibling

print("h1 = " + h1.string)
print("p1 = " + p1.string)
print("p1 = " + p2.string)
</pre>

<br>

## id 요소를 활용한 파싱

<pre class="prettyprint">
from bs4 import BeautifulSoup

html = &quot;&quot;&quot;
&lt;html&gt;
    &lt;body&gt;
        &lt;h1&gt;Hello, BeautifulSoup&lt;/h1&gt;
        &lt;p&gt;This is a example.&lt;/p&gt;
        &lt;p&gt;BeautifulSoup helps to scrap web page easily.&lt;/p&gt;
    &lt;/body&gt;
</html>
&quot;&quot;&quot;

soup = BeautifulSoup(html, "html.parser")

h1 = soup.find(id="title")
p1 = soup.find(id="first")

print("h1 = " + h1.string)
print("p1 = " + p1.string)
</pre>

<br>

또한 아래의 코드를 이용하여 파싱이 잘 되었는지 확인할 수 있습니다.

<pre class="prettyprint">
soup = BeautifulSoup(html, "html.parser")
print(soup.prettify())
</pre>

<br>

## 기상청 페이지 정보 파싱하기

<pre class="prettyprint">
import urllib.request as req
from bs4 import BeautifulSoup

REST_API = "http://www.kma.go.kr/weather/forecast/mid-term-rss3.jsp"
values = {
    'stnId': '108'
}
url = REST_API + "?" + "stnId=108"
res = req.urlopen(url)

soup = BeautifulSoup(res, "html.parser")

title = soup.find("title")
wf = soup.find("wf")

print("title = " + title.string)
print("wf = " + wf.string)
</pre>