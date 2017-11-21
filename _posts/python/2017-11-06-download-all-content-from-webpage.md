---
layout: post
title: 링크에 있는 컨텐츠를 전부 내려 받는 방법
category: Python
tag: [Python]
---

Python 3.x 기반의 코드입니다.

# 상대 경로 사용 방법

## 소스 코드
<pre class="prettyprint">
from urllib.parse import urljoin

baseUrl = "http://snowdeer.github.io/blog/categories/"

print(urljoin(baseUrl, "/python/2017/05/01/difference-between-python-2-7-x-and-python-3-x/"))
print(urljoin(baseUrl, "/python/2017/11/03/download-file-from-network/"))
print(urljoin(baseUrl, "/python/2017/11/04/get-html-body/"))
</pre>

<br>

## 실행 결과

~~~
http://snowdeer.github.io/python/2017/05/01/difference-between-python-2-7-x-and-python-3-x/
http://snowdeer.github.io/python/2017/11/03/download-file-from-network/
http://snowdeer.github.io/python/2017/11/04/get-html-body/
~~~

위의 예제와 같이 `baseUrl` 변수를 기본 URL 아래의 하위 카테고리 경로까지 지정했음에도, `urljoin`을 이용해서 만들어진 상대 경로들은 올바르게 만들어지는 것을 확인할 수 있습니다.

<br>

# 재귀적으로 하위 링크들의 컨텐츠를 모두 내려받는 코드

아래의 코드를 활용하면 특정 웹 페이지의 하위 링크들을 재귀적으로 방문하면서 모두 저장하게 됩니다. 단, `BeautifulSoup` 라이브러리를 이용하여 웹페이지 파싱을 통해 하위 링크들을 검색하기 때문에 저장하려는 대상 페이지에 따라 파싱 코드를 따로 작성을 해야 합니다.

## 소스 코드

<pre class="prettyprint">
from bs4 import BeautifulSoup
from urllib.request import *
from urllib.parse import *
from os import makedirs
import os.path, time, re

# 이미 처리가 끝난 파일인지 확인하기 위한 용도
finished_files = {}

# HTML 문서 내부의 링크 추출
def get_links_in_html(html, base):
    soup = BeautifulSoup(html, "html.parser")
    links = soup.select("link[rel='stylesheet']")
    links += soup.select("a[href]")
    result_list = []

    for a in links:
        href = a.attrs['href']
        url = urljoin(base, href)   # 링크를 절대 경로로 변환
        result_list.append(url)

    return result_list

# URL로부터 파일 다운로드
def download_file(url):
    out = urlparse(url)
    file_path = "./" + out.netloc + out.path
    if re.search(r"/$", file_path):
        file_path += "index.html"

    folder_path = os.path.dirname(file_path)

    if os.path.exists(file_path):
        return file_path

    if not os.path.exists(folder_path):
        print("make folder: ", folder_path)
        makedirs(folder_path)

    try:
        print("download file: ", url)
        urlretrieve(url, file_path)
        time.sleep(1)
        return file_path

    except:
        print("download failed: ", url)
        return None

def analyze_html(url, root_url):
    file_path = download_file(url)

    if file_path is None:
        return

    if file_path in finished_files:
        return

    finished_files[file_path] = True
    print("analyze_html: ", url)

    html = open(file_path, "r", encoding="utf-8").read()
    links = get_links_in_html(html, url)

    for link_url in links:
        if link_url.find(root_url) != 0:
            if not re.search(r".css$", link_url):
                continue

            if re.search(r".(html|htm)$", link_url):
                analyze_html(link_url, root_url)
                continue

            download_file(link_url)

if __name__ == "__main__":
    url = "https://docs.python.org/3.7/library/"
    analyze_html(url, url)
</pre>

<br>

## 실행 결과

~~~
make folder:  ./docs.python.org/3.7/library
download file:  https://docs.python.org/3.7/library/
analyze_html:  https://docs.python.org/3.7/library/
make folder:  ./docs.python.org/3.7/_static
download file:  https://docs.python.org/3.7/_static/pydoctheme.css
download file:  https://docs.python.org/3.7/_static/pygments.css
~~~