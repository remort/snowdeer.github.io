---
layout: post
title: Django Template 및 View 구현하기
category: Python
tag: [Python, django]
---

## URL 과 View간 맵핑

Django에서 `URL`과 `View`는 1:1로 맵핑됨. 예를 들면 다음과 같은 형태로 맵핑할 수 있습니다.

---|---|---
URL 패턴 | View 이름 | 처리하는 내용
---|---|---
/posts/ | index() | index.html 템플릿을 렌더링
/posts/3 | detail() | 각 포스트의 내용을 detail.html 템플릿을 통해 렌더링

<br>

## urls.py 작성

`urls.py`에는 사이트로 요청된 URL을 이용해서 실행할 함수까지 맵핑을 할 수 있습니다. `urls.py`의 위치는 프로젝트 전체 영역에 해당하는 `blog` 디렉토리 아래에 위치하며, 각 어플리케이션마다 별도로 작성해서 관리할 수도 있습니다.

로직의 개발 순서는 정해지진 않았지만, URLConf -> View -> Template 형태로 개발하는 것이 일반적입니다.

여기서는 각 어플리케이션마다 `urls.py` 파일을 정의한 다음, 프로젝트 전체 URLConf에서 각 파일을 include 하도록 하는 예제가 포스팅되었습니다.

<br>

### post/urls.py

`urls.py` 파일이 존재하지 않기 때문에 생성해줍니다. 맵핑될 URL 포맷은 정규식으로 작성합니다.

<pre class="prettyprint">
from django.contrib import admin
from django.urls import path

from django.conf.urls import url
from post import views

app_name = 'post'

urlpatterns = [
    url(r"^$", views.index, name="index"),
    # url(r"^(?P&lt;post_id&gt;\d+)/$", views.detail, name="detail"),
]
</pre>

<br>

### blog/urls.py

그리고, 프로젝트 전체에 해당하는 `blog/urls.py`에는 다음과 같이 작성합니다.

<pre class="prettyprint">
from django.contrib import admin
from django.urls import path

from django.conf.urls import url, include
from post import views

urlpatterns = [
    url(r"^admin/", admin.site.urls),
    url(r"^post/", include("post.urls", namespace="post")),
]
</pre>

<br>

## view 작성

<pre class="prettyprint">
from django.shortcuts import render
from post.models import Post


# Create your views here.

def index(request):
    post_list = Post.objects.all().order_by("-pub_date")[:10]
    context = {"post_list": post_list}
    return render(request, "post/index.html", context)
</pre>

<br>

## template 작성

`post/templates/post/` 디렉토리 아래에 `index.html` 파일을 생성합니다.

{%raw%}
<pre class="prettyprint">
{% if post_list %}
    &lt;ul&gt;
        {% for post in post_list %}
            &lt;li&gt;&lt;a href="/post/{{ post.id }}/"&gt;{{ post.title }}&lt;/a&gt;&lt;/li&gt;
        {% endfor %}
    &lt;/ul&gt;

{% else %}
    &lt;p&gt;Empty List&lt;/p&gt;
{% endif %}
</pre>
{%endraw%}

<br>

## 확인

그 이후 `http://127.0.0.1:8000/admin` 사이트에서 Post 테이블의 아이템을 추가한다음 `http://127.0.0.1:8000/post`에서 아이템 리스트를 조회할 수 있습니다.

<br>

## 추가 화면 구성

### post/urls.py 수정

<pre class="prettyprint">
urlpatterns = [
    url(r"^$", views.index, name="index"),
    url(r"^(?P&lt;post_id&gt;\d+)/$", views.detail, name="detail"),
]
</pre>

<br>

### post/view.py 수정

<pre class="prettyprint">
from django.shortcuts import render, get_object_or_404
from post.models import Post


# Create your views here.

def index(request):
    post_list = Post.objects.all().order_by("-pub_date")[:10]
    context = {"post_list": post_list}
    return render(request, "post/index.html", context)


def detail(request, post_id):
    item = get_object_or_404(Post, pk=post_id)
    return render(request, "post/detail.html", {"post": item})
</pre>

<br>

### post/template/post/detail.html

{%raw%}
<pre class="prettyprint">
&lt;h1&gt;{{ post.title }}&lt;/h1&gt;
&lt;h3&gt;{{ post.text }}&lt;/h3&gt;
</pre>
{%endraw%}