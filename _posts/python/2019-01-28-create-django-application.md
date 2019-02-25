---
layout: post
title: Django 어플리케이션 생성 방법
category: Python
tag: [Python, django]
---

Django 프로젝트 생성 이후 그 안에 어플리케이션을 생성하는 방법입니다.
다음은 `post`라는 어플리케이션을 생성한 다음 이루어지는 작업들입니다.

<pre class="prettyprint">
$ python3 manage.py startapp post
</pre>

<br>

그 다음에는 어플리케이션에서 사용할 데이터 모델(Model)을 생성합니다.

Django는 기본적으로 `SQLite3` 데이터베이스를 사용하며, `settings.py` 파일에서 다른 데이터베이스를 사용할 수도 있습니다.
`settings.py`에는 다음과 같이 데이터베이스가 설정되어 있습니다.

<pre class="prettyprint">
# Database
# https://docs.djangoproject.com/en/2.1/ref/settings/#databases

DATABASES = {
    'default': {
        'ENGINE': 'django.db.backends.sqlite3',
        'NAME': os.path.join(BASE_DIR, 'db.sqlite3'),
    }
}
</pre>

<br>

## post 어플리케이션을 settings.py에 등록하기

방금 생성한 `post` 어플리케이션을 `setting.py` 내에 다음과 같이 등록합니다.

<pre class="prettyprint">
INSTALLED_APPS = [
    'django.contrib.admin',
    'django.contrib.auth',
    'django.contrib.contenttypes',
    'django.contrib.sessions',
    'django.contrib.messages',
    'django.contrib.staticfiles',
    'post',
]
</pre>

<br>

## 타임존(TimeZone) 변경

`settings.py`내 타임존 부분을 변경합니다.

<pre class="prettyprint">
TIME_ZONE = 'Asia/Seoul'
</pre>

<br>

## 데이터베이스 테이블(Table) 정의

위에서 생성한 `post` 어플리케이션내 `models.py` 파일에서 테이블을 정의할 수 있습니다.

<pre class="prettyprint">
from django.db import models


# Create your models here.

class Post(models.Model):
    pub_date = models.DateTimeField('date published')
    title = models.CharField(max_length=20)
    text = models.CharField(max_length=300)

    def __str__(self):
        return self.text
</pre>

<br>

## admin.py에 모델 반영

<pre class="prettyprint">
from django.contrib import admin
from post.models import Post

# Register your models here.
admin.site.register(Post)
</pre>

<br>

그 이후 `manage.py` 파일의 `makemigrations` 명령어를 실행해서 데이터베이션에 변경사항을 반영(migration)합니다.

<pre class="prettyprint">
$ python3 manage.py makemigrations

Migrations for 'post':
  post/migrations/0001_initial.py
    - Create model Post



$ python3 manage.py migrate

Operations to perform:
  Apply all migrations: admin, auth, contenttypes, post, sessions
Running migrations:
  Applying post.0001_initial... OK
</pre>

그 이후 브라우저에서 `http://127.0.0.1:8000/admin`에 접속하면 방금 작업한 테이블이 잘 반영되어 있는 것을 확인할 수 있습니다.
