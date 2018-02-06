---
layout: post
title: Django 프로젝트 생성하기
category: Python
tag: [Python, Django]
---
# Django 프로젝트 생성하는 방법

Django에서 프로젝트 뼈대는 다음 명령어를 이용해서 만들 수 있습니다. 

~~~
$ django-admin startproject mysite
~~~

위 명령어를 입력하면 현재 디렉토리에 `mysite`라는 이름의 디렉토리가 만들어지고 Django 프로젝트를 시작할 수 있게 됩니다.

<br>

## 프로젝트 구성

`tree`(Windows에서는 `tree /f`) 명령어를 이용하면 다음과 같은 디렉토리 구조가 만들어진 걸 확인할 수 있습니다.

~~~
C:.
└─mysite
    │  manage.py
    │
    └─mysite
            settings.py
            urls.py
            wsgi.py
            __init__.py
~~~

<br>

위에서 각 디렉토리 및 파일은 다음과 같은 역할을 합니다.

* mysite : 가장 바깥쪽의 디렉토리인 `mysite`는 Django와 아무 관련이 없는 디렉토리이며 다른 이름으로 바꿔도 상관없습니다.
* manage.py : Django의 다양한 명령어를 실행할 수 있게 해주는 커맨드라인 형태의 유틸리티입니다.
* mysite : 하위에 있는 `mysite` 디렉토리에 실질적인 프로젝트 파일들이 위치합니다.
* settings.py : 프로젝트의 환경 설정 파일입니다. 
* urls.py : 프로젝트 레벨의 URL 패턴을 정의하는 URLConf입니다.
* wsgi.py : Apache 등과 같은 상용 웹 서버와 WSGI 규격으로 연동할 수 있게 해주는 파일입니다.
* __init__.py : 이 디렉토리가 Python 패키지임을 알려주는 빈 파일입니다.

<br>

## Django 어플리케이션 생성

하나의 프로젝트 안에는 여러 개의 어플리케이션을 만들 수 있습니다. 다음 명령어를 이용하여 'books'라는 어플리케이션을 생성할 수 있습니다.

~~~
$ cd mysite

$ python manage.py startapp books
~~~

위 명령어를 실행하면 프로젝트는 다음과 같은 구조가 됩니다.

~~~
C:.
└─mysite
    │  manage.py
    │
    ├─books
    │  │  admin.py
    │  │  apps.py
    │  │  models.py
    │  │  tests.py
    │  │  views.py
    │  │  __init__.py
    │  │
    │  └─migrations
    │          __init__.py
    │
    └─mysite
        │  settings.py
        │  urls.py
        │  wsgi.py
        │  __init__.py
        │
        └─__pycache__
                settings.cpython-35.pyc
                __init__.cpython-35.pyc
~~~

<br>

## 데이터베이스에 변경 사항 반영

다음 명령어로 데이터베이스에 변경 사항을 반영할 수 있습니다.

~~~
$ python manage.py migrate

Operations to perform:
  Apply all migrations: admin, auth, contenttypes, sessions
Running migrations:
  Applying contenttypes.0001_initial... OK
  Applying auth.0001_initial... OK
  Applying admin.0001_initial... OK
  Applying admin.0002_logentry_remove_auto_add... OK
  Applying contenttypes.0002_remove_content_type_name... OK
  Applying auth.0002_alter_permission_name_max_length... OK
  Applying auth.0003_alter_user_email_max_length... OK
  Applying auth.0004_alter_user_username_opts... OK
  Applying auth.0005_alter_user_last_login_null... OK
  Applying auth.0006_require_contenttypes_0002... OK
  Applying auth.0007_alter_validators_add_error_messages... OK
  Applying auth.0008_alter_user_username_max_length... OK
  Applying auth.0009_alter_user_last_name_max_length... OK
  Applying auth.0010_auto_20180201_0911... OK
  Applying sessions.0001_initial... OK
~~~

<br>

## Django 프로젝트 실행

~~~
$ python manage.py runserver
~~~