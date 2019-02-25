---
layout: post
title: Django 프로젝트 생성 방법
category: Python
tag: [Python, django]
---

`Django` 프로젝트를 생성하는 방법입니다.

<br>

## python 가상 환경 생성

가상 환경을 따로 구성하지 않아도 되지만, 여기서는 가상 환경 생성부터 포스팅합니다.

<pre class="prettyprint">
python3 -m venv snowdeer_env
</pre>

위 명령어를 수행하면 `snowdeer_env`라는 `virtualenv` 가상 환경을 생성합니다.
현재 디렉토리내에 `snowdeer_env`라는 서브 디렉토리가 생성됩니다.

<br>

## 활성화

다음 명령어를 실행하면 조금 전에 생성한 가상 환경을 활성화(activate)합니다.

<pre class="prettyprint">
source snowdeer_env/bin/activate

(snowdeer_env) $
</pre>

<br>

## Django 설치

<pre class="prettyprint">
(snowdeer_env) $ sudo pip3 install Django
</pre>

<br>

## Django 프로젝트 생성

<pre class="prettyprint">
mkdir snowdeer_blog
cd snowdeer_blog

django-admin.py startproject blog
</pre>

생성된 파일 구조는 다음과 같습니다.

<pre class="prettyprint">
$ tree snowdeer_blog

snowdeer_blog
└── blog
    ├── blog
    │   ├── __init__.py
    │   ├── settings.py
    │   ├── urls.py
    │   └── wsgi.py
    └── manage.py
</pre>

<br>

## 어플리케이션 생성

위에서 생성한 디렉토리 구조에서 `manage.py` 파일을 이용해서 나머지 작업을 수행할 수 있습니다.

<pre class="prettyprint">
$ python3 manage.py startapp post
</pre>

<br>

## Database 반영

Database는 `migrate`라는 명령어를 이용해서 생성 및 업데이트할 수 있습니다.

<pre class="prettyprint">
$ python3 manage.py migrate

Operations to perform:
  Apply all migrations: admin, auth, contenttypes, sessions
Running migrations:
  Applying contenttypes.0001_initial... OK
  Applying auth.0001_initial... OK
  Applying admin.0001_initial... OK
  Applying admin.0002_logentry_remove_auto_add... OK
  Applying admin.0003_logentry_add_action_flag_choices... OK
  Applying contenttypes.0002_remove_content_type_name... OK
  Applying auth.0002_alter_permission_name_max_length... OK
  Applying auth.0003_alter_user_email_max_length... OK
  Applying auth.0004_alter_user_username_opts... OK
  Applying auth.0005_alter_user_last_login_null... OK
  Applying auth.0006_require_contenttypes_0002... OK
  Applying auth.0007_alter_validators_add_error_messages... OK
  Applying auth.0008_alter_user_username_max_length... OK
  Applying auth.0009_alter_user_last_name_max_length... OK
  Applying sessions.0001_initial... OK
</pre>

<br>

## 실행 및 브라우저에서 확인

<pre class="prettyprint">
$ python3 manage.py runserver

Performing system checks...

System check identified no issues (0 silenced).
January 30, 2019 - 01:27:06
Django version 2.1.5, using settings 'blog.settings'
Starting development server at http://127.0.0.1:8000/
Quit the server with CONTROL-C.
</pre>

그 이후 브라우저에서 `http://127.0.0.1:8000` 또는 `http://127.0.0.1:8000/admin`으로 접속하면 확인할 수 있습니다.

<br>

## admin 관리자 등록

<pre class="prettyprint">
$ python3 manage.py createsuperuser
</pre>

그 이후 `http://127.0.0.1:8000/admin`에서 로그인 가능합니다.