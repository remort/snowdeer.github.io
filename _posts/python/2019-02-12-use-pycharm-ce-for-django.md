---
layout: post
title: Pycharm CE 버전을 이용한 Django 개발하기
category: Python
tag: [Python, django]
---

Pycharm 유료 버전은 `Django`를 정식으로 지원하지만, CE(Community Edition) 버전은 지원하지 않습니다.
하지만, 사용자 설정을 통해서 CE버전에서도 Django 빌드 및 디버깅을 할 수 있습니다.

<br>

## 프로젝트 생성

먼저 터미널에서 다음 명령어를 이용해서 `Django` 프로젝트를 생성합니다.
(프로젝트 생성은 그냥 콘솔에서 하는게 더 편합니다.)

<pre class="prettyprint">
django-admin.py startproject sample
</pre>

<br>

## Pycharm에서 해당 프로젝트 열기

위에서 생성한 프로젝트 폴더를 Pycharm을 통해서 엽니다.

<br>

## 실행 환경 구성

바로 실행할 수 없기 때문에 아래 메뉴에 접근해서 실행 환경을 구성해야 합니다.

Run > Edit Configurations

실행 환경 창이 뜨면 좌측 상단의 `+` 버튼을 눌러서 새로운 항목을 추가합니다. `Python` 항목을 선택합니다.

* Name: runserver(아무거나 원하는대로 정하면 된다.)
* Script Path: /Users/snowdeer/Workspace/python/django/sample/manage.py
* Parameters: runserver

